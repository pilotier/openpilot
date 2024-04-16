#!/usr/bin/env python3
import time
import math
import atexit
import cv2
import numpy as np
import threading
import argparse
import capnp
import zmq
import json

import cereal.messaging as messaging
from common.params import Params
from common.realtime import Ratekeeper, DT_DMON
from openpilot.tools.sim.lib.common import SimulatorState
from openpilot.tools.sim.lib.common import World
from openpilot.tools.sim.lib.camerad import Camerad
import multiprocessing
import numpy as np
import time
import functools
from multiprocessing import Pipe, Array

from openpilot.tools.sim.lib.simulated_car import SimulatedCar
from openpilot.tools.sim.lib.simulated_sensors import SimulatedSensors
from selfdrive.car.honda.values import CruiseButtons
from selfdrive.test.helpers import set_params_enabled
from collections import namedtuple


W, H = 1928, 1208
REPEAT_COUNTER = 5
PRINT_DECIMATION = 100


sm = messaging.SubMaster(['carControl', 'controlsState'])
from openpilot.common.mock.generators import sim_liveLocationKalman
vec3 = namedtuple("vec3", ["x", "y", "z"])

CURRENT_LOCATION = (53.53834372303548, -113.51946368610008)


def generate_messages_loop(gps_queue: 'multiprocessing.Queue[(float,float, float)]'):
    pm = messaging.PubMaster(['liveLocationKalman'])
    rk = Ratekeeper(20, 0.05)
    prev_location = (53.53834372303548, -113.51946368610008)
    prev_heading = 0

    while 1:
        if not gps_queue.empty():
            data = gps_queue.get()
            location = (data[0], data[1])
            heading = data[2]
            prev_location = location
            prev_heading = heading
        else:
            location = prev_location
            heading = prev_heading
        message = sim_liveLocationKalman(location, heading)
        pm.send('liveLocationKalman', message)
        #LOCATION1 = (LOCATION1[0] + 0.000001, LOCATION1[1] + 0.000)
        rk.keep_time()


class CamFaker:
    def __init__(self):
        self.camerad = Camerad(False)
    def send_camera_images(self, q: 'multiprocessing.Queue[(float,float, float)]', speed_queue: 'multiprocessing.Queue[float]'):

        #test_img = np.zeros((1208,1928,3),np.uint8)
        #imarray = (np.random.rand(1208,1928,3) * 255).astype(np.uint8)

        im = cv2.imread("test_img.jpg")
        yuv = self.camerad.rgb_to_yuv(im)

        # print("sending image")
        zmq_handler = ZmqHandler("tcp://192.168.0.13:8765","")
        rate = 20
        old_im = None
        while 1:
            starttime = time.time()
            try:
                (im, jdata) = zmq_handler.get_latest_packet()
            except Exception as e:
                continue
            if im is not None and jdata is not None:
                yuv = self.camerad.rgb_to_yuv(im)
                self.camerad.cam_send_yuv_road(yuv)
                lat = jdata['data']['lat']
                lon = jdata['data']['lng']
                heading = jdata['data']['heading']
                speed = jdata['data']['speed']
                q.put((lat, lon, heading))
                speed_queue.put(speed)
                old_im = im
            else:
                self.camerad.cam_send_yuv_road(yuv)
            elapsed = time.time() - starttime
            if elapsed < 1/rate:
                time.sleep(1/rate - elapsed)


class ZmqHandler:
    def __init__(self, address, topic):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, topic)
        self.topic = topic
    def get_latest_json(self):
        try:
            in_data = None
            while in_data is None:
                try:
                    in_data = self.socket.recv_json(zmq.NOBLOCK)
                except zmq.Again:
                    break
        except json.JSONDecodeError:
            print("json decode error")
        return in_data
    def get_latest_image(self):
        try:
            raw_image_data = b""
            while True:
                try:
                    chunk = self.socket.recv(zmq.NOBLOCK)
                    print(len(chunk))
                    if not chunk:
                        break
                    raw_image_data += chunk
                except zmq.Again:
                    break
            if not raw_image_data:
                return None
            nparr = np.frombuffer(raw_image_data, np.uint8)
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            resized_image = cv2.resize(image, (W, H))
            return resized_image
        except Exception as e:
            print(e)
            return None

    def get_latest_packet(self):
        try:
            raw_image_data = b""
            chunk = []
            while len(chunk) < 3:
                try:
                    chunk = self.socket.recv_multipart(zmq.NOBLOCK)
                except zmq.Again:
                    return None
            raw_image_data += chunk[2]
            if len(raw_image_data) != W * H * 4:
                return None
            image = np.frombuffer(raw_image_data, dtype=np.uint8).reshape(H, W, 4)
            image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
            chunk1_json = json.loads(chunk[1])
            return (image, chunk1_json)
        except Exception as e:
            print(e)
            return None



def sensor_spoof(q: 'multiprocessing.Queue[str]', gps_queue: 'multiprocessing.Queue[(float,float, float)]', speed_queue: 'multiprocessing.Queue[float]'):


    # can loop
    rk = Ratekeeper(100, 0.05)
    simulator_state = SimulatorState()
    simulated_car = SimulatedCar()
    simulated_sensors = SimulatedSensors(False)
    simulator_state.valid = True
    cam_faker = CamFaker()

    simulated_cam_thread = threading.Thread(target=cam_faker.send_camera_images, args=(gps_queue,speed_queue))
    simulated_cam_thread.start()
    count = 0
    while 1:
        if simulator_state.cruise_button == CruiseButtons.DECEL_SET:
            count += 1
            if count == 100:
                simulator_state.cruise_button = CruiseButtons.MAIN
                count = 0
        if not q.empty():
            msg = q.get()
            if msg == "cruise_up":
                simulator_state.cruise_button = CruiseButtons.RES_ACCEL
            elif msg == "cruise_down":
                simulator_state.cruise_button = CruiseButtons.DECEL_SET
                simulator_state.is_engaged = True
                print("cruise_down")
            elif msg == "cruise_cancel":
                simulator_state.cruise_button = CruiseButtons.CANCEL
                simulator_state.is_engaged = False
        if not speed_queue.empty():
            data = speed_queue.get()
            velocity = vec3(data,0,0)
            simulator_state.velocity = velocity
        simulated_car.update(simulator_state)
        simulated_sensors.update(simulator_state, None)
        simulated_car.sm.update(0)


        #cam_faker.send_camera_images()
        rk.keep_time



if __name__ == "__main__":

    # make sure params are in a good state
    params = Params()
    #params.clear_all()
    set_params_enabled()
    params.put_bool("ExperimentalLongitudinalEnabled", True)
    queue = multiprocessing.Queue()
    gps_queue = multiprocessing.Queue()
    speed_queue = multiprocessing.Queue()

    from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread

    process_sim = multiprocessing.Process(name="metadrive process", target=functools.partial(sensor_spoof,queue, gps_queue, speed_queue))
    process_km = multiprocessing.Process(name="kalmann process", target=functools.partial(generate_messages_loop, gps_queue))
    process_sim.start()
    process_km.start()

    keyboard_poll_thread(queue)


    # start input poll for keyboard
    # from lib.keyboard_ctrl import keybthreard_poll_thread
    # keyboard_poll_thread(q)



