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


W, H = 1928, 1208
REPEAT_COUNTER = 5
PRINT_DECIMATION = 100


sm = messaging.SubMaster(['carControl', 'controlsState'])
from openpilot.common.mock.generators import generate_liveLocationKalman




def generate_messages_loop():
    pm = messaging.PubMaster(['liveLocationKalman'])
    rk = Ratekeeper(20, 0.05)
    LOCATION1 = (53.53834372303548, -113.51946368610008)

    while 1:
        message = generate_liveLocationKalman(LOCATION1)
        pm.send('liveLocationKalman', message)
        #LOCATION1 = (LOCATION1[0] + 0.000001, LOCATION1[1] + 0.000)
        rk.keep_time()


class CamFaker:
    def __init__(self):
        self.camerad = Camerad(False)
    def send_camera_images(self):

        #test_img = np.zeros((1208,1928,3),np.uint8)
        #imarray = (np.random.rand(1208,1928,3) * 255).astype(np.uint8)
        rk = Ratekeeper(20, 0.05)

        im = cv2.imread("test_img.jpg")
        yuv = self.camerad.rgb_to_yuv(im)

        # print("sending image")
        #zmq_handler = ZmqHandler("tcp://127.0.0.1:6666","")

        while 1:
            #im = zmq_handler.get_latest_image()
            if im is not None:
                #yuv = self.camerad.rgb_to_yuv(im)
                self.camerad.cam_send_yuv_road(yuv)

            rk.keep_time()


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

def sensor_spoof(q: 'multiprocessing.Queue[str]'):


    # can loop
    rk = Ratekeeper(100, 0.05)
    simulator_state = SimulatorState()
    simulated_car = SimulatedCar()
    simulated_sensors = SimulatedSensors(False)
    simulator_state.valid = True
    cam_faker = CamFaker()

    simulated_cam_thread = threading.Thread(target=cam_faker.send_camera_images)
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
        simulated_car.update(simulator_state)
        simulated_sensors.update(simulator_state)
        #cam_faker.send_camera_images()
        rk.keep_time



if __name__ == "__main__":

    # make sure params are in a good state
    # params = Params()
    # params.clear_all()
    # set_params_enabled()
    queue = multiprocessing.Queue()

    from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread

    process_sim = multiprocessing.Process(name="metadrive process", target=functools.partial(sensor_spoof,queue))
    process_km = multiprocessing.Process(name="kalmann process", target=functools.partial(generate_messages_loop))
    process_sim.start()
    process_km.start()

    keyboard_poll_thread(queue)


    # start input poll for keyboard
    # from lib.keyboard_ctrl import keybthreard_poll_thread
    # keyboard_poll_thread(q)



