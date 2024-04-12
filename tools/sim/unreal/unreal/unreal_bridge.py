#!/usr/bin/env python3
import time
import math
import atexit
import cv2
import numpy as np
import threading
import argparse
import capnp

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


W, H = 1164, 874
REPEAT_COUNTER = 5
PRINT_DECIMATION = 100


pm = messaging.PubMaster(['frame', 'sensorEvents', 'can'])
sm = messaging.SubMaster(['carControl', 'controlsState'])

class CamFaker:
    def __init__(self):
        self.camerad = Camerad(False)
    def send_camera_images(self):
        #test_img = np.zeros((1208,1928,3),np.uint8)
        #imarray = (np.random.rand(1208,1928,3) * 255).astype(np.uint8)
        im = cv2.imread("test_img.jpg")
        yuv = self.camerad.rgb_to_yuv(im)
        self.camerad.cam_send_yuv_road(yuv)




def go():


    # can loop
    rk = Ratekeeper(100, None)
    simulator_state = SimulatorState()
    simulated_car = SimulatedCar()
    simulated_sensors = SimulatedSensors(False)
    simulator_state.valid = True

    cam_faker = CamFaker()
    while 1:
        simulated_car.update(simulator_state)
        simulated_sensors.update(simulator_state)
        cam_faker.send_camera_images()
        print("Loop")
        rk.keep_time



if __name__ == "__main__":

    # make sure params are in a good state
    params = Params()
    params.clear_all()
    set_params_enabled()
    # params.put("HasAcceptedTerms", '1')
    # uncomment to skip calibration
    # msg = messaging.new_message('liveCalibration')
    # msg.liveCalibration.validBlocks = 20
    # msg.liveCalibration.rpyCalib = [0.0, 0.0, 0.0]
    # params.put("CalibrationParams", msg.to_bytes())
    process_sim = multiprocessing.Process(name="metadrive process", target=functools.partial(go))

    process_sim.start()
    sm = messaging.SubMaster(['liveLocationKalman'])
    while True:
        llk = sm['liveLocationKalman']
        print(llk)
        time.sleep(1)

    # start input poll for keyboard
    # from lib.keyboard_ctrl import keyboard_poll_thread
    # keyboard_poll_thread(q)



