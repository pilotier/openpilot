#!/usr/bin/env python3
import cereal.messaging as messaging
from opendbc.can.packer import CANPacker
from openpilot.tools.sim.lib.common import SimulatorState
from selfdrive.car.honda.values import CAR


packer = CANPacker("honda_civic_touring_2016_can_generated")
rpacker = CANPacker("acura_ilx_2016_nidec")



def sendcan_function(sendcan):
  sc = messaging.drain_sock_raw(sendcan)
  cp.update_strings(sc, sendcan=True)

  if cp.vl[0x1fa]['COMPUTER_BRAKE_REQUEST']:
    brake = cp.vl[0x1fa]['COMPUTER_BRAKE'] / 1024.
  else:
    brake = 0.0

  if cp.vl[0x200]['GAS_COMMAND'] > 0:
    gas = ( cp.vl[0x200]['GAS_COMMAND'] + 83.3 ) / (0.253984064 * 2**16)
  else:
    gas = 0.0

  if cp.vl[0xe4]['STEER_TORQUE_REQUEST']:
    steer_torque = cp.vl[0xe4]['STEER_TORQUE']/3840
  else:
    steer_torque = 0.0

  return gas, brake, steer_torque