from cereal import messaging

LOCATION1 = (53.53834372303548, -113.51946368610008)
LOCATION2 = (32.7558, -117.2037)

LLK_DECIMATION = 10
RENDER_FRAMES = 15
DEFAULT_ITERATIONS = RENDER_FRAMES * LLK_DECIMATION


def generate_liveLocationKalman(location=LOCATION1):
  msg = messaging.new_message('liveLocationKalman')
  msg.liveLocationKalman.positionGeodetic = {'value': [*location, 0], 'std': [0., 0., 0.], 'valid': True}
  msg.liveLocationKalman.positionECEF = {'value': [0., 0., 0.], 'std': [0., 0., 0.], 'valid': True}
  msg.liveLocationKalman.calibratedOrientationNED = {'value': [0., 0., 1.8], 'std': [0., 0., 0.], 'valid': True}
  msg.liveLocationKalman.velocityCalibrated = {'value': [0., 0., 0.], 'std': [0., 0., 0.], 'valid': True}
  msg.liveLocationKalman.status = 'valid'
  msg.liveLocationKalman.gpsOK = True
  return msg


def sim_liveLocationKalman(location=0.0, yaw=0.0):
  msg = messaging.new_message('liveLocationKalman')
  msg.liveLocationKalman.positionGeodetic = {'value': [*location, 0], 'std': [0., 0., 0.], 'valid': True}
  msg.liveLocationKalman.positionECEF = {'value': [0., 0., 0.], 'std': [0., 0., 0.], 'valid': True}
  msg.liveLocationKalman.calibratedOrientationNED = {'value': [0., 0., yaw], 'std': [0., 0., 0.], 'valid': True}
  msg.liveLocationKalman.velocityCalibrated = {'value': [0., 0., 0.], 'std': [0., 0., 0.], 'valid': True}
  msg.liveLocationKalman.status = 'valid'
  msg.liveLocationKalman.gpsOK = True
  return msg