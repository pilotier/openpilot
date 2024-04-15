import zmq
import cv2
import numpy as np
import time

def test_zmq(image_path, zmq_addr):
  context = zmq.Context()
  socket = context.socket(zmq.PUB)
  socket.bind(zmq_addr)
  im = cv2.imread(image_path, cv2.IMREAD_COLOR)
  if im is None:
    print("Image not found")
    return
  _, im = cv2.imencode('.jpg', im)
  while True:
    socket.send(im.tobytes())
    time.sleep(1/30)

test_zmq("test_img.jpg", "tcp://127.0.0.1:6666")
