import pygame
import serial
from time import sleep

pygame.init()
js=pygame.joystick.Joystick(0)
js.init()

ser = serial.Serial('/dev/tty.usbmodem1411', 9600)
print js.get_numbuttons()  # perhaps coincidentally correctly prints 17 which is the number of buttons on a PS3 controller
try:
  while True:
    pygame.event.pump()
    axis_val = js.get_axis(0)
    if axis_val < -0.5:
      print "LEFT", axis_val
      ser.write('l')
    elif axis_val > 0.5:
      print "RIGHT", axis_val
      ser.write('r')
    # prevent sending faster than the controller can handle
    sleep(0.02)
except KeyboardInterrupt as ki:
  ser.close()
