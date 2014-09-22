import pygame
import serial
from time import sleep

pygame.init()
js=pygame.joystick.Joystick(0)
js.init()

ser = serial.Serial('/dev/tty.usbmodem12341', 9600)
print js.get_numbuttons()  # perhaps coincidentally correctly prints 17 which is the number of buttons on a PS3 controller
try:
  state = 's'
  writeState = True
  while True:
    pygame.event.pump()
    axis_val = js.get_axis(0)
    if axis_val < -0.5:
      print "LEFT", axis_val
      writeState = state != 'l'
      state = 'l'
    elif axis_val > 0.5:
      print "RIGHT", axis_val
      writeState = state != 'r'
      state = 'r'
    else:
      writeState = state != 's'
      state = 's'
    if writeState:
        ser.write(state)
        writeState = False
    # prevent sending faster than the controller can handle
    sleep(0.02)
except KeyboardInterrupt as ki:
  ser.close()
