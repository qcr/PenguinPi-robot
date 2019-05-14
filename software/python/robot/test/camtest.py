import picamera

IM_WIDTH = 320
IM_HEIGHT = 240

camera = picamera.PiCamera()
camera.resolution = (IM_WIDTH, IM_HEIGHT)

camera.capture('z.png', format='png')
