import argparse
import time
import cv2
from pibot_client import PiBot

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='PiBot client')
    parser.add_argument('--ip', type=str, default='localhost', help='IP address of PiBot')
    args = parser.parse_args()

    bot = PiBot(ip=args.ip)

    print(f'Voltage: {bot.getVoltage():.2f}V')
    print(f'Current: {bot.getCurrent():.2f}A')

    enc_begin_left, enc_begin_right = bot.getEncoders()
    print(f"get encoders state at beginning: {enc_begin_left}, {enc_begin_right}")

    print("test left motor")
    bot.setVelocity(10,0)
    time.sleep(2)

    print("test right motor")
    bot.setVelocity(0,10)
    time.sleep(2)

    print("stop")
    bot.setVelocity(0,0)

    enc_end_left, enc_end_right = bot.getEncoders()
    print(f"get encoders state at end: {enc_end_left}, {enc_end_right}")

    print("initialise camera")
    time.sleep(2)
    print("grab image")
    image = bot.getImage()
    print(f"image size {image.shape[0]} by {image.shape[1]}")

    try:
        while True:
            cv2.imshow('image', image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            image = bot.getImage()
    except KeyboardInterrupt:
        exit()

