#!/usr/bin/env python3

import time
import piVideoStream

IM_WIDTH=640
IM_HEIGHT=480

if __name__=="__main__":
    stream = io.BytesIO()
    camera = piVideoStream.PiVideoStream(
            resolution=(IM_WIDTH, IM_HEIGHT),
            framerate=32
        )

    camera.start()

    while not camera.frame_available:
        continue
