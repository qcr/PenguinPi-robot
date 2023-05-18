#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd ) #Find dir of this script

python $SCRIPT_DIR/../python/robot/ppweb.py &
python $SCRIPT_DIR/GPIOSoftShutdown.py &
