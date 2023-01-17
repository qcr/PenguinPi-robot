#!/usr/bin/env python

import penguinPi as ppi
import time

ppi.init()

oled = ppi.OLED('AD_OLED')

oled.set_ip_eth( '1.2.3.4' )
oled.set_ip_wlan( '10.11.12.13' )
