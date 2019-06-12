#NAME: flashLights.py
#DATE: 12/06/2019
#AUTH: Ryan McCartney
#DESC: Python function to flash farmbot lights
#COPY: Copyright 2019, All Rights Reserved, Ryan McCartney

from farmbot import Farmbot
import time

ip_address = "192.168.0.50"
farmbot = Farmbot(ip_address)
delay = 1

while farmbot.connected:
    farmbot.lightOff()
    time.sleep(delay)
    farmbot.lightOn()
    time.sleep(delay)
