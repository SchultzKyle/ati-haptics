#!/usr/bin/env python

import NetFT
import numpy as np
from time import sleep

if __name__ == '__main__':
    sensor = NetFT.Sensor("192.168.0.11")
    sensor.tare()
    sensor.startStreaming(handler=True)
    while True:
        measurement = sensor.receive()
        z_force = measurement[2] / 1e6
        print(z_force)
        sleep(0.01)