#!/usr/bin/env python

import NetFT
import rospy
import numpy as np
from time import sleep
import multiprocessing
from std_msgs.msg import Float64MultiArray

forces_list = [0]*3
ati_publisher = rospy.Publisher('/ati/forces', Float64MultiArray, queue_size=1000)

def init_process(sensor,initialized):
    # for initializing network error handling
    try:
        sensor.tare() # tests connection, won't tare from inside process
        initialized.value = 1
    except:
        initialized.value = 0

def receive(sensor,forces):
    # for network disconnection error handling
    try:
        measurement = np.array(sensor.receive()[0:3]) / 1e6
        for i in range(len(forces)):
            forces[i] = measurement[i]
    except:
        pass

def read_sensor():
    rospy.init_node('ati_sensor', anonymous=True)
    initialized = multiprocessing.Value('i', 0)
    print(' ')
    print('Connecting to ATI...')
    print(' ')
    while not initialized.value:
        sensor = NetFT.Sensor("192.168.0.11")
        p = multiprocessing.Process(target=init_process, name="Initialization", args=(sensor,initialized))
        p.start()
        p.join(0.5)
        sleep(5.0)
    sensor.tare()
    sensor.startStreaming(handler=True)
    print(' ')
    print('Successfully streaming ATI data...')
    print(' ')
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        forces = multiprocessing.Array('d', [0]*3)
        p = multiprocessing.Process(target=receive, name="Receive", args=(sensor,forces))
        p.start()
        p.join(0.009)
        if not all(force == 0 for force in forces):
            global forces_list
            for i in range(len(forces)):
                forces_list[i] = forces[i]
        msg.data = forces_list
        ati_publisher.publish(msg)
        rate.sleep()
    return 0
        
if __name__ == '__main__':
    try:
        read_sensor()
    except rospy.ROSInterruptException:
        pass
