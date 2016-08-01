#!usr/bin/env python
from __future__ import division
import rospy, rosbag, time, csv
from ocean_optics.msg import Spectrum
import matplotlib.pyplot as plt
import Tkinter as tk
import numpy as np

class Preview(object):
    def __init__(self):
        self.spectrum = []
        self.wavelengths = []
        self.initAnimation()
        while not rospy.is_shutdown():
            self.getData()

    def initAnimation(self):
        self.fig = plt.figure()
        self.l, = plt.plot([], [])

    def getData(self):
        rospy.init_node('subscriber')
        self.subscriber = rospy.Subscriber("/spectrometer/spectrum", Spectrum,
                                            self.getDataWrapper)

    def getDataWrapper(self, data):
        self.spectrum = data.spectrum
        self.wavelengths = data.wavelengths
        self.subscriber.unregister()
        self.l.set_data([self.wavelengths, self.spectrum])
        plt.show(block = False)