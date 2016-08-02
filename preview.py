#!usr/bin/env python
from __future__ import division
import rospy, rosbag, time, csv
from ocean_optics.msg import Spectrum
import matplotlib.pyplot as plt
import Tkinter as tk
import numpy as np

class Preview(object):
    def __init__(self):
        plt.ion()
        self.spectrum = []
        self.wavelengths = []
        self.initAnimation()
        # self.getData()

    def initAnimation(self):
        self.l, = plt.plot([], [])

    def getData(self):
        rospy.init_node('subscriber')
        self.subscriber = rospy.Subscriber("/spectrometer/spectrum", Spectrum,
                                            self.getDataWrapper)

    def getDataWrapper(self, data):
        print 'got data'
        self.subscriber.unregister()
        self.spectrum = data.spectrum
        self.wavelengths = data.wavelengths
        self.redraw()

    def redraw(self):
        self.l.set_data([self.wavelengths, self.spectrum])
        plt.plot(self.wavelengths, self.spectrum)
        plt.draw()


p = Preview()