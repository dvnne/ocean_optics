#!usr/bin/env python
from __future__ import division
import rospy, rosbag, time, csv
from ocean_optics.msg import Spectrum
import matplotlib.pyplot as plt
# import Tkinter as tk
import numpy as np

class SpectrumWriter(object):
    def __init__(self):
        self.wavelengths = None
        self.spectrum = None
        self.data = list()
        self.first_callback = True

    def getData(self, prefix = '', mode = 'single'):
        rospy.init_node('subscriber')
        self.subscriber = rospy.Subscriber("/spectrometer/spectrum", Spectrum,
                                lambda data: self.getDataWrapper(data, mode = mode))
        if mode != 'single':
            rospy.spin()

    def getDataWrapper(self, data, mode = 'single'):
        self.timestamp = time.asctime()
        self.spectrum = list(data.spectrum)
        self.wavelengths = list(data.wavelengths)
        if mode == 'single':
            self.subscriber.unregister()
        self.makeline()
        self.first_callback = False

    def makeline(self):
        if self.first_callback == True:
            self.data.append(['wavelengths'] + self.wavelengths)
        self.data.append([self.timestamp] + self.spectrum)
   
    def single(self, prefix = ''):
        self.getData(mode = 'single')

    def bag(self):
        self.getData(mode = 'bag')
        self.writeCSV()

    def writeCSV(self):
        with open('spectrum.csv', mode = 'w+b') as csvout:
            writer = csv.writer(csvout)
            # transform rows into cols
            self.data = np.transpose(np.array(self.data))
            for row in self.data:
                writer.writerow(row)

s = SpectrumWriter()
