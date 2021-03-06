#!usr/bin/env python
from __future__ import division
import rospy, rosbag, time, csv
from ocean_optics.msg import Spectrum
import matplotlib.pyplot as plt
import Tkinter as tk
import numpy as np

class Reflectance(object):
    def __init__(self):
        # self.initAnimation() # later
        self.bg = None
        self.ref = None
        self.spectrum = None
        self.wavelengths = None
        self.data = list()
        self.first_callback = True
    
    # grab a spectrum and return to the shell prompt
    def getData(self, prefix = ''):
        rospy.init_node('subscriber')
        self.subscriber = rospy.Subscriber("/spectrometer/spectrum", Spectrum,
                                            self.getDataWrapper)

    def getDataWrapper(self, data):
        self.timestamp = time.asctime()
        self.spectrum = data.spectrum
        self.wavelengths = data.wavelengths
        self.subscriber.unregister()
        self.makeline()

    def makeline(self):
        if self.first_callback == True:
            self.data.append(['wavelengths'] + self.wavelengths)
        self.data.append([self.timestamp] + self.spectrum

    def writeAll(self):
        with open('spectrum.csv', mode = 'w+b') as csvout:
            writer = csv.writer(csvout)
            # transform rows into cols
            self.data = np.transpose(np.array(self.data))
            for row in self.data:
                writer.writerow(row)

    # similar to the above, but writes data
    def writeData(self):
        rospy.init_node('subscriber')
        self.writer = rospy.Subscriber("/spectrometer/spectrum", Spectrum, self.callback)
        rospy.spin()

    def callback(self, data):
        if self.first_callback == True:
            self.data.append(data.wavelengths)
            self.first_callback = False
        self.data.append(data.spectrum)

    def writeCSV(self):
        with open('spectrum.csv', mode = 'w+b') as csvout:
            writer = csv.writer(csvout)
            # transform rows into cols
            self.data = np.transpose(np.array(self.data))
            for row in self.data:
                writer.writerow(row)

    def initAnimation(self): # line length
        root = tk.Tk()
        # Buttons for capturing spectra
        dark = tk.Button(root, command = lambda : self.capture(mode = 'dark'))
        light = tk.Button(root, command = lambda : self.capture(mode = 'ref'))
        spec = tk.Button(root, command = lambda : self.capture(mode = 'sample'))
        # Buttons for displaying current spectra
        raw = tk.Button(root, command = lambda : self.display(mode = 'raw'))
        reflect = tk.Button(root, command = lambda : self.display(mode = 'reflect'))
        # Pack all
        darkButton.pack()
        lightButton.pack()
        specButton.pack()

    def capture(self, mode = 'sample'):
        # Display capture?
        self.getData()
        if mode.lower() in ['background', 'bg']:
            self.bg = self.spectrum
        elif mode.lower() in ['ref', 'referece']:
            self.ref = self.spectrum
        elif mode.lower() == 'sample':
        # Don't allow sample if no reference spectra
            if bg == None:
                self.noBackgroundWarn()
                warn = True
            if ref == None:
                self.noReferenceWarn()
                warn = True
            if warn == True:
                return
            # R = (sample - background)/(reference - background)
            specturm, bg, ref = self.spectrum, self.bg, self.ref
            self.R = (spectrum - bg)/(ref - bg)

    def noBackgroundWarn(self):
        print "Must take background spectrum before calculating reflectance"

    def noReferenceWarn(self):
        print "Must take reference spectrum before calculating reflectance"

    def display(self, data, mode = 'raw'):
        self.getData()
        plt.figure()
        wavelengths = data.wavelengths
        spectrum = data.spectrum
        plt.plot(wavelengths, spectrum)
        plt.show(block = False)

    def bagRecord(self):
        self.bag = bag = rosbag.Bag('spectra.bag')
        for item in bag.read_messages(topics = ['/spectrometer/spectrum']):
            self.item = item
            break
        bag.close()

r = Reflectance()