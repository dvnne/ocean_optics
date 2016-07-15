#!usr/bin/env python
from __future__ import division
import rospy
from ocean_optics.msg import Spectrum
import matplotlib.pylab as plt
import Tkinter as tk
import numpy as np

class Reflectance(object):
    def __init__(self):
        self.initAnimation()
        self.bg = None
        self.ref = None
        self.spectrum = None

    def subscribe(self):
        rospy.init_node('subscriber')
        # set self.callback to specific fn that needs the spectrum
        self.subscriber = rospy.Subscriber("/spectrometer/spectrum", Spectrum,
                                            self.callback)
        rospy.spin() # test if this code can be run without rospy.spin

    def initAnimation(self):
        root = tk.Tk()
        # Define buttons
        darkButton = tk.Button(root, command = lambda : self.capture(mode = 'dark'))
        lightButton = tk.Button(root, command = lambda : self.capture(mode = 'ref'))
        specButton = tk.Button(root, command = lambda : self.capture(mode = 'sample')
        # Pack all
        darkButton.pack()
        lightButton.pack()
        specButton.pack()

    def capture(self, mode = 'sample'):
        spectrum = self.spectrum # current sample
        bg = self.bg # background
        ref = self.ref # reference
        wavelengths = self.wavelengths
        # Don't allow sample if no reference spectra
        if mode.lower() == 'sample':
            if bg == None:
                self.noBackgroundWarn()
                warn = True
            if ref == None:
                self.noReferenceWarn()
                warn = True
            if warn == True:
                return
            # R = (sample - background)/(reference - background)
            self.R = (spectrum - bg)/(ref - bg)
            return self.R
        return

    def noBackgroundWarn(self):
        print "Must take background spectrum before calculating reflectance"

    def noReferenceWarn(self):
        print "Must take reference spectrum before calculating reflectance"

    def callback_dec(self, fn):
        def wrapper():
            fn = self.callback
            self.subscribe()

