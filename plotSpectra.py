#!/usr/bin/env python
# Subscribes to /spectrometer/spectrum topic and plots every 100th spectrum from the Ocean Optics Spectrum messages 
import rospy
from ocean_optics.msg import Spectrum
import matplotlib.pyplot as plt
plt.ion()
fig = plt.figure()
l,  = plt.plot([], [])

def callback(data):
	wavelengths = data.wavelengths	
	spectrum = data.spectrum
	plt.pause(.08)
	l.set_data(wavelengths, spectrum)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/spectrometer/spectrum", Spectrum, callback)
    rospy.spin()
        
if __name__ == '__main__':
    listener()
