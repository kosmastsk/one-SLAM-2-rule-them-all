#!/usr/bin/env python

import roslib
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import numpy as np
import python_utils
import cv2
import webcolors

def centroid_histogram(clt):
	# grab the number of different clusters and create a histogram
	# based on the number of pixels assigned to each cluster
	numLabels = np.arange(0, len(np.unique(clt.labels_)) + 1)
	(hist, _) = np.histogram(clt.labels_, bins = numLabels)
 
	# normalize the histogram, such that it sums to one
	hist = hist.astype("float")
	hist /= hist.sum()
 
	# return the histogram
	return hist

def plot_colors(hist, centroids):
	# initialize the bar chart representing the relative frequency
	# of each of the colors
	#bar = np.zeros((50, 300, 3), dtype = "uint8")
	startX = 0
	div_sum = 0
 
	# loop over the percentage of each cluster and the color of
	# each cluster
	for (percent, color) in zip(hist, centroids):
		# plot the relative percentage of each cluster
		endX = startX + (percent * 300)
		#cv2.rectangle(bar, (int(startX), 0), (int(endX), 50),
		#	color.astype("uint8").tolist(), -1)
		#----------------------------------------------------
		magnitude = endX - startX
		actual_name, closest_name = get_colour_name(color)
		
		per = round(magnitude*1/3,2) 	#fix percentage from x/300 to x/100
										#and round it to 2 decimal digits
 		
		print closest_name, " -->", per , "%"

		divergence = abs(per - 10)	#percentage divergence from 10% (which is 
								#equally distributed colors for 10 clusters)
		div_sum += divergence




		startX = endX
		#----------------------------------------------------
	
	print "Equal color distribution divergence: " 	#antistrofi metriki
	return div_sum / 10
def closest_colour(requested_colour):
    min_colours = {}
    for key, name in webcolors.css3_hex_to_names.items():
        r_c, g_c, b_c = webcolors.hex_to_rgb(key)
        rd = (r_c - requested_colour[0]) ** 2
        gd = (g_c - requested_colour[1]) ** 2
        bd = (b_c - requested_colour[2]) ** 2
        min_colours[(rd + gd + bd)] = name
    return min_colours[min(min_colours.keys())]

def get_colour_name(requested_colour):
    try:
        closest_name = actual_name = webcolors.rgb_to_name(requested_colour)
    except ValueError:
        closest_name = closest_colour(requested_colour)
        actual_name = None
    return actual_name, closest_name

def sub_cluster():
	
	
	rospy.Subscriber("/camera/rgb/image_raw", Image, cb)

def cb(data):

	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	
	#cv2.imshow("Image Window", cv_image) 	#won't run because of complex calculations below

	cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)	#necessary?
	cv_image = cv_image.reshape((cv_image.shape[0] * cv_image.shape[1], 3)) 

	clt = KMeans(n_clusters = 10)	#define number of clusters 
	clt.fit(cv_image)

	hist = centroid_histogram(clt) 
	print plot_colors(hist, clt.cluster_centers_)

	'''
	plt.figure()
	plt.axis("off")
	plt.imshow(bar)
	plt.show()
	'''
	#cv2.waitKey(1)	#wont' run

	
	print "----------------------------"



if __name__=='__main__':
	rospy.init_node('clusterer', anonymous = True)
	try:
		sub_cluster()
		rospy.spin()

	except rospy.ROSInterruptException:
		pass