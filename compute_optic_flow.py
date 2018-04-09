import datetime
import io
import os
import cv2
import numpy as np
import threading
import time
import random
from operator import add
import scipy.io
import csv
import sys

full = False
seq_time = 69           	# nominal video time in sec
frame_rate = 60

fast = cv2.FastFeatureDetector_create(10)	
feature_N = 100
lk_params = dict( winSize  = (10, 10),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
feature_params = dict( maxCorners = feature_N,	# 100
                       qualityLevel = 0.5,
                       minDistance = 7,
                       blockSize = 7 )


class ImageReceiver(threading.Thread):
	def __init__(self):
		super(ImageReceiver, self).__init__()
		self.event = threading.Event()
		self.terminated = False
		self.img_time = 0.


		self.start()

	def run(self):
		# This method runs in a separate thread
		global done
		global frame
		lkflow = LKFlow()
		directflow = DirectFlow()
		while not self.terminated:
			if True:
				try:
					ctime = self.img_time
                    # Read the image and do some processing on it
					if 'image' in locals():
						prev_img = image
						image = cv2.imread('images60fps' + '/test' + str(frame) + '.png', 0)

						time_diff = ctime-ltime
						ltime = ctime

					else:
						# first time running
						prev_img = cv2.imread('images60fps' + '/test' + str(frame) + '.png', 0)
						image = prev_img.copy()
						ltime = ctime-1
						time_diff = 1

					# now we have both prev_img and image in greyscale for processing
					print('processing frame:' + str(frame) + "\n")

					lkflow.img1 = prev_img
					lkflow.img2 = image
					lkflow.frameid = frame
					lkflow.flow_calculate()


					directflow.nimg = image
					directflow.frameid = frame
					directflow.flow_calculate ()

					frame = frame+1
					if frame == seq_time*frame_rate:
						self.terminated = True
				finally:
					# Reset the stream and event
					self.event.clear()
		lkflow.savemat()
		directflow.savemat()
		self.stopped = True




class LKFlow ():
	def __init__(self):
		self.img1 = []
		self.img2 = []
		self.track_len = 2         # default 10
		self.detect_interval = 1    # default 5
		self.points_no = 100			# maximum number of points we keep tracking
		self.tracks = []
		self.terminated = False
		self.rx = 0.
		self.ry = 0.
		self.rz = 0.
		self.rxyz = np.zeros((seq_time*frame_rate,4))
		self.frameid = 0
		self.dt = 0. 	# for output
	def flow_calculate(self):
    	# based on the OpenCV example at https://github.com/opencv/opencv/blob/master/samples/python/lk_track.py
		global frame

		if frame % self.detect_interval == 0:
			#print "detecting features"

			kp = fast.detect(cv2.resize(self.img2,None,fx=1, fy=1, interpolation = cv2.INTER_CUBIC),None)
			#print len(kp)
			if kp is not None:
				self.tracks = []
				for point in kp:
					self.tracks.append([(point.pt[0], point.pt[1])])

		if len(self.tracks) > 4:
			#print len(self.tracks)

			# select the maximum of feature_N points
			if len(self.tracks) < feature_N:
				Np = len(self.tracks)
			else:
				Np = feature_N
			# select Np points randomly
			tracks = random.sample(self.tracks, Np)

			p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 1, 2)
			p1, st, err = cv2.calcOpticalFlowPyrLK(self.img1, self.img2, p0, None, **lk_params)
			p0r, st, err = cv2.calcOpticalFlowPyrLK(self.img2, self.img1, p1, None, **lk_params)
			d = abs(p0-p0r).reshape(-1, 2).max(-1)
			good = d < 1
			self.tracks = np.concatenate((p0.reshape(-1, 1, 2)[good,:],p1.reshape(-1, 1, 2)[good,:]),axis=1).tolist()[-self.points_no:]

			ftracks = self.tracks[:]


			points = np.array(ftracks).reshape([-1,4])
			xpos = np.mean(points[:,[0,2]], axis=1)
			ypos = np.mean(points[:,[1,3]], axis=1)
			# apply correction to pixel indices (to be centred in the middle of the image)
			xpos = xpos -0.5*img_w
			ypos = ypos - 0.5*img_h
			xflow = points[:,2]-points[:,0]
			yflow = points[:,3]-points[:,1]						

			N = len(xpos)
			if N > 10:
				p1, p2, p3, p4 = [0.,0.,0.,0.]
				if full:
					G1 = np.multiply(xpos, xpos)
					G2 = np.multiply(ypos, ypos)
					G3 = np.multiply(xpos, ypos)


					A = np.vstack((np.vstack((-np.ones(N),np.zeros(N),xpos,G1,G3,-G2,np.zeros(N))).T, np.vstack((np.zeros(N),-np.ones(N),ypos,G3,G2,np.zeros(N),-G1)).T))
					lsq_sol = np.linalg.lstsq(A,  np.concatenate((xflow,yflow), axis=0) )
					rx, ry, rz, p1, p2, p3, p4 = lsq_sol[0]
				else:
					A = np.vstack((np.vstack((-np.ones(N),np.zeros(N),xpos)).T, np.vstack((np.zeros(N),-np.ones(N),ypos)).T))
					lsq_sol = np.linalg.lstsq(A,  np.concatenate((xflow,yflow), axis=0) )
					rx, ry, rz = lsq_sol[0]

				self.rxyz[self.frameid,] = (self.frameid, rx, ry, rz)
				residual = lsq_sol[1]
				ATA = np.dot(np.transpose(A),A)
				self.rxyz[self.frameid,] = (self.frameid, rx, ry, rz)


		else:
			#print('no track' + "\n")
			ftracks = self.tracks[:]

		N = len(ftracks)

	def savemat(self):
		scipy.io.savemat('lkflow.mat', mdict={'opticflow': self.rxyz})

class DirectFlow():

	def __init__(self):
		self.p_sobel_x = np.zeros((img_h,img_w))
		self.p_sobel_y = np.zeros((img_h,img_w))
		self.p_img_f = np.zeros((img_h,img_w))
		self.nimg = []
		self.frameid = 0
		self.rxyz = np.zeros((seq_time*frame_rate,8))
		self.tracks = []
		self.terminated = False


	def flow_calculate(self):

		# image motion
		n_img_f = cv2.blur(self.nimg,(5,5))
		n_sobelx = cv2.Sobel(n_img_f,cv2.CV_32F,1,0,ksize=3)/8.	# new image sobel
		n_sobely = cv2.Sobel(n_img_f,cv2.CV_32F,0,1,ksize=3)/8.
		a_sobelx = 0.5*(n_sobelx+self.p_sobel_x)				# average
		a_sobely = 0.5*(n_sobely+self.p_sobel_y)				# average
		img_grad = np.int16(n_img_f) - np.int16(self.p_img_f)
		## find the visual observables
		step = 4
		Y, X = np.mgrid[step/2:img_h:step, step/2:img_w:step].reshape(2,-1).astype(int)

		Ex = a_sobelx[Y,X]
		Ey = a_sobely[Y,X]
		Et = img_grad[Y,X]
		X = X-img_w*0.5
		Y = Y-img_h*0.5
		np.min(X)
		p1, p2, p3, p4 = [0.,0.,0.,0.]
		B = -Et
		G1 =  np.multiply(X,Ex)+np.multiply(Y,Ey)
		if full:
			G2 = np.multiply(Y, Ex)
			G3 = np.multiply(X, Ey)
			G4 = np.multiply(G1, X)
			G5 = np.multiply(G1, Y)
			A = np.vstack((Ex, Ey, G1, G2, G3, G4, G5)).T
			lsq_sol = np.linalg.lstsq(A,  B )
			rx, ry, rz, p1, p2, p3, p4= lsq_sol[0]
		else:
			A = np.vstack((Ex, Ey, G1)).T
			lsq_sol = np.linalg.lstsq(A,  B )
			rx, ry, rz = lsq_sol[0]
		
		self.rxyz[self.frameid,] = (self.frameid, rx, ry, rz, p1, p2, p3, p4)

		# keep buffer of the previous img
		self.p_sobel_x = n_sobelx[:]
		self.p_sobel_y = n_sobely[:]
		self.p_img_f = n_img_f[:]

	def savemat(self):
		scipy.io.savemat('directflow.mat', mdict={'directflow': self.rxyz})
	
	
## main routine


frame = 0
done = False

# optic flow parameters
img_h = 240;				# in pixels
img_w = 320;


processor = ImageReceiver()
processor.join()

