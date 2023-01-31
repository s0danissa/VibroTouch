#!/usr/bin/env python2.7

# ---------------------------------------------------------------
# Importing all of the required libraries
import os
import rospy
import cv2
import math
import scipy
from scipy.fftpack import fft, fftfreq
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from matplotlib import pyplot as plt
import csv
import datetime
import time

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

# ------------------------------------------------------------------------------------------------------------------------------------
# 							GLOBAL VARIABLES
# ------------------------------------------------------------------------------------------------------------------------------------

global exp4CounterCSV
exp4CounterCSV = 0
global fftRecordingFlag
fftRecordingFlag = 0

# ---------------------------------------------------------------
# Classification parameters
global expProcedureState
expProcedureState = 0
global exp3CounterCSV
exp3CounterCSV = 0
global expType
expType = 1
global forceControlCallbackFlag
forceControlCallbackFlag = 0

# ---------------------------------------------------------------
# Touch detection
global gripperPosition
gripperPosition = 0.0

# ---------------------------------------------------------------
# Global force parameters
global forceTrackbarPos
forceTrackbarPos = 0
global forceControlPub
global wittForceY
wittForceY = 0.0
global wittForceZ
wittForceZ = 0.0

# ---------------------------------------------------------------
# Spectrogram global parameters
global bufferCounter
bufferCounter = 0
global accelerometerData
accDataSize = 2500
accelerometerData = np.zeros(accDataSize)
global bufferUpdateCounter
bufferUpdateCounter = 0
global bufferUpdateThr
bufferUpdateThr = 2
global initializerCounter
initializerCounter = 0
global localMaxArray
localMaxArray = []
global spectrIntensity
spectrIntensity = 0
global spectrSave
spectrSave = 0
global fftSave
fftSave = 0


# ------------------------------------------------------------------------------------------------------------------------------------
# 							COMPUTATIONAL FUNCTIONS - GUI SPECIFIC
# ------------------------------------------------------------------------------------------------------------------------------------
# Estimating the number of peaks
def peaks_n(arr_n):
	peak_counter = 0
	N = len(arr_n)
	new_arr_n = arr_n[200:400]
	return new_arr_n

# Outputs five most prominent frequency bands
def max_hz(a):
	global localMaxArray
	localMaxArray = []
	localMaxArray.append(np.where(a==max(a[20:250]))[0][0])
	localMaxArray.append(np.where(a==max(a[250:500]))[0][0])
	localMaxArray.append(np.where(a==max(a[500:750]))[0][0])
	localMaxArray.append(np.where(a==max(a[750:1000]))[0][0])
	localMaxArray.append(np.where(a==max(a[1000:1250]))[0][0])
	out_ar = np.array(localMaxArray)
	localMaxArray = []
	return out_ar

# Takes in the signal and returns the frequency with a max frequency
def fft_short(sig):
	N = len(sig)
	T = 1.0 / (2500*0.8975)
	x = np.linspace(0.0, N*T, N, endpoint=False)
	y = sig
	y = y - np.mean(y)
	yf = fft(y)
	xf = fftfreq(N, T)[:N//2]
	yff = 2.0/N * np.abs(yf[0:N//2])
	yf_im = yf[0:N//2].imag
	yf_real = yf[0:N//2].real

	max_im = max(yf_im)
	max_real = max(yf_real)

	#print(len(yf_real))
	ind = np.argmax(yff[50:150])
 	return xf[0:len(xf)], yff[0:len(yff)], ind, np.abs(yf_im)/max(np.abs(yf_im))*200, np.abs(yf_real)/max(np.abs(yf_real))*200, max_im ,max_real


# ------------------------------------------------------------------------------------------------------------------------------------
# 							GRAPHIC FUNCTIONS - GUI SPECIFIC
# ------------------------------------------------------------------------------------------------------------------------------------
# Function responsible for the conversion from BW to RGB
def to_rgb(ff):
	global spectrIntensity
	final_arr = np.zeros((len(ff),3))
	ff_max = max(ff)
	r_ff = 0
	g_ff = 0
	b_ff = 0
	for t in range(0,len(ff)):
		rgb_ff = (ff[t]/spectrIntensity)*255
		rgb_arr = np.zeros(3)
		rgb_arr[0] = 0
		rgb_arr[1] = 0
		rgb_arr[2] = rgb_ff
		final_arr[t] = rgb_arr
	return final_arr

# Draws a grid on a given image
def grid_draw(img,xfx,yfy):
	y = img.shape[0]
	x = img.shape[1]

	stepx = int(x/6)
	stepy = int(y/5)

	x_a = np.arange(start = 0, stop = x, step = stepx)
	y_a = np.arange(start = 0, stop = y, step = stepy)

	font                   = cv2.FONT_HERSHEY_PLAIN
	fontScale              = 1
	fontColor              = (0.5,0.5,0.5)
	thickness              = 1

	for i in x_a:
		img = cv2.line(img,(i+15,0), (i+15, y), (0.2,0.2,0.2),1)
	for j in y_a:
		img = cv2.line(img,(0,j+15), (x,j+15), (0.2,0.2,0.2),1)
	return img

# Puts the axis tick labels in the places according to the grid drawn previously
def ax_draw(img,xfx,yfy):
	y = img.shape[0]
	x = img.shape[1]

	# pixel step for x axis
	stepx = int(x/6)
	# pixel step for y axis
	stepy = int(y/5)

	# x axis lines
	x_a = np.arange(start = 0, stop = x, step = stepx)
	# y axis lines
	y_a = np.arange(start = 0, stop = y, step = stepy)

	xl_max = max(xfx)
	xl_step = xl_max/6
	# x axis ticks
	x_atick = np.arange(start = 0, stop = xl_max, step = 200)

	yl_max = 500
	yl_step = yl_max/5
	# y axis ticks
	y_atick = np.arange(start = 0, stop = yl_max, step = 100)

	font                   = cv2.FONT_HERSHEY_PLAIN
	fontScale              = 0.75
	fontColor              = (0.7,0.7,0.7)
	thickness              = 1

	for i in range(0,5):
		img = cv2.putText(img,str(int(y_atick[i])), (5, y-int(y_a[i])-15), font, fontScale, fontColor, thickness, cv2.LINE_AA)
	for i in range(0,6):
		img = cv2.putText(img,str(int(x_atick[i])), (int(x_a[i])+15, y-5), font, fontScale, fontColor, thickness, cv2.LINE_AA)
	return img



# ------------------------------------------------------------------------------------------------------------------------------------
# 							BUTTON CALLBACKS
# ------------------------------------------------------------------------------------------------------------------------------------
# Callback function for "Apply force" button
def send_callback(*args):
	global forceTrackbarPos, forceControlPub
	forceControlPub.publish(forceTrackbarPos/50.0)

# Callback function for "Release" button
def release_call(*args):
	global forceControlPub
	rospy.loginfo("Releasing..")
	forceControlPub.publish(-10.0)

# Callback function for PROCEDURE 3
def exp_call(*args):
	global expProcedureState, expType
	expType = 1
	expProcedureState = 1
	rospy.loginfo("STARTING EXPERIMENTAL PROCEDURE FOR CLASSIFICATION...")

# Callback function for PROCEDURE 4
def config_call(*args):
	global expType, expProcedureState
	expType = 2
	expProcedureState = 1
	rospy.loginfo("STARTING CONFIGURATION PROCEDURE...")

# Callback function saving the spectrogram in full resolution
def save_call(*args):
	global spectrSave
	spectrSave = 1
	rospy.loginfo("SAVING CURRENT SPECTROGRAM...")

# Callback function saving the spectrogram in full resolution
def fft_save_call(*args):
	global fftSave
	if fftSave == 0:
		fftSave = 1
	else:
		fftSave = 0



# ------------------------------------------------------------------------------------------------------------------------------------
# 							TRACKBAR CALLBACKS
# ------------------------------------------------------------------------------------------------------------------------------------
# Callback function for the applied force trackbar
def trackb_callback(val):
	pass

# Callback function for the spectrogram intensity trackbar
def intens_call(val):
	pass



# ------------------------------------------------------------------------------------------------------------------------------------
# 							SUBSCRIBER CALLBACKS
# ------------------------------------------------------------------------------------------------------------------------------------
# Callback function for the current force display from Wittenstein
def see_callback(data_witt):
	global wittForceZ, wittForceY
	wittForceY = data_witt.data[1]
	wittForceZ = data_witt.data[2]

# Callback function for the exp_status topic
def exp_callback(dataa):
	global forceControlCallbackFlag, exp3CounterCSV
	if dataa.data[0] == 2:
		forceControlCallbackFlag = 2	
		rospy.loginfo("Starting acquisition, EXP:"+str(exp3CounterCSV))
	if dataa.data[0] == 1:
		forceControlCallbackFlag = 1
		rospy.loginfo("Recieved ending signal, ending acquisition")
		exp3CounterCSV = exp3CounterCSV+1

# Callback function for obtaining gripper data
def grip_callback(d_grip):
	global gripperPosition
	gripperPosition = d_grip.data[0]

# Callback function for the "chatter" topic - accelerometer data k_here
def callback(data):
	global bufferCounter
	global accelerometerData
	if bufferCounter <= accDataSize-5:
		accelerometerData[bufferCounter] = data.data[0]
		accelerometerData[bufferCounter+1] = data.data[1]
		accelerometerData[bufferCounter+2] = data.data[2]
		accelerometerData[bufferCounter+3] = data.data[3]
		accelerometerData[bufferCounter+4] = data.data[4]
		bufferCounter = bufferCounter+1
	else:
		accelerometerData = np.roll(accelerometerData, 5)
		accelerometerData[accDataSize-5] = data.data[0]
		accelerometerData[accDataSize-4] = data.data[1]
		accelerometerData[accDataSize-3] = data.data[2]
		accelerometerData[accDataSize-2] = data.data[3]
		accelerometerData[accDataSize-1] = data.data[4]



# ------------------------------------------------------------------------------------------------------------------------------------
# 								MAIN
# ------------------------------------------------------------------------------------------------------------------------------------

def fft_main():
	val_a = 0
	old_val_a = 0

	old_val_norm_arr = 0
	val_norm_arr = 0

	global wittForceZ, wittForceY, expType, forceControlCallbackFlag, gripperPosition, fftSave, exp4CounterCSV
	global spectrIntensity, fftRecordingFlag
	# Array creation for sliding window
	slid_arr = np.zeros((150))
	local_max = 0.0
	N_of_exp = 100
	onetime_c = 0

	amp_max = 0
	# Initialization of global parameters for impusle fft
	
	rospy.init_node('imp_fft', anonymous = True) # added

	rospy.Subscriber("chatter", Int16MultiArray, callback) # added
	rospy.Subscriber("forceFloat32", Float32MultiArray, see_callback) # added
	rospy.Subscriber("exp_status", Int16MultiArray, exp_callback) # added
	rospy.Subscriber("float_grip", Float32MultiArray, grip_callback) # added

	forceControlPub = rospy.Publisher("send_force", Float32, queue_size = 10) # added

	rate = rospy.Rate(500)

	imp_data = np.array([])
	ros_work = False

	font1                   = cv2.FONT_HERSHEY_PLAIN
	fontScale1              = 1
	fontColor1              = (0,0.7,0)
	thickness1              = 1

	cv2.namedWindow("FFT")
	X_s = 200
	Y_s = 600
	image = np.zeros((X_s+30,Y_s+30,3))

	# ---------------------------------------------------------------
	# GUI PARAMETERS
	# ---------------------------------------------------------------

	# Trackbar parameters
	trackb = 'Force'
	global forceTrackbarPos, forceControlPub
	cv2.createTrackbar(trackb, "FFT", 0, 500, trackb_callback)

	# Intensity trackbar
	tracka = 'Spectrogram intensity'
	cv2.createTrackbar(tracka, "FFT", 1, 20, intens_call)
	
	trackb_callback(0)
	intens_call(0)

	# Buttons
	button_t = 'APPLY FORCE'
	cv2.createButton(button_t, send_callback, "FFT", cv2.QT_PUSH_BUTTON, 1)
	button_r = 'RELEASE'
	cv2.createButton(button_r, release_call, "FFT", cv2.QT_PUSH_BUTTON, 1)
	button_exp = 'CLASSIFY'
	cv2.createButton(button_exp, exp_call, "FFT", cv2.QT_PUSH_BUTTON, 1)
	button_config = "CONFIGRATION TESTING"
	cv2.createButton(button_config, config_call, "FFT", cv2.QT_PUSH_BUTTON, 1)	
	button_save = "SAVE THE SPECTROGRAM"
	cv2.createButton(button_save, save_call, "FFT", cv2.QT_PUSH_BUTTON, 1)
	fft_save = "SAVE CURRENT SPECTRUM"
	cv2.createButton(fft_save, fft_save_call, "FFT", cv2.QT_PUSH_BUTTON, 1)
	global spectrSave
	fft_save_sount = 0
	
	# Initialization of global parameters for the spectrogram 
	global accelerometerData, bufferUpdateThr, initializerCounter
	bufferUpdateCounter = 0
	hz_max_c = 0
	initializerThr = 5
	m = (1250,1250,3)
	rotated = np.zeros(m)
	m = np.zeros(m)
	spectr_image = np.zeros((630,630,3))
	
	# Horizontal padding for he supplementary plots
	sup_x = np.zeros((5, 600, 3)) + (0.3,0.3,0.3)

	# Intermediate padding declaration
	padx = np.zeros((5, 630, 3)) + (0.3,0.3,0.3)
	pady = np.zeros((905, 5, 3)) + (0.3,0.3,0.3)	

	# Force data padding declaration
	force_pad = np.zeros((30, 630, 3)) + (1.0,1.0,1.0)
	fft_save_c = 0
	spect = True
	imp_counter = 0

	# Touch detection
	archive = np.zeros((40))
	c_touch_start = 0
	freq_archive = np.zeros((40))
	curr_f_val = 0.0
	sum_interval = 0.0
	mean_archive = np.zeros((10))
	mean_archive_2 = np.zeros((80))

	# Amplitude plot
	amp_arr = np.zeros((600))
	abs_max_amp = 0
	amp_x = np.linspace(0, 600, 600, endpoint=False)
	amp_count = 0
	amp_norm = 0
	file_open = 0

	# Frequency plot
	freq_arr = np.zeros((600))
	abs_max_freq = 10.0
	freq_x = np.linspace(0,600,600, endpoint=False)
	freq_norm = 0
	
	# Experimental procedure parameters
	global expProcedureState
	force_thr = 0
	squeeze_count = 0
	global exp3CounterCSV
	spike_count = 0
	rospy.loginfo("Initializing...")

	while not rospy.is_shutdown():


		touch_spike = 0
		c_touch_start = c_touch_start + 1
		forceTrackbarPos = cv2.getTrackbarPos(trackb, "FFT")
		spectrIntensity = cv2.getTrackbarPos(tracka, "FFT")*50
		# ---------------------------------------------------------------
		# Spectrogram update + 
		# ---------------------------------------------------------------
		if spect == True:
			bufferUpdateCounter =  bufferUpdateCounter+1
			# Real-time FFT ------------------------------------
			N = 2500
			maxN = N/2-1
			T = 1.0/2500
		
			font                   = cv2.FONT_HERSHEY_PLAIN
			fontScale              = 1.5
			fontColor              = (255,255,255)
			thickness              = 1

			if bufferUpdateCounter > bufferUpdateThr:
				initializerCounter = initializerCounter + 1
				if initializerCounter == initializerThr:
					hz_max_c = hz_max_c + 1
					initializerCounter = 0
					yf = fft(accelerometerData)
					xf = fftfreq(N, T)[:N//2]
					yff = 2.0/N * np.abs(yf[0:N//2])
					yf_im = yf[0:N//2].imag
					yf_real = yf[0:N//2].real
					
					save_yff = yff

					amp_max = max(yff)
					yff_old = (yff/spectrIntensity)*255	
					sum_interval = np.argmax(save_yff[180:250])+100
					yff = to_rgb(yff)
					yff[180] = yff[180]+(0,0,255)
					yff[250] = yff[250]+(0,0,255)

					if hz_max_c > 10:				
						out_maxx = max_hz(yff_old)
			
					m[0:maxN-bufferUpdateThr] = m[bufferUpdateThr:maxN]
					m[maxN-bufferUpdateThr:maxN] = yff
					(cX, cY) = (1250 // 2, 1250 // 2)
					rot = cv2.getRotationMatrix2D((cX, cY), 90, 1.0)
					rotated = cv2.warpAffine(m, rot, (1250, 1250))
					bufferUpdateCounter = 0
					if spectrSave == 1:
						current_time = datetime.datetime.now()
						saved_spec = "spectra_" +str(current_time.month)+"_"+str(current_time.day)+"_"+str(current_time.hour)+"_"+str(current_time.minute)+"_"+str(current_time.second)+".csv"
						spectra_file = open(saved_spec, mode = 'w')
						spectra_writer = csv.writer(spectra_file)
						try:				
							spectra_writer.writerow(yff_old)
							spectra_writer.writerow(xf)
						except:
							rospy.loginfo("SAVING FAILED")
						else:
							rospy.loginfo("SPECTROGRAM SAVED SUCCESSFULLY| FILENAME: " + saved_spec)
						spectrSave = 0

			if fftSave == 1:
				fft_save_c = fft_save_c + 1
				if fftRecordingFlag == 0:
					rospy.loginfo("RECORDING INTIATED")
					fft_real_file = open('fft_data_real'+str(fft_save_c)+'.csv', mode = 'w')						

					fft_real_writer = csv.writer(fft_real_file)
					fftRecordingFlag = 1
				if fftRecordingFlag == 1:
					fft_real_writer.writerow([save_yff[sum_interval], xf[sum_interval], abs(wittForceY)])
			if fftSave == 0 and fftRecordingFlag == 1:
				fft_real_file.close()
				fftRecordingFlag = 0
				fftSave = 0
				rospy.loginfo("RECORDING STOPPED")		

			if hz_max_c > 10:
				for i in range(0,len(out_maxx)):
					rotated = cv2.putText(rotated, str(out_maxx[i])+' Hz', (100,1250-out_maxx[i]), font, fontScale, fontColor, thickness, cv2.LINE_AA)
			spectr_image = cv2.resize(rotated, (630,630))
			if fftRecordingFlag == 1:
				spectr_image = cv2.circle(spectr_image, (30,600), 20, (0,0,255), -1)
		
		# ---------------------------------------------------------------
		# FFT plot - REGULAR (NO IMPULSE) MODE
		# ---------------------------------------------------------------
			if ros_work == False:
				ros_work = True
				rospy.loginfo("Initialization successful!")
			image = np.zeros((X_s+30,Y_s+30,3))
				
			# Initialization of the supplementary plots
			amp_m = np.zeros((220, 600, 3))
			im_m = np.zeros((220, 600, 3))
			real_m = np.zeros((220, 200, 3))
			freq_m = np.zeros((220, 600, 3))

			fft_x, fft_y, in_max, fft_imag, fft_real, ffm_im, ffm_real  = fft_short(accelerometerData)
			fft_origx, fft_origy = fft_x, fft_y
			f_max = fft_x[50:150][in_max]

			# Sliding window
			N = len(slid_arr)
			ind_max_a = np.argmax(slid_arr)
			max_a = slid_arr[ind_max_a]
			if ind_max_a > int(0.30*N) and ind_max_a < int(0.70*N):
				local_max = max_a

			slid_arr = np.roll(slid_arr,1)
			slid_arr[149] = max(fft_origy[50:150])

				

			# Updating the amplitude array
			amp_count = amp_count + 1
			if amp_count == 1:
				rospy.loginfo("Calibration started, please wait...")

			if amp_count > 250:
				if amp_count == 251:
					rospy.loginfo("Callibration finished")
					amp_norm = local_max
					freq_norm = f_max
				if amp_count == 252:
					old_val_norm_arr = val_norm_arr
					val_norm_arr = amp_norm/abs_max_amp*100
					if np.isnan(val_norm_arr) or np.isinf(val_norm_arr):
						val_norm_arr = old_val_norm_arr
					else:
						val_norm_arr = 	int(val_norm_arr)					
					norm_arr = np.zeros((600)) + val_norm_arr
					norm_freq_arr = np.zeros((600)) + 100
				if int(max(fft_origy)) > abs_max_amp:
					abs_max_amp = int(max(fft_origy[50:150]))
				amp_arr = np.roll(amp_arr, 1)
				freq_arr = np.roll(freq_arr, 1)
				olv_val_a = val_a
				val_a = (local_max-amp_norm)/abs_max_amp*100
				if np.isnan(val_a) or np.isinf(val_a):
					val_a = old_val_a
				else:
					val_a = int(val_a)
				val_f = int((f_max-freq_norm)/abs_max_freq*100)

				if val_a < 0:
					amp_arr[599] = 100-(-val_a)
				else:
					amp_arr[599] = 100 + val_a	
				if val_f < 0:
					freq_arr[599] = 100-(-val_f)
					curr_f_val = 100-(-val_f)
				else:
					freq_arr[599] = 100 + val_f
					curr_f_val = 100 + val_f
				
			fft_x = fft_x/max(fft_x)*Y_s + 15
			fft_y = fft_y/max(fft_y)*X_s + 15
				
# UPDATING ARCHIVED VALUES ARRAY ---------------------------------------------------------------
			old_mean_arc = np.mean(archive)
			archive = np.roll(archive, 1)
			archive[39] = max(fft_origy[50:150])
			freq_archive = np.roll(freq_archive, 1)
			freq_archive[39] = curr_f_val
			mean_archive = np.roll(mean_archive, 1)
			mean_archive[9] = sum_interval
			mean_archive_2 = np.roll(mean_archive_2, 1)
			mean_archive_2[79] = sum_interval

			image = grid_draw(image, fft_origx, fft_origy)
			curve = np.column_stack((fft_x.astype(np.int32), fft_y.astype(np.int32)))
	
			image = cv2.polylines(image, [curve], False, (0,255,0))
			image = cv2.flip(image, 0)
			image = ax_draw(image, fft_origx, fft_origy)

			curve_imag = np.column_stack((fft_x.astype(np.int32), fft_imag.astype(np.int32)))
			zoom_fft = peaks_n(fft_origy)
			zoom_x = fft_origx[200:400]-fft_origx[200]
			curve_real = np.column_stack((zoom_x.astype(np.int32), zoom_fft.astype(np.int32)))
			curve_freq = np.column_stack((freq_x.astype(np.int32), freq_arr.astype(np.int32)))
			curve_amp = np.column_stack((amp_x.astype(np.int32), amp_arr.astype(np.int32)))

			if amp_count > 252:				
				curve_amp_norm = np.column_stack((amp_x.astype(np.int32), norm_arr.astype(np.int32)))
				curve_freq_norm = np.column_stack((freq_x.astype(np.int32), norm_freq_arr.astype(np.int32)))
				amp_m = cv2.polylines(amp_m, [curve_amp_norm], False, (0.3,0.3,0.3))
				freq_m = cv2.polylines(freq_m, [curve_freq_norm], False, (0.3,0.3,0.3))

			im_m = cv2.polylines(im_m, [curve_imag], False, (0,1.0,0.5))
			im_m = cv2.flip(im_m, 0)
			real_m = cv2.polylines(real_m, [curve_real], False, (0,0.5,1.0))
			real_m = cv2.flip(real_m, 0)
			real_m = cv2.resize(real_m, None, fx = 3, fy = 1)
			freq_m = cv2.polylines(freq_m, [curve_freq], False, (1.0,0,0.5))
			freq_m = cv2.flip(freq_m, 0)
			amp_m = cv2.polylines(amp_m, [curve_amp], False, (1.0,0.5, 0))
			amp_m = cv2.flip(amp_m, 0)

			image = cv2.putText(image,"Max: "+str(round(f_max,2))+"Hz", (Y_s-100, 20), font1, fontScale1, fontColor1, thickness1, cv2.LINE_AA)
			im_m = cv2.putText(im_m,"Max: "+str(round(ffm_im,2)), (400, 20), font1, fontScale1, (0,1.0,0.5), thickness1, cv2.LINE_AA)
			real_m = cv2.putText(real_m,"REalM: "+str(round(ffm_real,2)), (400, 20), font1, fontScale1, (0,0.5,1.0), thickness1, cv2.LINE_AA)
			freq_m = cv2.putText(freq_m,"Max: "+str(round(f_max,2)), (400, 20), font1, fontScale1, (1.0,0,0.5), thickness1, cv2.LINE_AA)
			amp_m = cv2.putText(amp_m, "Current AMP: "+str(round(local_max,2)), (400,20), font1, fontScale1, (1.0,0.5, 0), thickness1, cv2.LINE_AA)
				
			if amp_count > 252:
				amp_m = cv2.putText(amp_m, str(round(amp_norm,2)), (520,110), font1, 0.8, (0.7,0.7,0.7), thickness1, cv2.LINE_AA)
				freq_m = cv2.putText(freq_m, str(round(freq_norm,2))+ "Hz", (520,110),  font1, 0.8, (0.7,0.7,0.7), thickness1, cv2.LINE_AA)

			# ---------------------------------------------------------------
			# EXPERIMENTAL PROCEDURES
			# ---------------------------------------------------------------
			if expType == 1: # PROCEDURE 3
				if amp_count > 252 and expProcedureState == 1:

					exp_file = open('exp_dataset/experiment_'+str(exp3CounterCSV)+'.csv', mode = 'w')
					onetime_c = 1
					exp_writer = csv.writer(exp_file)
					file_open = 1
					exp_writer.writerow(['force','amp','freq'])
					forceControlPub.publish(-5.0)
					expProcedureState = 2		

				if expProcedureState == 2 and forceControlCallbackFlag == 1:
					file_open = 0
					exp_file.close()
					onetime_c = 0
					force_thr = 0
						
					if exp3CounterCSV > N_of_exp:
						expProcedureState = 0
						forceControlCallbackFlag = 0
						rospy.loginfo("RELEASING")
						forceControlPub.publish(-10.0)
					else:
						expProcedureState = 1
			elif expType == 2: # PROCEDURE 4
				if amp_count > 252 and expProcedureState == 1:
					exp_file = open('exp_dataset/config/experiment_'+str(exp4CounterCSV)+'.csv', mode = 'w')
					expProcedureState = 2
					exp4CounterCSV = exp4CounterCSV + 1
					current_time = datetime.datetime.now()
					fft_real_file = open('fft_data_real'+str(current_time.minute)+str(current_time.second)+'.csv', mode = 'w')
					fft_im_file = open('fft_data_im'+str(current_time.minute)+str(current_time.second)+'.csv', mode = 'w')
				
					fft_real_writer = csv.writer(fft_real_file)
					fft_im_writer = csv.writer(fft_im_file)

					exp_writer = csv.writer(exp_file)
					file_open = 1
					exp_writer.writerow(['force', 'gripperPosition', 'freq', 'amp', 'touch_event'])
					forceControlPub.publish(-15.0)
					
				if expProcedureState == 2 and forceControlCallbackFlag == 1:
					file_open = 0
					exp_file.close()
					fft_real_file.close()
					fft_im_file.close()

					expProcedureState = 0
					forceControlCallbackFlag = 0
					rospy.loginfo(spike_count)
					spike_count = 0
					rospy.loginfo("Configuration experiment finished")
					rospy.loginfo("RELEASING")


					time.sleep(1)

					if exp4CounterCSV < 9:
						expProcedureState = 1
						rospy.loginfo(exp4CounterCSV)
					else:
						forceControlPub.publish(-10.0)
						rospy.loginfo("DONE")
						expProcedureState = 0
				 					
		if forceControlCallbackFlag == 2 and expType == 1:
			exp_writer.writerow([abs(wittForceZ), local_max, f_max])
		if forceControlCallbackFlag == 2 and expType == 2:
			fft_save_c = fft_save_c + 1
			if fft_save_c == 100:
				fft_save_sount = 1
				fft_im_writer.writerow(yf_im)
				fft_real_writer.writerow(yf_real)
			if fft_save_c == 400:
				fft_save_sount = 1
				fft_im_writer.writerow(yf_im)
				fft_real_writer.writerow(yf_real)
			if sum_interval-np.mean(mean_archive_2) > 145 and spike_count == 0:
				touch_spike = 1
				spike_count = 1
				fft_im_writer.writerow(yf_im)
				fft_real_writer.writerow(yf_real)
				rospy.loginfo("touch")
			exp_writer.writerow([abs(wittForceZ), gripperPosition, sum_interval, np.mean(mean_archive_2), touch_spike])
			if spike_count == 1:
				touch_spike = 0
				spike_count = 2
				
		main_image = cv2.vconcat([padx, spectr_image, padx, image, padx, force_pad])
		right_image = cv2.vconcat([sup_x, im_m, sup_x, real_m, sup_x, freq_m, sup_x, amp_m, sup_x])
		main_image2 = cv2.hconcat([pady, main_image, pady, right_image, pady])
		final_Y = np.shape(main_image2)[0]
		final_X = np.shape(main_image2)[1]
		cv2.putText(main_image2,"FORCE: " + str(forceTrackbarPos/50.0) + " N", (10, final_Y-8),font1, 1.5, (0.0,0.0,0.0), 2, cv2.LINE_AA)
		cv2.putText(main_image2, "CURRENT FORCE: " + str(round(-wittForceZ,2)) + " N", (final_X/2-300, final_Y-8),font1, 1.5, (0.5,0.0,0.0), 2, cv2.LINE_AA)
		cv2.imshow("FFT", main_image2)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
			cv2.destroyAllwindows()
		rate.sleep()

if __name__ == '__main__':
	fft_main()
