#!/usr/bin/env python

import argparse
import h5py
import numpy as np
import cv2
import os
import json

if __name__=="__main__":

	parser = argparse.ArgumentParser(description='Load images from a directory into a hdf5 file')
	parser.add_argument('input', help='path to video')
	parser.add_argument('calib', help='path to calibration json')
	parser.add_argument('output', help='path of hdf5 file to output')

	args = parser.parse_args()

	# read calibration
	calibration = json.load(open(args.calib,"r"))
	assert( abs(calibration["width"]/float(calibration["height"])- 640./480.)<0.001 )

	calibration["fx"] = calibration["fx"]/float(calibration["width"])
	calibration["fy"] = calibration["fy"]/float(calibration["height"])

	calibration["cx"] = calibration["cx"]/float(calibration["width"])
	calibration["cy"] = calibration["cy"]/float(calibration["height"])

	# check video parameters
	cap = cv2.VideoCapture(args.input)

	in_width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
	in_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

	print "checking length"
	frame_count = 0
	while(True):
		ret, frame = cap.read()
		if not ret:
			break
		frame_count += 1

	cap = cv2.VideoCapture(args.input)

	out_width = 640
	out_height = 480

	scale = max(480.0/in_height, 640/in_width)

	resize_width = int(in_width*scale)
	resize_height = int(in_height*scale)

	if resize_width==out_width:
		assert( resize_height >= out_height )
		offs_x = 0
		offs_y = (resize_height-out_height)/2
	elif resize_height==out_height:
		assert( resize_width >= out_width )
		offs_x = (resize_width-out_width)/2
		offs_y = 0
	else:
		raise Exception("can't be")


	# create callibration string

	callibration_row = [
		calibration["fx"] * resize_width,
		calibration["fy"] * resize_height,
		calibration["cx"] * resize_width - offs_x,
		calibration["cy"] * resize_height - offs_y,
	]
	callibration_row += calibration["d"]


	callibration_str = "%s\n%i %i\nfull\n%i %i\n"%(
		" ".join(map( lambda f:str(f), callibration_row)),
		out_width, out_height,
		out_width, out_height
	)

	outp = h5py.File(args.output, "w")
	dset = None
	shape = None
	frame_id = 0

	while(True):
		# Capture frame-by-frame
		ret, frame = cap.read()
		if not ret:
			break

		print "%i/%i"%(frame_id, frame_count)

		# Our operations on the frame come here
		img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		img = cv2.resize(img, (resize_width,resize_height))
		img = img[ offs_y : (offs_y+out_height), offs_x : (offs_x+out_width) ] # crop

		if dset==None:
			# dataset not initialized yet, create now
			shape = img.shape
			assert( len(shape)==2 )
			dset_shape = ( frame_count, shape[0], shape[1] )
			dset = outp.create_dataset("frames", dset_shape, dtype=img.dtype, compression="gzip")
			dset.attrs['calibration'] = callibration_str
		assert( shape==img.shape )
		dset[frame_id,:,:] = img

		frame_id += 1