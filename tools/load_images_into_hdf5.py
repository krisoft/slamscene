#!/usr/bin/env python

import argparse
import h5py
import numpy as np
import cv2
import os

if __name__=="__main__":

	parser = argparse.ArgumentParser(description='Load images from a directory into a hdf5 file')
	parser.add_argument('input', help='path to directory')
	parser.add_argument('calib', help='path to calibration file')
	parser.add_argument('output', help='path of hdf5 file to output')

	args = parser.parse_args()

	# read calibration
	calibration = open(args.calib,"r").read()

	# get filenames
	filenames = []
	for filename in os.listdir( args.input ):
		if filename.endswith(".jpg") or filename.endswith(".jpeg") or filename.endswith(".png"):
			fullpath = os.path.join( args.input, filename )
			filenames.append( fullpath )
	filenames.sort()

	outp = h5py.File(args.output, "w")
	dset = None
	shape = None
	for idx, filename in enumerate(filenames):
		img = cv2.imread(filename,cv2.IMREAD_GRAYSCALE )
		if dset==None:
			# dataset not initialized yet, create now
			shape = img.shape
			assert( len(shape)==2 )
			dset_shape = ( len(filenames), shape[0], shape[1] )
			dset = outp.create_dataset("frames", dset_shape, dtype=img.dtype, compression="gzip")
			dset.attrs['calibration'] = calibration
		assert( shape==img.shape )
		dset[idx,:,:] = img
	outp.close()
