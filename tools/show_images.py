#!/usr/bin/env python

import argparse
import h5py
import numpy as np
import cv2
import cvutil
import os

if __name__=="__main__":

	parser = argparse.ArgumentParser(description='Display images from an hdf5 file')
	parser.add_argument('input', help='path to the hdf5 file')

	args = parser.parse_args()

	inp = h5py.File(args.input, "r")
	frames = inp["frames"]
	frame_count = frames.shape[0]
	for idx in range(54,frame_count):
		print idx
		img = frames[idx,::]
		cv2.imshow('image',img)
		cvutil.wait()
