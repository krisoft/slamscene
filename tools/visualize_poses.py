#!/usr/bin/env python

import argparse
import h5py
import numpy as np
import cv2
import cvutil
import os
import math

import matplotlib.pyplot as plt



def matrix2euler( m ):
	if m[1,0] > 0.998: # singularity at north pole
		heading = math.atan2(m[0,2],m[2,2])
		attitude = math.pi/2;
		bank = 0
		return heading, attitude, bank
	if m[1,0] < -0.998: # singularity at south pole
		heading = math.atan2(m[0,2],m[2,2])
		attitude = -math.pi/2
		bank = 0
		return heading, attitude, bank

	heading = math.atan2(-m[2,0],m[0,0])
	bank = math.atan2(-m[1,2],m[1,1])
	attitude = math.asin(m[1,0])
	return heading, attitude, bank


if __name__=="__main__":

	parser = argparse.ArgumentParser(description='Display poses from an hdf5 file')
	parser.add_argument('input', help='path to the hdf5 file')

	args = parser.parse_args()

	inp = h5py.File(args.input, "r")
	frames = inp["frames"]
	frame_count = frames.shape[0]

	poses = inp["poses"]
	assert( len(poses.shape)==2 )
	assert( poses.shape[0]==frame_count )
	assert( poses.shape[1]==12 )


	Tx = []
	Ty = []
	Tz = []

	Rh = []
	Ra = []
	Rb = []

	for idx in range(frame_count):
		transform =  poses[idx,:].reshape(3,4)
		r = transform[ :, :-1 ]
		heading, attitude, bank = matrix2euler( r )

		Tx.append( transform[ 0, 3 ] )
		Ty.append( transform[ 1, 3 ] )
		Tz.append( transform[ 2, 3 ] )


		Rh.append( math.degrees( heading ) )
		Ra.append( math.degrees( attitude ) )
		Rb.append( math.degrees( bank ) )

	plt.subplot(2, 1, 1)
	plt.plot(Tx, label="Tx")
	plt.plot(Ty, label="Ty")
	plt.plot(Tz, label="Tz")
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=1)

	plt.subplot(2, 1, 2)
	plt.plot(Rh, label="Rh")
	plt.plot(Ra, label="Ra")
	plt.plot(Rb, label="Rb")

	plt.grid(True)
	plt.legend(bbox_to_anchor=(0, 1), loc='upper left', ncol=1)
	plt.show()
