/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "glog/logging.h"
#include "gflags/gflags.h"

#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#include "LiveSLAMWrapper.h"
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"
#include "DataStructures/Frame.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include "util/Undistorter.h"
#include "util/FrameReader.h"
#include "util/PoseWriter.h"
#include "util/TrackFrameDebug.h"

#include "opencv2/opencv.hpp"

#include "H5Cpp.h"

DEFINE_string(input, "", "HDF5 file containing the frames to process.");
DEFINE_string(output, "", "HDF5 output file path.");
DEFINE_string(debug, "", "debug dir path.");

using namespace lsd_slam;
int main( int argc, char** argv )
{

	gflags::ParseCommandLineFlags(&argc, &argv, true);
	google::InitGoogleLogging(argv[0]);

	
	boost::filesystem::copy_file(FLAGS_input, FLAGS_output, boost::filesystem::copy_option::overwrite_if_exists);

	H5::H5File file( FLAGS_output, H5F_ACC_RDWR );

	FrameReader frame_reader( file );
	PoseWriter pose_writer( file );

	std::shared_ptr<Undistorter> undistorter = frame_reader.getUndistorter();

	int w = undistorter->getOutputWidth();
	int h = undistorter->getOutputHeight();

	int w_inp = undistorter->getInputWidth();
	int h_inp = undistorter->getInputHeight();

	float fx = undistorter->getK().at<double>(0, 0);
	float fy = undistorter->getK().at<double>(1, 1);
	float cx = undistorter->getK().at<double>(2, 0);
	float cy = undistorter->getK().at<double>(2, 1);
	Sophus::Matrix3f K;
	K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

	cv::Mat image = cv::Mat(h,w,CV_8U);
	int runningIDX=0;
	float fakeTimeStamp = 0;

	// make output wrapper. just set to zero if no output is required.
	Output3DWrapper* outputWrapper = 0;


	// make slam system
	SlamSystem* system = new SlamSystem(w, h, K, doSlam);
	system->setVisualization(outputWrapper);
	

	for(unsigned int i=0;i<frame_reader.getCount();i++)
	{
		std::cout << i << std::endl;
		cv::Mat imageDist = frame_reader.getFrame( i );

		assert(imageDist.type() == CV_8U);

		undistorter->undistort(imageDist, image);
		assert(image.type() == CV_8U);

		if(runningIDX == 0)
		{
			system->randomInit(image.data, fakeTimeStamp, runningIDX);
		} else {
			std::stringstream ss;
			ss << std::setw(5) << std::setfill('0') << i;
			H5::H5File file(
				FLAGS_debug + "/"+ ss.str() +".hdf5",
				H5F_ACC_TRUNC
			);

			TrackFrameDebug trackFrameDebug( file );
			system->trackFrame(
				image.data,
				runningIDX,
				true, // block until mapped
				fakeTimeStamp,
				&trackFrameDebug
			);
		}
		runningIDX++;
		fakeTimeStamp+=0.03;

		Frame* keyFrame = system->getCurrentKeyframe();

		std::cout << i << " vs. " << keyFrame->id() << std::endl;

		SE3 pose = system->getCurrentPoseEstimate();
		pose_writer.writePose( i, pose );

	}



	delete system;
	return 0;
}
