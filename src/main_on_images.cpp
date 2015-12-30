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

#include "LiveSLAMWrapper.h"

#include "glog/logging.h"
#include "gflags/gflags.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include "util/Undistorter.h"
#include "util/FrameReader.h"

#include "opencv2/opencv.hpp"

#include "H5Cpp.h"

DEFINE_string(file, "", "HDF5 file containing the frames to process.");

using namespace lsd_slam;
int main( int argc, char** argv )
{

	gflags::ParseCommandLineFlags(&argc, &argv, true);
	google::InitGoogleLogging(argv[0]);

	H5::H5File file( FLAGS_file, H5F_ACC_RDWR );
	FrameReader frame_reader( file );

	/*cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );

	for(unsigned int i=0;i<frame_reader.getCount();i++)
	{
		std::cout << i << std::endl;
		cv::Mat image = frame_reader.getFrame( i );
		cv::imshow( "Display window", image );
		cv::waitKey(0);                        
	}*/

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

	// HZ defaults to 0. means full processing for each frame.
	double hz = 0;

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
			system->randomInit(image.data, fakeTimeStamp, runningIDX);
		else
			system->trackFrame(image.data, runningIDX ,hz == 0,fakeTimeStamp);
		runningIDX++;
		fakeTimeStamp+=0.03;

		if(fullResetRequested)
		{

			printf("FULL RESET!\n");
			delete system;

			system = new SlamSystem(w, h, K, doSlam);
			system->setVisualization(outputWrapper);

			fullResetRequested = false;
			runningIDX = 0;
		}

		SE3 pose = system->getCurrentPoseEstimate();
		std::cout << "pose:" << std::endl << pose.matrix() << std::endl;

	}


	system->finalize();



	delete system;
	// delete outputWrapper;
	return 0;

	/*// get camera calibration in form of an undistorter object.
	// if no undistortion is required, the undistorter will just pass images through.
	std::string calibFile;
	Undistorter* undistorter = 0;
	undistorter = Undistorter::getUndistorterForFile(FLAGS_calib.c_str());

	if(undistorter == 0)
	{
		printf("need camera calibration file! (set using _calib:=FILE)\n");
		exit(0);
	}

	



	
	H5::DataSet frames_dataset = file.openDataSet( "frames" );
	H5::DataSpace frames_dataspace = frames_dataset.getSpace();

	CHECK_EQ(frames_dataspace.getSimpleExtentNdims(), 3) << "wrong rank of frames";

	hsize_t frames_dims[3];
    frames_dataspace.getSimpleExtentDims( frames_dims, NULL);

    int frame_count = frames_dims[0];
    CHECK_EQ(frames_dims[1], h_inp) << "frame height inconsistency";
    CHECK_EQ(frames_dims[2], w_inp) << "frame width inconsistency";

    CHECK_EQ(frames_dataset.getTypeClass(), H5T_INTEGER ) << "type inconsistency, wrong class";
    CHECK_EQ(frames_dataset.getIntType().getSize(), 1 ) << "type inconsistency, wrong size";

    H5::Attribute frames_calibration = frames_dataset.openAttribute("calibration");
    H5::StrType stype = frames_calibration.getStrType();
    std::string frames_calibration_str;
    frames_calibration.read(stype, frames_calibration_str);

	hsize_t      offset[3];   // hyperslab offset in the file
	hsize_t      count[3];    // size of the hyperslab in the file
	offset[0] = 54;
	offset[1] = 0;
	offset[2] = 0;
	count[0]  = 1;
	count[1]  = h_inp;
	count[2]  = w_inp;
	frames_dataspace.selectHyperslab( H5S_SELECT_SET, count, offset );

	hsize_t     dimsm[2];             
	dimsm[0] = h_inp;
	dimsm[1] = w_inp;
	H5::DataSpace memspace( 2, dimsm );

    cv::Mat image = cv::Mat(h,w,CV_8U);
    frames_dataset.read( image.data, H5::PredType::NATIVE_UINT8, memspace, frames_dataspace );

    // Create a window for display.
    cv::imshow( "Display window", image );                   // Show our image inside it.

    cv::waitKey(0);                        
	*/
}
