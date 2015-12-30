#pragma once

#include "H5Cpp.h"

#include "util/Undistorter.h"

#include "opencv2/opencv.hpp"

#include <memory>

namespace lsd_slam
{

class FrameReader
{
public:
	FrameReader(H5::H5File &file);

	int getCount();
	std::shared_ptr<Undistorter> getUndistorter();
	cv::Mat getFrame(int frame_id);
private:
	H5::H5File file;
	H5::DataSet frames_dataset;
	H5::DataSpace frames_dataspace;

	int h_inp;
	int w_inp;
	int frame_count;
	std::shared_ptr<Undistorter> undistorter;
};

}