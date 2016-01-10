#pragma once

#include "H5Cpp.h"

#include "util/SophusUtil.h"

namespace lsd_slam
{

class PoseWriter
{
public:
	PoseWriter(H5::H5File &file);

	void writePose( int frame_idx, SE3 pose );
private:
	H5::H5File file;
	H5::DataSet dataset;
	H5::DataSpace dataspace;

	int frame_count;
};

}