#pragma once

#include "H5Cpp.h"
#include "util/SophusUtil.h"

#include <memory>

namespace lsd_slam
{

class TrackingReference;
class Frame;

class TrackFrameDebug
{
public:
	TrackFrameDebug(H5::H5File &file) : file(file) {};

	void write(
		TrackingReference* reference,
		Frame* frame,
		const SE3& frameToReference_initialEstimate
	);

	void writeResult(
		const SE3& newRefToFrame_poseUpdate
	);
private:
	H5::H5File file;

	void writeFrameLevel(
		Frame* frame,
		int level,
		std::string name
	);

	H5::DataSet writeMatrix(
		std::string name,
		const float* data,
		int w,
		int h
	);

	H5::DataSet writeMatrix(
		std::string name,
		const double* data,
		int w,
		int h
	);

	void writeAttributes(
		H5::DataSet& dataset,
		std::string key,
		int data
	);

	void writeAttributes(
		H5::DataSet& dataset,
		std::string key,
		float data
	);
};

}