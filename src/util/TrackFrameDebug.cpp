#include "TrackFrameDebug.h"

#include <Tracking/TrackingReference.h>
#include <DataStructures/Frame.h>

#include "util/settings.h"

using namespace lsd_slam;

void TrackFrameDebug::write(
	TrackingReference* reference,
	Frame* frame,
	const SE3& frameToReference_initialEstimate
){
	for(int i=0; i<PYRAMID_LEVELS; i++)
	{
		writeFrameLevel( reference->keyframe, i, "reference" );
		writeFrameLevel( frame, i, "frame" );
	}

	writeMatrix(
		"frameToReference_initialEstimate",
		frameToReference_initialEstimate.data(),
		1,
		7
	);
}

void TrackFrameDebug::writeResult(
	const SE3& newRefToFrame_poseUpdate
){
	writeMatrix(
		"newRefToFrame_poseUpdate",
		newRefToFrame_poseUpdate.data(),
		1,
		7
	);
}


void TrackFrameDebug::writeFrameLevel(
	Frame* frame,
	int level,
	std::string name
		
){
	H5::DataSet dataset = writeMatrix(
		name + "_" + std::to_string(level) + "_image",
		frame->image(level),
		frame->width(level),
		frame->height(level)
	);

	writeAttributes( dataset, "id", frame->id() );
	writeAttributes( dataset, "level", level );

	writeAttributes( dataset, "fx", frame->fx(level) );
	writeAttributes( dataset, "fy", frame->fy(level) );
	writeAttributes( dataset, "cx", frame->cx(level) );
	writeAttributes( dataset, "cy", frame->cy(level) );

	if( frame->hasIDepthBeenSet() )
	{
		writeMatrix(
			name + "_" + std::to_string(level) + "_idepth",
			frame->idepth(level),
			frame->width(level),
			frame->height(level)
		);
		writeMatrix(
			name + "_" + std::to_string(level) + "_idepthVar",
			frame->idepthVar(level),
			frame->width(level),
			frame->height(level)
		);
	}
}


H5::DataSet TrackFrameDebug::writeMatrix(
		std::string key,
		const float* data,
		int w,
		int h
) {
	hsize_t dims[2]  = { static_cast<hsize_t>(h), static_cast<hsize_t>(w) };
	hsize_t offset[2]  = { 0, 0 };
	H5::DataSpace mspace( 2, dims);

	H5::DataSet dataset = file.createDataSet(key, H5::PredType::NATIVE_FLOAT, mspace);
	H5::DataSpace fspace = dataset.getSpace();
	fspace.selectHyperslab( H5S_SELECT_SET, dims, offset );
	dataset.write( data, H5::PredType::NATIVE_FLOAT, mspace, fspace );
	return dataset;
}

H5::DataSet TrackFrameDebug::writeMatrix(
		std::string key,
		const double* data,
		int w,
		int h
) {
	hsize_t dims[2]  = { static_cast<hsize_t>(h), static_cast<hsize_t>(w) };
	hsize_t offset[2]  = { 0, 0 };
	H5::DataSpace mspace( 2, dims);

	H5::DataSet dataset = file.createDataSet(key, H5::PredType::NATIVE_DOUBLE, mspace);
	H5::DataSpace fspace = dataset.getSpace();
	fspace.selectHyperslab( H5S_SELECT_SET, dims, offset );
	dataset.write( data, H5::PredType::NATIVE_FLOAT, mspace, fspace );
	return dataset;
}

void TrackFrameDebug::writeAttributes(
	H5::DataSet& dataset,
	std::string key,
	int data
) {
	H5::IntType int_type(H5::PredType::NATIVE_INT);
	H5::DataSpace att_space(H5S_SCALAR);
	H5::Attribute att = dataset.createAttribute(key, int_type, att_space );
	att.write( int_type, &data );
}

void TrackFrameDebug::writeAttributes(
	H5::DataSet& dataset,
	std::string key,
	float data
) {
	H5::FloatType float_type(H5::PredType::NATIVE_FLOAT);
	H5::DataSpace att_space(H5S_SCALAR);
	H5::Attribute att = dataset.createAttribute(key, float_type, att_space );
	att.write( float_type, &data );
}