#include "PoseWriter.h"

#include "glog/logging.h"

#include <iostream>

using namespace lsd_slam;

PoseWriter::PoseWriter(H5::H5File &file)
{
	this->file = file;

	hsize_t      dims[2]  = { 0, 12};  // dataset dimensions at creation
	hsize_t      maxdims[2] = {H5S_UNLIMITED, 12};
	H5::DataSpace mspace( 2, dims, maxdims);

	H5::DSetCreatPropList cparms;
	hsize_t      chunk_dims[2] ={1, 12};
	cparms.setChunk( 2, chunk_dims );

	dataset = file.createDataSet( "poses", H5::PredType::NATIVE_DOUBLE, mspace, cparms);

	frame_count = 0;
}



void PoseWriter::writePose( int frame_idx, SE3 pose )
{
	if( frame_count <= frame_idx )
	{
		frame_count = frame_idx+1;
		hsize_t      size[2];
		size[0]   = frame_count;
		size[1]   = 12;
		dataset.extend( size );
	}

	Eigen::Matrix<double,3,4, Eigen::RowMajor> matrix = pose.matrix3x4();

	H5::DataSpace fspace = dataset.getSpace ();
	hsize_t      dims[2]  = { 1, 12};
	hsize_t      offset[2]  = { static_cast<hsize_t>(frame_idx), 0};
	fspace.selectHyperslab( H5S_SELECT_SET, dims, offset );
	H5::DataSpace mspace( 2, dims );

	dataset.write( matrix.data(), H5::PredType::NATIVE_DOUBLE, mspace, fspace );
}


