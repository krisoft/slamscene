#include "FrameReader.h"

#include "glog/logging.h"

using namespace lsd_slam;

FrameReader::FrameReader(H5::H5File &file)
{
	this->file = file;

	frames_dataset = file.openDataSet( "frames" );
	frames_dataspace = frames_dataset.getSpace();

	CHECK_EQ(frames_dataspace.getSimpleExtentNdims(), 3) << "wrong rank of frames";

	hsize_t frames_dims[3];
    frames_dataspace.getSimpleExtentDims( frames_dims, NULL);
    frame_count = frames_dims[0];
    h_inp = frames_dims[1];
    w_inp = frames_dims[2];

    H5::Attribute frames_calibration = frames_dataset.openAttribute("calibration");
    H5::StrType stype = frames_calibration.getStrType();
    std::string callibration_str;
    frames_calibration.read(stype, callibration_str);
    undistorter = Undistorter::getUndistorterFromString( callibration_str );

    // check sizes
    CHECK_EQ(h_inp, undistorter->getInputHeight()) << "frame height inconsistency";
    CHECK_EQ(w_inp, undistorter->getInputWidth()) << "frame width inconsistency";
};

int FrameReader::getCount()
{
	return frame_count;
}

std::shared_ptr<Undistorter> FrameReader::getUndistorter()
{
	return undistorter;
}


cv::Mat FrameReader::getFrame(int frame_id)
{
	CHECK_LT( frame_id, frame_count ) << "frame_id over indexing";
	CHECK_GE( frame_id, 0 ) << "frame_id under indexing";

	hsize_t      offset[3];   // hyperslab offset in the file
	hsize_t      count[3];    // size of the hyperslab in the file
	offset[0] = frame_id;
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

    cv::Mat image = cv::Mat(h_inp,w_inp,CV_8U);
    frames_dataset.read( image.data, H5::PredType::NATIVE_UINT8, memspace, frames_dataspace );
    return image;
}