#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <string>
#include "pointcloudio.h"

int PointCloudIO::getPointCloudAndIm(pcl::PointCloud<PointColor>::Ptr curr_cloud, 
		cv::Mat &rgbIm)
{
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0; 
	std::string serial = "";

	cv::Mat grayMat;

	// Set cloud dimensions
	curr_cloud->width = 512;
	curr_cloud->height = 424;
	curr_cloud->is_dense = false;
	// Set cloud dimensions
	curr_cloud->points.resize(
			curr_cloud->width * curr_cloud->height);

	if(freenect2.enumerateDevices() == 0)
	{
		std::cerr << "No device connected" << std::endl;
		return -1;
	}

	// OPENGL by default
	if(!pipeline)
	{
		pipeline = new libfreenect2::OpenGLPacketPipeline();
	} 
	if(serial == "")
	{
		serial = freenect2.getDefaultDeviceSerialNumber();
		std::cout << "Serial No: " << serial << std::endl;
	}   

	if(pipeline)
	{
		//Open device
		dev = freenect2.openDevice(serial, pipeline);
	}

	// listeners
	// Enable rgb and depth by default
	int types = 0;
	types |= libfreenect2::Frame::Color;
	types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

	libfreenect2::SyncMultiFrameListener listener(types);
	libfreenect2::FrameMap frames;

	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);

	// start
	if(!dev->start())
		return -1;

	// registration
	libfreenect2::Registration* registration = 
		new libfreenect2::Registration(
				dev->getIrCameraParams(),
				dev->getColorCameraParams());

	libfreenect2::Frame undistorted(512, 424, 4), 
		registered(512, 424, 4);

	if(!listener.waitForNewFrame(frames, 10*1000)) // 10 seconds 
	{ 
		std::cout << "timeout!" << std::endl; 
		return -1; 
	} 

	// Grab individual frames from frameMap
	libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color]; 
	libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir]; 
	libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth]; 

	registration->apply(rgb, depth, &undistorted, &registered,
			true);

	// Deal with rgb frame
	cv::Mat(rgb->height, rgb->width, 
			CV_8UC4, rgb->data).copyTo(rgbIm);
	cv::cvtColor(rgbIm, grayMat, CV_BGRA2GRAY);

	// Reflect image before saving
	cv::flip(grayMat, grayMat, 1);
	// Save gray image for apriltag TODO: Remove later
	cv::imwrite("gray.pgm", grayMat);

	// Kinect image size 512x424
	float x, y, z, rgbData;
	int idt, idx;
	for(int r = 0; r < 424; ++r)
	{
		idt = r * 512;
		for(int c = 0; c < 512; ++c)
		{
			idx = idt + c;
			registration->getPointXYZRGB(&undistorted, &registered, 
					r, c, x, y, z, rgbData);
			//Skip if NaN values
			if(!(isnan(x) || isnan(y) || isnan(z)))
			{
				curr_cloud->points[idx].x = x;
				curr_cloud->points[idx].y = y;
				curr_cloud->points[idx].z = z;
				curr_cloud->points[idx].rgb = rgbData;
			}
		}
	}

	// TODO: Flip point cloud as well?

	dev->stop();
	dev->close();
	return 0;
}

/**
 * @brief Save point cloud to pcd file
 *
 * TODO: Add options for other formats.
 */
void PointCloudIO::savePointCloud( pcl:: PointCloud<PointColor>::Ptr curr_cloud, 
		std::string cloud_name)
{
	pcl::io::savePCDFileASCII(cloud_name, *curr_cloud);
}

/**
 * @brief Save rgb image to file
 */
void PointCloudIO::saveImage(cv::Mat rgbIm, std::string im_name)
{
	cv::imwrite(im_name, rgbIm);
}
