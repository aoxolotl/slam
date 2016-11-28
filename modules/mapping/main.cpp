#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "pointcloudio.hpp"
#include "cloud_ops.hpp"
#include "../objdet/windowdetector.h"
#include "dynamixel.h"
#include "apriltag_utils.h"

#define SERVO_ENABLE 0
#define USE_APRILTAG 0

int main(int argc, char **argv)
{
    int deviceIndex = 0;
    int baudnum = 34;
	int numSteps = 1;

#if SERVO_ENABLE
	if(argc < 2)
	{
		std::cerr << "Usage: ./main numSteps(0-12)" << std::endl;
		exit(-1);
	}

	numSteps  = atoi(argv[1]);
#endif

    pcl::PointCloud<PointColor>::Ptr input_cloud(new pcl::PointCloud<PointColor>);
	cv::Mat rgbIm;
	PointCloudIO<PointColor> *pio = new PointCloudIO<PointColor>();
	CloudOps<PointColor> *co = new CloudOps<PointColor>();

	if(pio->getPointCloudAndIm(input_cloud, rgbIm) < 0)
	{
		std::cerr << "Critical Error. Exiting..." << std::endl;
		exit(-1);
	}
	pio->savePointCloud(input_cloud, "cloud.pcd");
	pio->saveImage(rgbIm, "rgb.png");
	
	co->setInputCloud(input_cloud);

	WindowDetector *wd = new WindowDetector("resources/model.yml");
	if(wd->readImage("rgb.png") > 0)
	{
		wd->detectEdges();
		wd->detectRectangles(true);
	}	

	return 0;
}
