#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "pointcloudio.hpp"
#include "cloud_ops.hpp"
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

    pcl::PointCloud<PointColor>::Ptr curr_cloud(new pcl::PointCloud<PointColor>);
	cv::Mat rgbIm;
	PointCloudIO<PointColor> pio;
	CloudOps<PointColor> co;

	pio.getPointCloudAndIm(curr_cloud, rgbIm);
	pio.savePointCloud(curr_cloud, "cloud.pcd");

	return 0;
}
