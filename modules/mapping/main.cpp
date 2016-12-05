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
	float rot_ang_deg = 30.0f; // Rotation angle in degrees

#if SERVO_ENABLE
	if(argc < 2)
	{
		std::cerr << "Usage: ./main numSteps(0-13)" << std::endl;
		exit(-1);
	}

	numSteps  = atoi(argv[1]);

	//Initialize servo
	if(dxl_initialize(deviceIndex, baudnum) == 0)
	{
		std::cerr << "Error initiliazing servo. Exiting..." << std::endl;
		exit(-1);
	}

	dxl_set_speed(254, 3.42f);
#endif

    pcl::PointCloud<PointColor>::Ptr input_cloud(new pcl::PointCloud<PointColor>);
	cv::Mat rgbIm;
	PointCloudIO<PointColor> *pio = new PointCloudIO<PointColor>();
	CloudOps<PointColor> *co = new CloudOps<PointColor>();

	// All the bounding rectangles
	std::vector<cv::Rect> boundRectOut;

	// Stack of all clouds
	std::vector<pcl::PointCloud<PointColor>::Ptr> cloud_stack;

	// Corner points
	std::vector<Eigen::Vector4f> points_out;
	int i = 0;

#if SERVO_ENABLE
	for(i = 0; i < numSteps; i++)
	{
		dxl_rotate_by(254, rot_ang_deg * i);
		sleep(4);
#endif

		if(pio->getPointCloudAndIm(input_cloud, rgbIm) < 0)
		{
			std::cerr << "Critical Error. Exiting..." << std::endl;
			exit(-1);
		}
		pio->savePointCloud(input_cloud, "cloud.pcd");
		std::cout << "Saved point cloud..." << std::endl;
		pio->saveImage(rgbIm, "rgb.png");


		co->setInputCloud(input_cloud);
		std::cout << "Set input point cloud..." << std::endl;
		co->apply_transform(0, 0, 0, i * rot_ang_deg);
		std::cout << "applied transform" << std::endl;
		Eigen::Vector4f final_mean = co->getIntersectionPoints(points_out, 0.001);

		WindowDetector *wd = new WindowDetector("../resources/model.yml");
		std::cout << "Loaded model" << std::endl;
		if(!wd->readImage("gray.pgm"))
		{
			wd->detectEdges();
			wd->detectRectangles(boundRectOut, true);
			// TODO: Find corresponding rectangle in point cloud
			std::cout << "Detected windows :" << boundRectOut.size() << std::endl;
			co->printWorldCoords(124);
		}

#if SERVO_ENABLE
		cloud_stack.push_back(co->curr_cloud);
	}
#endif

#if SERVO_ENABLE
	pcl::PointCloud<PointColor>::Ptr stitched_cloud(new pcl::PointCloud<PointColor>);
	for(int i = 0; i < cloud_stack.size(); ++i)
		*stitched_cloud += *cloud_stack[i];
	pio->savePointCloud(stitched_cloud, "final_cloud.pcd");
#endif

	// Visualizer
	pcl::visualization::PCLVisualizer viewer;
	
#if SERVO_ENABLE
	viewer.addPointCloud(stitched_cloud, "final_cloud");
#else
	viewer.addPointCloud(input_cloud, "input_cloud");
	pcl::PointXYZ center;

	center.x = final_mean.x();
	center.y = final_mean.y();
	center.z = final_mean.z();

	viewer.addSphere(center, 0.05, 1.0, 1.0, 1.0, "center1");
#endif
	while(!viewer.wasStopped())
		viewer.spinOnce(100);

	return 0; 
}
