#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "pointcloudio.hpp"
#include "cloud_ops.hpp"
#include "../objdet/windowdetector.h"
#include "dynamixel.h"
#include "apriltag_utils.h"

#define SERVO_ENABLE 1
#define USE_APRILTAG 0

int main(int argc, char **argv)
{
    int deviceIndex = 0;
    int baudnum = 34;
	int numSteps = 1;
	float rot_ang_deg = 10.0f; // Rotation angle in degrees

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
    pcl::PointCloud<PointColor>::Ptr cloud_out_icp(new pcl::PointCloud<PointColor>);
	cv::Mat rgbIm;
	PointCloudIO<PointColor> *pio = new PointCloudIO<PointColor>();
	CloudOps<PointColor> *co = new CloudOps<PointColor>();

	// All the bounding rectangles
	std::vector<cv::Rect> boundRectOut;

	// Stack of all clouds
	std::vector<pcl::PointCloud<PointColor> > cloud_stack;
	std::vector<pcl::ModelCoefficients::Ptr> line_coeffs_out;

	// Corner points
	std::vector<Eigen::Vector4f> corners;
	int i = 0;
	// Load trained model for detecting edges
	WindowDetector *wd = new WindowDetector("../resources/model.yml");
	std::cout << "Loaded model" << std::endl;

#if SERVO_ENABLE
	for(i = 0; i < numSteps; i++)
	{
		std::cout << "*****Beginning " << i+1 << "th iteration*****" << std::endl;
		dxl_rotate_by(254, rot_ang_deg * i);
		sleep(4);
#endif

		if(pio->getPointCloudAndIm(input_cloud, rgbIm) < 0)
		{
			std::cerr << "Critical Error. Exiting..." << std::endl;
			exit(-1);
		}
		std::cout << "Saved point cloud..." << std::endl;
		pio->saveImage(rgbIm, "rgb.png");

		// Copy curr Cloud to prev after 1st iteration
		if(i)
			co->savePrevCloud();
		
		co->setInputCloud(input_cloud);
		std::cout << "Set input point cloud" << std::endl;
		co->apply_transform(0, 0, 0, i * rot_ang_deg);
		std::cout << "Applied transform" << std::endl;
		if(i)
			co->incremental_icp(cloud_out_icp);
		else
			co->filter_cloud(input_cloud, cloud_out_icp);

		co->getIntersectionPoints(corners, line_coeffs_out, 0.001);

		if(!wd->readImage("gray.pgm"))
		{
			wd->detectEdges();
			wd->detectRectangles(boundRectOut, true);
			// TODO: Find corresponding rectangle in point cloud
			std::cout << "Detected windows :" << boundRectOut.size() << std::endl;
			co->printWorldCoords(124);
		}

#if SERVO_ENABLE
		cloud_stack.push_back(*cloud_out_icp);
		input_cloud->clear();
		cloud_out_icp->clear();
	}
#endif

#if SERVO_ENABLE
	pcl::PointCloud<PointColor>::Ptr stitched_cloud(new pcl::PointCloud<PointColor>);
	for(int i = 0; i < cloud_stack.size(); ++i)
	{
		*stitched_cloud += cloud_stack[i];
		// Filthy tactics to save memory
		cloud_stack[i].clear();
	}
	pio->savePointCloud(stitched_cloud, "final_cloud.pcd");
#endif

	// Visualizer
	pcl::visualization::PCLVisualizer viewer;
	
#if SERVO_ENABLE
	viewer.addPointCloud(stitched_cloud, "final_cloud");
	std::ofstream corner_file;
	corner_file.open("corners.csv");
	for(int i = 0; i < corners.size(); ++i)
	{
		pcl::PointXYZ centre;
		std::stringstream ss;

		centre.x = corners[i].x();
		centre.y = corners[i].y();
		centre.z = corners[i].z();
		corner_file << i << ":"
			<< corners[i].x() << ","
			<< corners[i].y() << ","
			<< corners[i].z() << ","
			<< std::endl;

		ss << "centre_" << i;

		viewer.addSphere(centre, 0.05, 1.0, 1.0, 1.0, ss.str());
		ss.clear();
		ss.str("");
	}
	corner_file.close();
	
	for(int i = 0; i < line_coeffs_out.size(); ++i)
	{
		std::stringstream ss;

		ss << "line_" << i;
		viewer.addLine(*(line_coeffs_out[i]), ss.str());
		ss.clear();
		ss.str("");
	}
#else
	viewer.addPointCloud(input_cloud, "input_cloud");
#endif
	while(!viewer.wasStopped())
		viewer.spinOnce(100);

	return 0; 
}
