#ifndef POINTCLOUDIO_H
#define POINTCLOUDIO_H
#include <opencv2/core/core.hpp>
#include "pcl_typedefs.h"

/** 
 * Operation related to grabbing, saving and loading point cloud.
 *
 * Currently only deals with PointXYZRGBA type.
 */
class PointCloudIO
{
	public:
		/**
		 * @brief Grab pointcloud using kinect. 
		 * @param curr_cloud Place to store grabbed point cloud
		 * Grabs both pointcloud and the associated rgbImage.
		 * TODO: Replace with OpenNI
		 */
		int getPointCloudAndIm(pcl:: PointCloud<PointColor>::Ptr curr_cloud, 
				cv::Mat &rgbIm);

		/**
		 * @brief Save point cloud to pcd file
		 * @param curr_cloud PointCloud to save
		 * @param cloud_name Saved file name
		 * TODO: Add options for other formats.
		 */
		void savePointCloud(pcl:: PointCloud<PointColor>::Ptr curr_cloud, 
				std::string cloud_name);

		/**
		 * @brief Save rgb image to file
		 *
		 */
		void saveImage(cv::Mat rgbIm, std::string im_name);
};

#endif //POINTCLOUDIO_H
