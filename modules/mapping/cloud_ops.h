/** 
 * @file cloud_ops.hpp
 * Template class for cloud operations
 */
#ifndef CLOUD_OPS_H
#define CLOUD_OPS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_typedefs.h"

#define SQ(a) (a * a)

/**
 * @class CloudOps
 * @brief Template class for operations on point clouds
 *
 * After setting an initial point cloud, all operations such as
 * rotation, translation, plane segmentation are all done on this 
 * input cloud.
 */
class CloudOps
{
	private:
		pcl:: PointCloud<PointColor>::Ptr prev_cloud; //< previous cloud

	public:
		pcl:: PointCloud<PointColor>::Ptr curr_cloud; //< input cloud

		CloudOps()
			:curr_cloud(new pcl:: PointCloud<PointColor>)
			, prev_cloud(new pcl:: PointCloud<PointColor>)
		{
		}

		/**
		 * @brief Set the input cloud on which all operations are 
		 * to be done.
		 *
		 * As opposed to the usual pcl implementation of setInputCloud,
		 * a deep copy of the point cloud is done. This is clearer to 
		 * deal with than a simple pointer copy.
		 *
		 * @param[in] cloud_in shared_ptr to input cloud
		 */
		void setInputCloud(
				pcl:: PointCloud<PointColor>::Ptr cloud_in);

		void savePrevCloud();

		/**
		 * @brief Downsample the cloud using VoxelGrid
		 * @param[in] cloud_in Input Point Cloud
		 * @param[out] cloud_out Filtered/Downsampled cloud
		 */
		void filter_cloud(
				pcl:: PointCloud<PointColor>::Ptr cloud_in, 
				pcl:: PointCloud<PointColor>::Ptr cloud_out);


		/**
		 * @brief Apply translation and rotation to input cloud
		 */
		void apply_transform(float tx, float ty, float tz, float theta);

		/**
		 * @brief Register clouds using ICP
		 * @param[out] Transformed cloud
		 */
		void incremental_icp(pcl:: PointCloud<PointColor>::Ptr cloud_out);

		/**
		 * @brief Segment point cloud into planes
		 * @param[out] planes Point cloud of segmented planes
		 * @param[out] coeffs_out Coefficients of segmented planes
		 */
		void segmentPlanes(pcl:: PointCloud<PointColor>::Ptr cloud_in,
				std::vector<pcl:: PointCloud<PointColor>::Ptr> &planes,
				std::vector<pcl::ModelCoefficients::Ptr> &coeffs_out);

		/**
		 * @brief Get lines of intersection between planes
		 * @param[in] plane_coeffs_in Coefficients of planes
		 * @param[out] line_coeffs Coefficients of intersecting lines
		 */
		void getIntersectionLines(
				std::vector<pcl::ModelCoefficients::Ptr> plane_coeffs_in, 
				std::vector<pcl::ModelCoefficients::Ptr> &line_coeffs);

		/**
		 * @brief Get points of intersection of lines
		 * @param[in] cloud_in Input point cloud
		 * @param[out] corners Coordinates of intersection points
		 * @param[out] plane_coeffs_out Coefficients of segmented planes
		 * @param[in] epsilon determinant values for parallel planes
		 * if value is < epsilon, intersection is nor calculated
		 */
		void getIntersectionPoints(
				pcl:: PointCloud<PointColor>::Ptr cloud_in,
				std::vector<Eigen::Vector3f> &corners, 
				std::vector<pcl::ModelCoefficients::Ptr> &plane_coeffs_out, 
				double epsilon);

		void computeNormals(pcl:: PointCloud<PointColor>::Ptr cloud_in,
				pcl::PointCloud<PointN>::Ptr normals_out);

		/**
		 * @brief Filter noise from the cloud using MovingLeastSquares
		 * @param mls_points Pointer to point normals of point cloud
		 * @param srcRadius Search radius(in m) to check for anomalies
		 */
		void smoothenSurface(pcl::PointCloud<PointN>::Ptr mls_points, 
				float srcRadius);


		/**
		 * @brief Concatenate a bunch of clouds into one
		 *
		 * NOTE: Only function that doesn't deal with the input cloud.
		 * TODO: Move it to cloud_utils.cpp or something
		 * @param[in] cloud_stack a nice, fat stack of pointclouds.Yum!
		 * @parma[out] cloud_out Concatenated output cloud
		 */
		void combineClouds(std::vector<pcl:: PointCloud<PointColor>::Ptr> cloud_stack, 
				pcl:: PointCloud<PointColor>::Ptr cloud_out);

		/**
		 * @deprecated Use windowdetector instead.
		 */
		void getWindows(pcl:: PointCloud<PointColor>::Ptr plane,
				pcl:: PointCloud<PointColor>::Ptr windows_out,
				PointColor &minXY, PointColor &maxXY);

		/**
		 * @brief Print world coords to stdout
		 */
		void printWorldCoords(int index);

		/**
		 * @brief Ain't nothing but a destructor.
		 */
		~CloudOps();

};
#endif //CLOUD_OPS
