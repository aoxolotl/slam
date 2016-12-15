/** 
 * @file cloud_ops.hpp
 * Template class for cloud operations
 */
#ifndef CLOUD_OPS_HPP
#define CLOUD_OPS_HPP

#include <pcl/io/pcd_io.h>

#include <pcl/common/transforms.h>
#include <pcl/common/intersections.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/surface/mls.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <string>
#include <iostream>
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
template<typename PointT>
class CloudOps
{
	private:
		typename pcl::template PointCloud<PointT>::Ptr prev_cloud; //< previous cloud
	public:
		typename pcl::template PointCloud<PointT>::Ptr curr_cloud; //< input cloud

		CloudOps()
			:curr_cloud(new typename pcl::template PointCloud<PointT>)
			, prev_cloud(new typename pcl::template PointCloud<PointT>)
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
		void setInputCloud(typename pcl::template PointCloud<PointT>::Ptr cloud_in)
		{
			pcl::copyPointCloud(*cloud_in, *curr_cloud);
		}

		void savePrevCloud()
		{
			pcl::copyPointCloud(*curr_cloud, *prev_cloud);
		}

		void filter_cloud(typename pcl::template PointCloud<PointT>::Ptr cloud_in, 
			typename pcl::template PointCloud<PointT>::Ptr cloud_out)
		{
			pcl::VoxelGrid<PointT> grid;
			grid.setLeafSize(0.05f, 0.05f, 0.05f);
			grid.setInputCloud(cloud_in);
			grid.filter(*cloud_out);
		}

		/**
		 * @brief Stitch clouds using ICP
		 */
		void incremental_icp(typename pcl::template PointCloud<PointT>::Ptr cloud_out)
		{

			typename pcl::template PointCloud<PointT>::Ptr 
				curr_cloud_filt(new typename pcl::template PointCloud<PointT>);
			typename pcl::template PointCloud<PointT>::Ptr 
				prev_cloud_filt(new typename pcl::template PointCloud<PointT>);
			filter_cloud(curr_cloud, curr_cloud_filt);
			filter_cloud(prev_cloud, prev_cloud_filt);

			/*
			// Normal estimation
`			pcl::PointCloud<PointN>::Ptr curr_filt_nor(
					new pcl::PointCloud<PointN>);
			pcl::PointCloud<PointN>::Ptr prev_filt_nor(
					new pcl::PointCloud<PointN>);

			std::cout << " computing Normals" << std::endl;
			computeNormals(curr_cloud_filt, curr_filt_nor);
			computeNormals(prev_cloud_filt, prev_filt_nor);
			std::cout << "Normals computed" << std::endl;

			typename pcl::template IterativeClosestPointNonLinear<PointN, PointN> icp;

			icp.setTransformationEpsilon(1e-6);
			icp.setMaxCorrespondenceDistance(0.5);

			icp.setMaximumIterations(50);
			icp.setInputSource(curr_filt_nor);
			icp.setInputTarget(prev_filt_nor);

			
			std::cout << "Aligning..." << std::endl;
			icp.align(*cloud_out);
			std::cout << "Aligned..." << std::endl;
			*/

			pcl::IterativeClosestPoint<PointT, PointT> icp;
			
			icp.setTransformationEpsilon(1e-6);
			icp.setMaxCorrespondenceDistance(0.5);
			icp.setMaximumIterations(50);
			
			icp.setInputSource(curr_cloud_filt);
			icp.setInputTarget(prev_cloud_filt);
			icp.align(*cloud_out);
			
			std::cout << "Converged? " << icp.hasConverged() << std::endl;
			std::cout << icp.getFinalTransformation() << std::endl;
		}

		/**
		 * @brief Apply translation and rotation to input cloud
		 */
		void apply_transform(float tx, float ty, float tz, float theta)
		{
			typename pcl::template PointCloud<PointT>::Ptr 
				temp_cloud(new typename pcl::template PointCloud<PointT>);
			Eigen::Affine3f transform = Eigen::Affine3f::Identity();

			// Convert to radians
			theta = theta / 180.0f * 3.14159f;
			//Rotation only about x axis
			transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));

			transform.translation() << tx, ty, tz;

			pcl::transformPointCloud(*curr_cloud, *temp_cloud, transform);
			pcl::copyPointCloud(*temp_cloud, *curr_cloud);

		}

		/**
		 * @brief Segment point cloud into planes
		 * @param[out] planes Point cloud of segmented planes
		 * @param[out] coeffs_out Coefficients of segmented planes
		 */
		void segmentPlanes(typename pcl::template PointCloud<PointT>::Ptr cloud_in,
				std::vector<typename pcl::template PointCloud<PointT>::Ptr> &planes,
				std::vector<pcl::ModelCoefficients::Ptr> &coeffs_out)
		{
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			typename pcl::template PointCloud<PointT>::Ptr 
				cloud_filt(new typename pcl::template PointCloud<PointT>);
			typename pcl::template PointCloud<PointT>::Ptr 
				temp_cloud(new typename pcl::template PointCloud<PointT>);

			// Retaining original cloud
			pcl::copyPointCloud(*cloud_in, *temp_cloud);

			typename pcl::template SACSegmentation<PointT> seg;
			typename pcl::template ExtractIndices<PointT> extr;
			float min_points = 0.2f * temp_cloud->points.size();

			seg.setModelType(pcl::SACMODEL_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(0.05);
			seg.setMaxIterations(500);

			int i = 0;

			while(temp_cloud->points.size() > min_points)
			{
				pcl::ModelCoefficients::Ptr 
					coeffs(new pcl::ModelCoefficients);

				seg.setInputCloud(temp_cloud);
				seg.segment(*inliers, *coeffs);

				coeffs_out.push_back(coeffs);

				if(inliers->indices.size())
				{
					typename pcl::template PointCloud<PointT>::Ptr plane(new typename pcl::template PointCloud<PointT>);
					//Extract points from cloud
					extr.setInputCloud(temp_cloud);
					extr.setIndices(inliers);
					extr.setNegative(false);
					extr.filter(*cloud_filt);

					*plane = *cloud_filt;
					if(plane->points.size() > 50)
						planes.push_back(plane);

					extr.setNegative(true);
					extr.filter(*cloud_filt);
					temp_cloud.swap(cloud_filt);

					++i;
				}
				else
				{
					break;
				}
			}
		}

		/**
		 * @brief Get lines of intersection between planes
		 * @param[in] coeffs_in Coefficients of planes
		 * @param[out] line_coeffs Coefficients of lines
		 */
		void getIntersectionLines(std::vector<pcl::ModelCoefficients::Ptr> coeffs_in, 
				std::vector<pcl::ModelCoefficients::Ptr> &line_coeffs)
		{

			double ang_tolerance = 0;
			for(int i = 0; i < (coeffs_in.size() - 1); i++)
			{
				Eigen::Vector4f plane_a; 
				Eigen::Vector4f plane_b;
				Eigen::VectorXf line;
				pcl::ModelCoefficients::Ptr temp_coeff(new pcl::ModelCoefficients);

				plane_a.x() = coeffs_in[i]->values[0];
				plane_a.y() = coeffs_in[i]->values[1];
				plane_a.z() = coeffs_in[i]->values[2];
				plane_a.w() = coeffs_in[i]->values[3];

				plane_b.x() = coeffs_in[i+1]->values[0];
				plane_b.y() = coeffs_in[i+1]->values[1];
				plane_b.z() = coeffs_in[i+1]->values[2];
				plane_b.w() = coeffs_in[i+1]->values[3];

				pcl::planeWithPlaneIntersection(plane_a, plane_b, line,
						ang_tolerance);
				for(int j = 0; j < 6; j++)
				{
					temp_coeff->values.push_back(line[j]);
				}

				line_coeffs.push_back(temp_coeff);
			}
		}

		/**
		 * @brief Get points of intersection of lines
		 * @param[out] points_out Coordinates of points
		 * @param[in] epsilon tolerance value
		 */
		void getIntersectionPoints(typename pcl::template PointCloud<PointT>::Ptr cloud_in,
				std::vector<Eigen::Vector3f> &corners, 
				std::vector<pcl::ModelCoefficients::Ptr> &plane_coeffs_out, 
				double epsilon)
		{
			Eigen::Vector4f final_mean(.0f, .0f, .0f, .0f);
			Eigen::Vector3f point_out;
			/// Segment planes
			std::vector<typename pcl::template PointCloud<PointT>::Ptr> planes_out;
			segmentPlanes(cloud_in, planes_out, plane_coeffs_out);
			int num_planes = plane_coeffs_out.size();
			std::cout << "planes: " << num_planes <<  std::endl;
			for(int i = 0; i < num_planes; ++i)
			{
				Eigen::Vector4f plane_a, plane_b, plane_c; 

				plane_a.x() = plane_coeffs_out[i % num_planes]->values[0];
				plane_a.y() = plane_coeffs_out[i % num_planes]->values[1];
				plane_a.z() = plane_coeffs_out[i % num_planes]->values[2];
				plane_a.w() = plane_coeffs_out[i % num_planes]->values[3];

				plane_b.x() = plane_coeffs_out[(i+1) % num_planes]->values[0];
				plane_b.y() = plane_coeffs_out[(i+1) % num_planes]->values[1];
				plane_b.z() = plane_coeffs_out[(i+1) % num_planes]->values[2];
				plane_b.w() = plane_coeffs_out[(i+1) % num_planes]->values[3];

				plane_c.x() = plane_coeffs_out[(i+2) % num_planes]->values[0];
				plane_c.y() = plane_coeffs_out[(i+2) % num_planes]->values[1];
				plane_c.z() = plane_coeffs_out[(i+2) % num_planes]->values[2];
				plane_c.w() = plane_coeffs_out[(i+2) % num_planes]->values[3];

				if(pcl::threePlanesIntersection(plane_a,
						plane_b,
						plane_c,
						point_out, 1e-2))
				{
					corners.push_back(point_out);
					std::cout << "planes: " << i << std::endl;
				}
			}
		}

		void computeNormals(typename pcl::template PointCloud<PointT>::Ptr cloud_in,
				pcl::PointCloud<PointN>::Ptr normals_out)
		{
			typename pcl::template NormalEstimation<PointT, PointN> ne;
			typename pcl::search::template KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>); 
			ne.setInputCloud(cloud_in);
			ne.setSearchMethod(tree);
			ne.setKSearch(50);
			ne.compute(*normals_out);
		}

		void estimateBoundaries(pcl::PointCloud<PointN>::Ptr normals_in, 
				pcl::PointCloud<PointB>::Ptr boundaries_out)
		{
			typename pcl::template BoundaryEstimation<PointT, PointN, PointB> est;

			est.setInputCloud(curr_cloud);
			est.setInputNormals(normals_in);
			est.setRadiusSearch(0.01);

			est.setSearchMethod(typename pcl::search::template KdTree<PointT>::Ptr (new typename pcl::search::template KdTree<PointT>));

			est.compute(*boundaries_out);
		}

		void smoothenSurface(pcl::PointCloud<PointN>::Ptr mls_points)
		{
			typename pcl::search::template KdTree<PointT>::Ptr tree(new typename pcl::search::template KdTree<PointT>);
			pcl::MovingLeastSquares<PointT, PointN> mls;

			mls.setComputeNormals(true);

			mls.setInputCloud(curr_cloud);
			mls.setPolynomialFit(true);
			mls.setSearchMethod(tree);
			mls.setSearchRadius(0.03f);

			mls.process(*mls_points);
		}


		/**
		 * @brief Concatenate a bunch of clouds into one
		 *
		 * NOTE: Only function that doesn't deal with the input cloud.
		 * TODO: Move it to cloud_utils.cpp or something
		 * @param[in] cloud_stack a nice, fat stack of pointclouds.Yum!
		 * @parma[out] cloud_out Concatenated output cloud
		 */
		void combineClouds(std::vector<typename pcl::template PointCloud<PointT>::Ptr> cloud_stack, 
				typename pcl::template PointCloud<PointT>::Ptr cloud_out)
		{
			//Concatenate clouds
			std::cout << "Size: " << cloud_stack.size() << std::endl;
			for(int i = 0; i < cloud_stack.size(); ++i)
				*cloud_out += *cloud_stack[i];
		}

		/**
		 * @deprecated Use windowdetector instead.
		 */
		void getWindows(typename pcl::template PointCloud<PointT>::Ptr plane,
				typename pcl::template PointCloud<PointT>::Ptr windows_out,
				PointT &minXY, PointT &maxXY)
		{
			//Check for distance thresh
			double thresh = 0.3;
			minXY.x = minXY.y = 1000.0f;
			maxXY.x = maxXY.y = -1000.0f;

			for(int i = 0; i < plane->size() - 1; i++)
			{
				if((plane->points[i+1].x - plane->points[i].x) > thresh)
				{
					windows_out->push_back(plane->points[i]);
					windows_out->push_back(plane->points[i+1]);
				}
			}

			for(int i = 0; i < windows_out->size(); i++)
			{
				if(windows_out->points[i].x < minXY.x)
					if(windows_out->points[i].y < minXY.y)
						minXY = windows_out->points[i];

				if(windows_out->points[i].x > maxXY.x)
					if(windows_out->points[i].y > maxXY.y)
						maxXY = windows_out->points[i];
			}
			//pcl::getMinMax3D(*windows_out, minXY, maxXY);
			std::cout << "Min X:" << minXY.x << " MinY:" << minXY.y << std::endl;
			std::cout << "Max X:" << maxXY.x << " MaxY:" << maxXY.y << std::endl;
		}

		void fitLines(typename pcl::template PointCloud<PointT>::Ptr windows,
				std::vector<pcl::ModelCoefficients> &window_lines)
		{
			pcl::ModelCoefficients line;
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			typename pcl::template SACSegmentation<PointT> seg;
			int min_size = 0.2 * windows->points.size();
			typename pcl::template ExtractIndices<PointT> extr;

			seg.setOptimizeCoefficients(true);
			seg.setModelType(pcl::SACMODEL_LINE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(0.05);

			while(windows->points.size() > min_size)
			{
				seg.setInputCloud(windows);
				seg.segment(*inliers, line);
				window_lines.push_back(line);
				if(inliers->indices.size())
				{
					typename pcl::template PointCloud<PointT>::Ptr cloud_filt(new pcl::PointCloud<PointT>);
					extr.setInputCloud(windows);
					extr.setIndices(inliers);
					extr.setNegative(true);

					extr.filter(*cloud_filt);
					windows.swap(cloud_filt);
				}
			}
		}

		/**
		 * @brief Print world coords to stdout
		 */
		void printWorldCoords(int index)
		{	
			std::cout << "X: " << curr_cloud->points[index].x << std::endl;
			std::cout << "Y: " << curr_cloud->points[index].y << std::endl;
			std::cout << "Z: " << curr_cloud->points[index].z << std::endl;
		}

		/**
		 * @brief Ain't nothing but a destructor.
		 */
		~CloudOps()
		{
			delete curr_cloud;
		}

};
#endif //CLOUD_OPS
