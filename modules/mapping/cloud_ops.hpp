#ifndef CLOUD_OPS_HPP
#define CLOUD_OPS_HPP
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/intersections.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <string>
#include <iostream>
#include "pcl_typedefs.h"

#define SQ(a) (a * a)

template<typename PointT>
class CloudOps
{
	private:
		typename pcl::template PointCloud<PointT>::Ptr curr_cloud;

	public:
		CloudOps()
			:curr_cloud(new typename pcl::template PointCloud<PointT>)
		{
		}

		void setInputCloud(typename pcl::template PointCloud<PointT>::Ptr cloud_in)
		{
			pcl::copyPointCloud(*cloud_in, *curr_cloud);
		}

		void apply_transform(typename pcl::template PointCloud<PointT>::Ptr cloud_out, 
				float tx, float ty, float tz, float theta)
		{
			Eigen::Affine3f transform = Eigen::Affine3f::Identity();
			// Convert to radians
			theta = theta / 180.0f * 3.14159f;
			//Rotation only about x axis
			transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));

			transform.translation() << tx, ty, tz;

			pcl::transformPointCloud(*curr_cloud, *cloud_out, transform);
		}

		/**
		 * @brief Segment point cloud into planes
		 * @param[in] cloud_in Input point cloud
		 * @param[out] planes Point cloud of segmented planes
		 * @param[out] coeffs_out Coefficients of segmented planes
		 */
		void segmentPlanes(std::vector<typename pcl::template PointCloud<PointT>::Ptr> &planes,
				std::vector<pcl::ModelCoefficients::Ptr> &coeffs_out)
		{
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			typename pcl::template PointCloud<PointT>::Ptr cloud_filt(new typename pcl::template PointCloud<PointT>);
			typename pcl::template PointCloud<PointT>::Ptr temp_cloud(new typename pcl::template PointCloud<PointT>);
			pcl::copyPointCloud(*curr_cloud, *temp_cloud);
			typename pcl::template SACSegmentation<PointT> seg;
			typename pcl::template ExtractIndices<PointT> extr;
			float min_points = 0.2f * temp_cloud->points.size();

			seg.setModelType(pcl::SACMODEL_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(0.02);
			seg.setMaxIterations(500);

			int i = 0;

			while(temp_cloud->points.size() > min_points)
			{
				pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
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
		 * @param[out] coeffs_in Coefficients of lines
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
		 * @param[in] coeffs_in Coefficients of lines
		 * @param[out] points_out Coordinates of points
		 * @param[in] epsilon tolerance value
		 */
		void getIntersectionPoints(std::vector<pcl::ModelCoefficients::Ptr> coeffs_in,
				std::vector<Eigen::Vector4f> &points_out, double epsilon)
		{
			Eigen::Vector4f temp_point;
			for(int i = 0; i < (coeffs_in.size() - 1); i++)
			{
				if(pcl::lineWithLineIntersection(*coeffs_in[i], *coeffs_in[i+1], temp_point, epsilon))
				{
					points_out.push_back(temp_point);
				}
			}
		}

		void computeNormals(pcl::PointCloud<PointN>::Ptr normals_out)
		{
			typename pcl::template NormalEstimation<PointT, PointN> ne;
			typename pcl::search::template KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>); 
			ne.setInputCloudcurr_cloud;
			ne.setSearchMethod(tree);
			ne.setRadiusSearch(0.008);
			ne.compute(*normals_out);
		}

		void estimateBoundaries(pcl::PointCloud<PointN>::Ptr normals_in, 
				pcl::PointCloud<PointB>::Ptr boundaries_out)
		{
			typename pcl::template BoundaryEstimation<PointT, PointN, PointB> est;

			est.setInputCloudcurr_cloud;
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

			mls.setInputCloudcurr_cloud;
			mls.setPolynomialFit(true);
			mls.setSearchMethod(tree);
			mls.setSearchRadius(0.03f);

			mls.process(*mls_points);
		}


		void combineClouds(std::vector<typename pcl::template PointCloud<PointT>::Ptr> cloud_stack, 
				typename pcl::template PointCloud<PointT>::Ptr cloud_out)
		{
			pcl::PointCloud<PointN>::Ptr mls_points(new pcl::PointCloud<PointN>);

			//Concatenate clouds
			for(int i = 0; i < cloud_stack.size(); ++i)
				*cloud_out += *cloud_stack[i];
		}

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

};
#endif //CLOUD_OPS
