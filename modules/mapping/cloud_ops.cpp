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

#include "cloud_ops.h"

void CloudOps::setInputCloud(
		pcl:: PointCloud<PointColor>::Ptr cloud_in)
{
	pcl::copyPointCloud(*cloud_in, *curr_cloud);
}

void CloudOps::savePrevCloud()
{
	pcl::copyPointCloud(*curr_cloud, *prev_cloud); } 
void CloudOps::filter_cloud(
		pcl:: PointCloud<PointColor>::Ptr cloud_in, 
		pcl:: PointCloud<PointColor>::Ptr cloud_out)
{
	pcl::VoxelGrid<PointColor> grid;
	grid.setLeafSize(0.05f, 0.05f, 0.05f);
	grid.setInputCloud(cloud_in);
	grid.filter(*cloud_out);
}


void CloudOps::apply_transform(float tx, float ty, float tz, float theta)
{
	pcl:: PointCloud<PointColor>::Ptr 
		temp_cloud(new pcl:: PointCloud<PointColor>);
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();

	// Convert to radians
	theta = theta / 180.0f * 3.14159f;
	//Rotation only about x axis
	transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));

	transform.translation() << tx, ty, tz;

	pcl::transformPointCloud(*curr_cloud, *temp_cloud, transform);
	pcl::copyPointCloud(*temp_cloud, *curr_cloud);

}

void CloudOps::incremental_icp(pcl:: PointCloud<PointColor>::Ptr cloud_out)
{

	pcl:: PointCloud<PointColor>::Ptr 
		curr_cloud_filt(new pcl:: PointCloud<PointColor>);
	pcl:: PointCloud<PointColor>::Ptr 
		prev_cloud_filt(new pcl:: PointCloud<PointColor>);

	filter_cloud(curr_cloud, curr_cloud_filt);
	filter_cloud(prev_cloud, prev_cloud_filt);

	pcl::IterativeClosestPoint<PointColor, PointColor> icp;

	icp.setTransformationEpsilon(1e-6);
	icp.setMaxCorrespondenceDistance(0.5);
	icp.setMaximumIterations(50);

	icp.setInputSource(curr_cloud_filt);
	icp.setInputTarget(prev_cloud_filt);
	icp.align(*cloud_out);

	std::cout << "Converged? " << icp.hasConverged() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
}

void CloudOps::segmentPlanes(pcl:: PointCloud<PointColor>::Ptr cloud_in,
		std::vector<pcl:: PointCloud<PointColor>::Ptr> &planes,
		std::vector<pcl::ModelCoefficients::Ptr> &coeffs_out)
{
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl:: PointCloud<PointColor>::Ptr 
		cloud_filt(new pcl:: PointCloud<PointColor>);

	pcl:: PointCloud<PointColor>::Ptr 
		temp_cloud(new pcl:: PointCloud<PointColor>);

	// Retaining original cloud
	pcl::copyPointCloud(*cloud_in, *temp_cloud);

	pcl:: SACSegmentation<PointColor> seg;
	pcl:: ExtractIndices<PointColor> extr;
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
			pcl:: PointCloud<PointColor>::Ptr plane(
					new pcl:: PointCloud<PointColor>);
			// Extract points of a plane from cloud
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

void CloudOps::getIntersectionLines(
		std::vector<pcl::ModelCoefficients::Ptr> plane_coeffs_in, 
		std::vector<pcl::ModelCoefficients::Ptr> &line_coeffs)
{

	double ang_tolerance = 0;
	for(int i = 0; i < (plane_coeffs_in.size() - 1); i++)
	{
		Eigen::Vector4f plane_a; 
		Eigen::Vector4f plane_b;
		Eigen::VectorXf line;
		pcl::ModelCoefficients::Ptr temp_coeff(
				new pcl::ModelCoefficients);

		// Converting from ModelCoefficients to Vector4f
		plane_a.x() = plane_coeffs_in[i]->values[0];
		plane_a.y() = plane_coeffs_in[i]->values[1];
		plane_a.z() = plane_coeffs_in[i]->values[2];
		plane_a.w() = plane_coeffs_in[i]->values[3];

		plane_b.x() = plane_coeffs_in[i+1]->values[0];
		plane_b.y() = plane_coeffs_in[i+1]->values[1];
		plane_b.z() = plane_coeffs_in[i+1]->values[2];
		plane_b.w() = plane_coeffs_in[i+1]->values[3];

		pcl::planeWithPlaneIntersection(plane_a, plane_b, line,
				ang_tolerance);
		for(int j = 0; j < 6; j++)
		{
			temp_coeff->values.push_back(line[j]);
		}

		line_coeffs.push_back(temp_coeff);
	}
}

void CloudOps::getIntersectionPoints(
		pcl:: PointCloud<PointColor>::Ptr cloud_in,
		std::vector<Eigen::Vector3f> &corners, 
		std::vector<pcl::ModelCoefficients::Ptr> &plane_coeffs_out, 
		double epsilon)
{
	Eigen::Vector4f final_mean(.0f, .0f, .0f, .0f);
	Eigen::Vector3f point_out;
	/// Segment planes
	std::vector<pcl:: PointCloud<PointColor>::Ptr> planes_out;
	segmentPlanes(cloud_in, planes_out, plane_coeffs_out);

	int num_planes = plane_coeffs_out.size();
	std::cout << "Planes detected: " << num_planes <<  std::endl;

	// Calculate intersection for every 3 planes
	for(int i = 0; i < num_planes; ++i)
	{
		Eigen::Vector4f plane_a, plane_b, plane_c; 

		// Converting from ModelCoefficients to Vector4f
		plane_a.x() = plane_coeffs_out[i]->values[0];
		plane_a.y() = plane_coeffs_out[i]->values[1];
		plane_a.z() = plane_coeffs_out[i]->values[2];
		plane_a.w() = plane_coeffs_out[i]->values[3];

		for(int j = i + 1; j < num_planes; ++j)
		{
			plane_b.x() = plane_coeffs_out[j]->values[0];
			plane_b.y() = plane_coeffs_out[j]->values[1];
			plane_b.z() = plane_coeffs_out[j]->values[2];
			plane_b.w() = plane_coeffs_out[j]->values[3];

			for(int k = j + 1; k < num_planes; ++k)
			{
				plane_c.x() = plane_coeffs_out[k]->values[0];
				plane_c.y() = plane_coeffs_out[k]->values[1];
				plane_c.z() = plane_coeffs_out[k]->values[2];
				plane_c.w() = plane_coeffs_out[k]->values[3];

				if(pcl::threePlanesIntersection(plane_a,
							plane_b,
							plane_c,
							point_out, epsilon))
				{

					// Add corners to list only if 
					// their x is separated by some distance
					// TODO: Look for a more elegant way
					if(corners.size())
					{
						int ct;
						for(ct = 0; ct < corners.size(); ++ct)
						{ 
							// 8cm
							if(fabs(corners[ct].x() 
										- point_out.x() < 0.08f))
								break;
						}
						if(ct == corners.size())
							corners.push_back(point_out);
					}
					else
						corners.push_back(point_out);
				}
			}
		}
	}
}

void CloudOps::computeNormals(pcl:: PointCloud<PointColor>::Ptr cloud_in,
		pcl::PointCloud<PointN>::Ptr normals_out)
{
	pcl:: NormalEstimation<PointColor, PointN> ne;
	pcl::search:: KdTree<PointColor>::Ptr tree(new pcl::search::KdTree<PointColor>); 
	ne.setInputCloud(cloud_in);
	ne.setSearchMethod(tree);
	ne.setKSearch(50);
	ne.compute(*normals_out);
}

void CloudOps::smoothenSurface(pcl::PointCloud<PointN>::Ptr mls_points, 
		float srcRadius)
{
	pcl::search:: KdTree<PointColor>::Ptr tree(
			new pcl::search:: KdTree<PointColor>);
	pcl::MovingLeastSquares<PointColor, PointN> mls;

	mls.setComputeNormals(true);

	mls.setInputCloud(curr_cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(srcRadius);

	mls.process(*mls_points);
}

void CloudOps::combineClouds(std::vector<pcl:: PointCloud<PointColor>::Ptr> cloud_stack, 
		pcl:: PointCloud<PointColor>::Ptr cloud_out)
{
	//Concatenate clouds
	std::cout << "Size: " << cloud_stack.size() << std::endl;
	for(int i = 0; i < cloud_stack.size(); ++i)
		*cloud_out += *cloud_stack[i];
}

void CloudOps::getWindows(pcl:: PointCloud<PointColor>::Ptr plane,
		pcl:: PointCloud<PointColor>::Ptr windows_out,
		PointColor &minXY, PointColor &maxXY)
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

void CloudOps::printWorldCoords(int index)
{	
	std::cout << "X: " << curr_cloud->points[index].x << std::endl;
	std::cout << "Y: " << curr_cloud->points[index].y << std::endl;
	std::cout << "Z: " << curr_cloud->points[index].z << std::endl;
}
