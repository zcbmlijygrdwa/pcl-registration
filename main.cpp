/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier*/

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/keypoints/sift_keypoint.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <ctime>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// This is a tutorial so we can afford having global variables 
//our visualizer
pcl::visualization::PCLVisualizer *p;
//its left and right viewports
int vp_1, vp_2;

//convenient structure to handle our pointclouds
struct PCD
{
	PointCloud::Ptr cloud;
	std::string f_name;

	PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
	bool operator () (const PCD& p1, const PCD& p2)
	{
		return (p1.f_name < p2.f_name);
	}
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
	public:
	MyPointRepresentation ()
	{
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray (const PointNormalT &p, float * out) const
	{
		// < x, y, z, curvature >
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
	p->removePointCloud ("vp1_target");
	p->removePointCloud ("vp1_source");

	PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
	PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
	p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
	p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

	p-> spinOnce();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */

void showCloudsRightAll(const PointCloud::Ptr cloud_target)
{
	p->removePointCloud ("vp2_all");


	//PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_target);
	p->addPointCloud (cloud_target, rgb, "vp2_all", vp_2);

	//p->spin();
	p-> spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
 * \param argc the number of arguments (pass from main ())
 * \param argv the actual command line arguments (pass from main ())
 * \param models the resultant vector of point cloud datasets
 */
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{



	/*
	   std::string extension (".pcd");
	// Suppose the first argument is the actual test model
	for (int i = 1; i < argc; i++)
	{
	std::string fname = std::string (argv[i]);
	// Needs to be at least 5: .plot
	if (fname.size () <= extension.size ())
	continue;

	std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

	//check that the argument is a pcd file
	if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
	{
	// Load the cloud and saves it into the global list of models
	PCD m;
	m.f_name = argv[i];
	pcl::io::loadPCDFile (argv[i], *m.cloud);
	//remove NAN points from the cloud
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

	models.push_back (m);
	}
	}
	 */

	std::string extension (".pcd");
	// Suppose the first argument is the actual test model
	for (int i = 1; i < 1001; i++)
	{

		std::stringstream filename;
		filename <<"capture"<<i<< ".pcd";
		//pcl::io::savePCDFile (ss.str (), *result, true);
		//std::string fname = std::string (argv[i]);

		// Load the cloud and saves it into the global list of models
		PCD m;
		m.f_name = filename.str();
		pcl::io::loadPCDFile (filename.str(), *m.cloud);
		//remove NAN points from the cloud
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

		models.push_back (m);
	}
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
 * \param cloud_src the source PointCloud
 * \param cloud_tgt the target PointCloud
 * \param output the resultant aligned source PointCloud
 * \param final_transform the resultant transform between source and target
 */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{

	time_t tstart, tend; 
	tstart = time(0);


	downsample = true;
	// Downsample for consistency and speed
	// \note enable this for large datasets
	PointCloud::Ptr src (new PointCloud);
	PointCloud::Ptr tgt (new PointCloud);
	pcl::VoxelGrid<PointT> grid;
	if (downsample)
	{
		grid.setLeafSize (0.005, 0.005, 0.005);
		grid.setInputCloud (cloud_src);
		grid.filter (*src);

		grid.setInputCloud (cloud_tgt);
		grid.filter (*tgt);
	}
	else
	{
		src = cloud_src;
		tgt = cloud_tgt;
	}



		//showCloudsRightAll(src);
		//p-> spin();

	/*
	//Normals estimation

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> ne_src;
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> ne_tgt;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_src (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals_tgt (new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_n_src(new pcl::search::KdTree<pcl::PointXYZRGB>());
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_n_tgt(new pcl::search::KdTree<pcl::PointXYZRGB>());

	ne_src.setInputCloud(src);
	ne_tgt.setInputCloud(tgt);
	ne_src.setSearchMethod(tree_n_src);
	ne_tgt.setSearchMethod(tree_n_tgt);
	ne_src.setRadiusSearch(0.05);
	ne_tgt.setRadiusSearch(0.05);
	ne_src.compute(*cloud_normals_src);
	ne_tgt.compute(*cloud_normals_tgt);

	// Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
	for(size_t i = 0; i<cloud_normals_src->points.size(); ++i)
	{
		cloud_normals_src->points[i].x = src->points[i].x;
		cloud_normals_src->points[i].y = src->points[i].y;
		cloud_normals_src->points[i].z = src->points[i].z;
	}

	for(size_t i = 0; i<cloud_normals_tgt->points.size(); ++i)
	{
		cloud_normals_tgt->points[i].x = tgt->points[i].x;
		cloud_normals_tgt->points[i].y = tgt->points[i].y;
		cloud_normals_tgt->points[i].z = tgt->points[i].z;
	}


	//SIFT

	// Parameters for sift computation
	float min_scale = 0.1f;
	int n_octaves = 3;
	int n_scales_per_octave = 2;
	float min_contrast = 0.001f;

	pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift_src;
	pcl::PointCloud<pcl::PointWithScale> result_src;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree_src(new pcl::search::KdTree<pcl::PointNormal> ());
	sift_src.setSearchMethod(tree_src);
	sift_src.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift_src.setMinimumContrast(min_contrast);
	sift_src.setInputCloud(cloud_normals_src);
	sift_src.compute(result_src);


	pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift_tgt;
	pcl::PointCloud<pcl::PointWithScale> result_tgt;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree_tgt(new pcl::search::KdTree<pcl::PointNormal> ());
	sift_tgt.setSearchMethod(tree_tgt);
	sift_tgt.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift_tgt.setMinimumContrast(min_contrast);
	sift_tgt.setInputCloud(cloud_normals_tgt);
	sift_tgt.compute(result_tgt);

	// Copying the pointwithscale to pointxyz
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_src (new pcl::PointCloud<pcl::PointXYZRGB>);
	copyPointCloud(result_src, *keypoints_src);
	std::cout << "keypoints in the src are " << keypoints_src->points.size () << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_tgt (new pcl::PointCloud<pcl::PointXYZRGB>);
	copyPointCloud(result_tgt, *keypoints_tgt);
	std::cout << "keypoints in the tgt are " << keypoints_tgt->points.size () << std::endl;
	
	*/


	//ICP

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(tgt);
	icp.setInputTarget(src);
	//icp.setInputSource(keypoints_tgt);
	//icp.setInputTarget(keypoints_src);
	icp.setMaximumIterations (30);
	pcl::PointCloud<pcl::PointXYZRGB> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

	final_transform = icp.getFinalTransformation();
	std::cout << final_transform << std::endl;


	tend = time(0); 
	std::cout << "It took "<< difftime(tend, tstart) <<" second(s)."<< std::endl;
}


/* ---[ */
int main (int argc, char** argv)
{
	// Load data
	std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
	loadData (argc, argv, data);

	// Check user input
	if (data.empty ())
	{
		PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
		PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
		return (-1);
	}
	PCL_INFO ("Loaded %d datasets.", (int)data.size ());

	// Create a PCLVisualizer object
	p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
	p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

	PointCloud::Ptr result (new PointCloud), tempResult(new PointCloud), source, target;


	PointCloud::Ptr v_filtered (new PointCloud);
	pcl::VoxelGrid<PointT> global_grid;
		
	global_grid.setLeafSize (0.005, 0.005, 0.005);

	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

	for (size_t i = 1; i < data.size (); ++i)
	{
		source = data[i-1].cloud;
		target = data[i].cloud;

		if(i==1)
		{
			*result = *source;
		}

		// Add visualization data
		showCloudsLeft(source, target);

		PointCloud::Ptr temp (new PointCloud);
		PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
		pairAlign (source, target, temp, pairTransform, true);



		//update the global transform
		GlobalTransform = GlobalTransform * pairTransform;

		std::cout<<"Global T:"<<std::endl;
		std::cout<<GlobalTransform.matrix()<<std::endl;

		//transform current pair into the global transform
		pcl::transformPointCloud (*target, *tempResult, GlobalTransform);
		*result += *tempResult;

		global_grid.setInputCloud (result);
		global_grid.filter (*v_filtered);
		*result = *v_filtered;


		// visualize current state
		showCloudsRightAll(result);
		std::cout<<"result size = "<<result->size()<<std::endl;
		////save aligned pair, transformed into the first cloud's frame
		//std::stringstream ss;
		//ss << i << ".pcd";
		//pcl::io::savePCDFile (ss.str (), *result, true);
		p-> spin();

	}
}
/* ]--- */	
