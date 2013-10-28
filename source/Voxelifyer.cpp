//
//  Voxelifier.cpp
//  voxelify
//
//  Created by Gusev, Vladimir on 10/22/13.
//
//


#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/make_shared.hpp>
#include <pcl/ros/conversions.h>

#include <pcl/io/vtk_lib_io.h>
#include <iostream>
#include "Voxelifyer.h"

using namespace boost;
using namespace pcl;
using namespace std;

    pcl::PolygonMesh triangles;
    pcl::PolygonMesh mesh;
    //boost::shared_ptr< VoxelGrid<PCLPointCloud2> > voxelGrid = make_shared<VoxelGrid<PCLPointCloud2> >();
    pcl::VoxelGrid<PCLPointCloud2> voxelGrid;
    pcl::PCLPointCloud2::Ptr cloud2 = make_shared<pcl::PCLPointCloud2>();
    pcl::PCLPointCloud2::Ptr cloud_filtered = make_shared<pcl::PCLPointCloud2> ();

void Voxelifier::voxelify(const std::vector<std::vector<float> >& points, VGrid& vGrid, const float leafSize){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    
    cloud->width  = points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize (points.size());
    
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        pcl::PointXYZ& pt = cloud->points[i];
        pt.z = points[i][0];
        pt.x = points[i][1];
        pt.y = points[i][2];
    }
    
    //pcl::PCLPointCloud2::Ptr cloud2 = make_shared<pcl::PCLPointCloud2>();
    pcl::io::loadPolygonFile("plugins/voxelify/bun0.obj",mesh);
    
    //pcl::io::loadPolygonFile("/Users/eight/eliot2/eliot2_0000000.obj", mesh);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
    << " data points (" << pcl::getFieldsList (*cloud) << ").";

    pcl::toPCLPointCloud2 (*cloud,*cloud2);
    // Fill in the cloud data
    pcl::PCDReader reader;


    
    // Create the filtering object
    voxelGrid.setInputCloud (cloud2);
    voxelGrid.setLeafSize (0.01f, 0.01f, 0.01f);
    voxelGrid.filter (*cloud_filtered);
    
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
    << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
    
    std::cerr << "Min box: " << endl<<voxelGrid.getMinBoxCoordinates()
    << endl<<" Max Box " << voxelGrid.getMaxBoxCoordinates() << endl;
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (*cloud_filtered, *out);
    size_t newSize = out->size();
    for (size_t depth_idx = 0; depth_idx < newSize; depth_idx++)
    {
        pcl::PointXYZ& pt = cloud->points[depth_idx];
        vGrid.points[depth_idx][0] = pt.x;
        vGrid.points[depth_idx][1] = pt.y;
        vGrid.points[depth_idx][2] = pt.z;
    }
}

