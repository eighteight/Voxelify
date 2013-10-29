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

void Voxelifier::voxelify(const std::vector<std::vector<float> >& points, VGrid& vGrid){
    
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
    //pcl::io::loadPolygonFile("plugins/voxelify/bun0.obj",mesh);
    
    //pcl::io::loadPolygonFile("/Users/eight/eliot2/eliot2_0000000.obj", mesh);
    //pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
    << " data points (" << pcl::getFieldsList (*cloud) << ").";

    pcl::toPCLPointCloud2 (*cloud,*cloud2);
    // Fill in the cloud data
    pcl::PCDReader reader;

    // Create the filtering object
    voxelGrid.setInputCloud (cloud2);
    voxelGrid.setLeafSize (vGrid.x_leaf, vGrid.y_leaf, vGrid.z_leaf);
    pcl::IndicesPtr indexVector (new vector<int>); 
    voxelGrid.setIndices(indexVector);
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
    
    ////////////////
    
    
  int data_cnt = 0;
  const Eigen::Vector3i v_ref = voxelGrid.getMinBoxCoordinates ();
  for (int i = 0; i < voxelGrid.getNrDivisions ()[0]; i++) {
    for (int j = 0; j < voxelGrid.getNrDivisions ()[1]; j++) {
      Eigen::Vector3f p (0, 0, 0);
      int cnt_same_z = 0;
//      std::cout << "****************************" << std::endl;
      for (int k = 0; k < voxelGrid.getNrDivisions ()[2]; k++)
      {
        Eigen::Vector3i v (i, j, k);
        v = v + v_ref;
        int index = voxelGrid.getCentroidIndexAt (v);

        if (index != -1)
        {
//          p[0] += pointcloud_data_->points[index].x;
//          p[1] += pointcloud_data_->points[index].y;
//          p[2] += pointcloud_data_->points[index].z;

//          std::cout << "i: " << i << " j: " << j << " k: " << k << std::endl;
//          std::cout << "pt: " << pointcloud_data_->points[index].x << " " << pointcloud_data_->points[index].y << " "
//              << pointcloud_data_->points[index].z << "\n";

          Eigen::Vector3f grid_leaf_size = voxelGrid.getLeafSize ();
          p = p + Eigen::Vector3f (v[0] * grid_leaf_size[0], v[1] * grid_leaf_size[1], v[2] * grid_leaf_size[2]);

//          std::cout << "gr: " << p[0] << " " << p[1] << " " << p[2] << "\n";

          cnt_same_z++;
        }
//        std::cout << v[0] << " " << v[1] << " " << v[2] << "\n";
//        std::cout << grid.getCentroidIndexAt (v) << std::endl;
      }
      if (cnt_same_z)
      {
        p /= cnt_same_z;
        pcl::PointXYZ point (p[0], p[1], p[2]);
//        std::cout << "pt: " << point.x << " " << point.y << " " << point.z << "\n";
        //resampled_data_->points[data_cnt] = point;
        data_cnt++;
      }
//      std::cout << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << std::endl;
    }
  }
}

