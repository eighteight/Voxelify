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

VGrid Voxelifier::voxelify(const std::vector<std::vector<float> >& points, const float xLeaf, const float yLeaf, const float zLeaf){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    
    cloud->width  = points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize (points.size());
    
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        pcl::PointXYZ& pt = cloud->points[i];
        pt.x = points[i][0];
        pt.y = points[i][1];
        pt.z = points[i][2];
    }
    
    //pcl::PCLPointCloud2::Ptr cloud2 = make_shared<pcl::PCLPointCloud2>();
    //pcl::io::loadPolygonFile("plugins/voxelify/bun0.obj",mesh);
    
    //pcl::io::loadPolygonFile("/Users/eight/eliot2/eliot2_0000000.obj", mesh);
    //pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    pcl::toPCLPointCloud2 (*cloud,*cloud2);
    // Fill in the cloud data
    pcl::PCDReader reader;

    // Create the filtering object
    voxelGrid.setInputCloud (cloud2);
    voxelGrid.setLeafSize (xLeaf, yLeaf, zLeaf);

    voxelGrid.setSaveLeafLayout(true);
    voxelGrid.filter (*cloud_filtered);
    
    cerr << cloud->width * cloud->height <<"->"<< cloud_filtered->width * cloud_filtered->height<<endl;
    
//    std::cerr << "Min box: " << endl<<voxelGrid.getMinBoxCoordinates()
//    << endl<<" Max Box " << voxelGrid.getMaxBoxCoordinates() << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr out = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    pcl::fromPCLPointCloud2 (*cloud_filtered, *out);
    
    VGrid vGrid;
    vGrid.x_leaf = xLeaf;
    vGrid.y_leaf = yLeaf;
    vGrid.z_leaf = zLeaf;
    
  int data_cnt = 0;
  const Eigen::Vector3i v_ref = voxelGrid.getMinBoxCoordinates ();
  int nr0 = voxelGrid.getNrDivisions ()[0], nr1 = voxelGrid.getNrDivisions ()[1], nr2 = voxelGrid.getNrDivisions ()[2];
  vGrid.points = std::vector<vector<float> >(nr0*nr1*nr2, vector<float>(3,0.0));
  vGrid.indices = std::vector<int>(nr0*nr1*nr2, -1);
  for (int i = 0; i < nr0; i++) {
    for (int j = 0; j < nr1; j++) {
      Eigen::Vector3f p (0, 0, 0);

      for (int k = 0; k < nr2; k++) {
        Eigen::Vector3i v (i, j, k);
        v = v + v_ref;
        int index = voxelGrid.getCentroidIndexAt (v);

        if (index != -1){
            vGrid.indices[data_cnt] = 1;

            vGrid.points[data_cnt][0] = out->points[index].x;
            vGrid.points[data_cnt][1] = out->points[index].y;
            vGrid.points[data_cnt][2] = out->points[index].z;

          Eigen::Vector3f grid_leaf_size = voxelGrid.getLeafSize ();
          p = p + Eigen::Vector3f (v[0] * grid_leaf_size[0], v[1] * grid_leaf_size[1], v[2] * grid_leaf_size[2]);

        }
        data_cnt++;
      }
    }
  }
    
    std::cout<<vGrid<<std::endl;
    return vGrid;
}

