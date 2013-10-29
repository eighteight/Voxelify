//
//  VGrid.h
//  voxelify
//
//  Created by Gusev, Vladimir on 10/28/13.
//
//

#ifndef __voxelify__VGrid__
#define __voxelify__VGrid__

#include <iostream>

class VGrid{
public:
    VGrid(const int x_size, const int y_size, const int z_size, const float x_leaf, const float y_leaf, const float z_leaf):
    x_size(x_size), y_size(y_size), z_size(z_size), x_leaf(x_leaf), y_leaf(y_leaf), z_leaf(z_leaf){
        size = x_size * y_size * z_size;
        points = std::vector<std::vector<float> >(size, std::vector<float>(3,0));
    }
    VGrid (const VGrid& other){
        x_size = other.x_size;
        y_size = other.y_size;
        z_size = other.z_size;
        x_leaf = other.x_leaf;
        y_leaf = other.y_leaf;
        z_leaf = other.z_leaf;
        points = other.points;
        size = other.size;
    }
    float x_leaf, y_leaf, z_leaf;
    int x_size, y_size, z_size, size;
    
    std::vector<std::vector<float> >points;
    
};

#endif /* defined(__voxelify__VGrid__) */
