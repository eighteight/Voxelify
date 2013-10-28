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
    VGrid(const int x_size, const int y_size, const int z_size, const float leafSize): x_size(x_size), y_size(y_size), z_size(z_size), leafSize(leafSize){
        points = std::vector<std::vector<float> >(x_size * y_size * z_size, std::vector<float>(3,0));
    }
    const float leafSize;
    const int x_size, y_size, z_size;
    
    std::vector<std::vector<float> >points;
    
};

#endif /* defined(__voxelify__VGrid__) */
