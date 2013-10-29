//
//  Voxelifier.h
//  voxelify
//
//  Created by Gusev, Vladimir on 10/22/13.
//
//

#ifndef __voxelify__Voxelifier__
#define __voxelify__Voxelifier__
#include "VGrid.h"

class Voxelifier{
public:
    Voxelifier(){};
    ~Voxelifier(){};
    
    void voxelify(const std::vector<std::vector<float> >& points, VGrid& vgrid);

};
#endif /* defined(__voxelify__Voxelifier__) */
