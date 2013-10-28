//
//  Voxelifier.h
//  voxelify
//
//  Created by Gusev, Vladimir on 10/22/13.
//
//

#ifndef __voxelify__Voxelifier__
#define __voxelify__Voxelifier__


class Voxelifier{
public:
    Voxelifier(){};
    ~Voxelifier(){};
    
    void voxelify(std::vector<std::vector<float> >& points, std::vector<std::vector<float> >& surfacePoints, const float leafSize);

};
#endif /* defined(__voxelify__Voxelifier__) */
