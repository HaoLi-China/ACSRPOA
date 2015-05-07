#ifndef DETECT_CHANGE_H
#define DETECT_CHANGE_H

#include "../Common/common_type.h"
#include "../Common/common_func.h"
#include "../Common/visualizer.h"
#include "../ObjPreSegment/scene_seg.h"
#include <pcl/octree/octree.h>

//detect change of two point cloud
void detect_change(PointCloudPtr_RGB_NORMAL cloud0, PointCloudPtr_RGB_NORMAL cloud1, PointCloudPtr_RGB_NORMAL result0, PointCloudPtr_RGB_NORMAL result1);

#endif // FILE_IO_H