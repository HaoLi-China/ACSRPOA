#ifndef OBJECT_SEGMENTATION_H
#define OBJECT_SEGMENTATION_H

#include "scene_seg.h"
#include "../Common/common_func.h"
#include "../Common/common_type.h"
#include "../Common/utility.h"
#include "../Common/color_op.h"

#include "../GraphCut/PointCloudAnalysis.h"
#include "../GraphCut/GraphCutBasicStruct.h"
#include "../GraphCut/MeasureConstruction.h"

using namespace std;

void segmentObject(PointCloudPtr_RGB_NORMAL source_cloud, PointCloudPtr_RGB result_cloud);
void overSegmentObject(PointCloudPtr_RGB_NORMAL source_cloud, PointCloudPtr_RGB result_cloud);

#endif // OBJECT_SEGMENTATION_H
