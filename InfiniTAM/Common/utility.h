#ifndef UTILITY_H
#define UTILITY_H

#include "common_type.h"
#include "visualizer.h"

const float g_color_table[256][3] = {
  0, 0, 0.5156
  , 0, 0, 0.5313
  , 0, 0, 0.5469
  , 0, 0, 0.5625
  , 0, 0, 0.5781
  , 0, 0, 0.5938
  , 0, 0, 0.6094
  , 0, 0, 0.6250
  , 0, 0, 0.6406
  , 0, 0, 0.6563
  , 0, 0, 0.6719
  , 0, 0, 0.6875
  , 0, 0, 0.7031
  , 0, 0, 0.7188
  , 0, 0, 0.7344
  , 0, 0, 0.7500
  , 0, 0, 0.76564
  , 0, 0, 0.7813
  , 0, 0, 0.7969
  , 0, 0, 0.8125
  , 0, 0, 0.8281
  , 0, 0, 0.8438
  , 0, 0, 0.8594
  , 0, 0, 0.8750
  , 0, 0, 0.8906
  , 0, 0, 0.9063
  , 0, 0, 0.9219
  , 0, 0, 0.9375
  , 0, 0, 0.9531
  , 0, 0, 0.9688
  , 0, 0, 0.9844
  , 0, 0, 1.0000
  , 0, 0.0156, 1.0000
  , 0, 0.0313, 1.0000
  , 0, 0.0469, 1.0000
  , 0, 0.0625, 1.0000
  , 0, 0.0781, 1.0000
  , 0, 0.0938, 1.0000
  , 0, 0.1094, 1.0000
  , 0, 0.1250, 1.0000
  , 0, 0.1406, 1.0000
  , 0, 0.1563, 1.0000
  , 0, 0.1719, 1.0000
  , 0, 0.1875, 1.0000
  , 0, 0.2031, 1.0000
  , 0, 0.2188, 1.0000
  , 0, 0.2344, 1.0000
  , 0, 0.2500, 1.0000
  , 0, 0.2656, 1.0000
  , 0, 0.2813, 1.0000
  , 0, 0.2969, 1.0000
  , 0, 0.3125, 1.0000
  , 0, 0.3281, 1.0000
  , 0, 0.3438, 1.0000
  , 0, 0.3594, 1.0000
  , 0, 0.3750, 1.0000
  , 0, 0.3906, 1.0000
  , 0, 0.4063, 1.0000
  , 0, 0.4219, 1.0000
  , 0, 0.4375, 1.0000
  , 0, 0.4531, 1.0000
  , 0, 0.4688, 1.0000
  , 0, 0.4844, 1.0000
  , 0, 0.5000, 1.0000
  , 0, 0.5156, 1.0000
  , 0, 0.5313, 1.0000
  , 0, 0.5469, 1.0000
  , 0, 0.5625, 1.0000
  , 0, 0.5781, 1.0000
  , 0, 0.5938, 1.0000
  , 0, 0.6094, 1.0000
  , 0, 0.6250, 1.0000
  , 0, 0.6406, 1.0000
  , 0, 0.6563, 1.0000
  , 0, 0.6719, 1.0000
  , 0, 0.6875, 1.0000
  , 0, 0.7031, 1.0000
  , 0, 0.7188, 1.0000
  , 0, 0.7344, 1.0000
  , 0, 0.7500, 1.0000
  , 0, 0.7656, 1.0000
  , 0, 0.7813, 1.0000
  , 0, 0.7969, 1.0000
  , 0, 0.8125, 1.0000
  , 0, 0.8281, 1.0000
  , 0, 0.8438, 1.0000
  , 0, 0.8594, 1.0000
  , 0, 0.8750, 1.0000
  , 0, 0.8906, 1.0000
  , 0, 0.9063, 1.0000
  , 0, 0.9219, 1.0000
  , 0, 0.9375, 1.0000
  , 0, 0.9531, 1.0000
  , 0, 0.9688, 1.0000
  , 0, 0.9844, 1.0000
  , 0, 1.0000, 1.0000
  , 0.0156, 1.0000, 0.9844
  , 0.0313, 1.0000, 0.9688
  , 0.0469, 1.0000, 0.9531
  , 0.0625, 1.0000, 0.9375
  , 0.0781, 1.0000, 0.9219
  , 0.0938, 1.0000, 0.9063
  , 0.1094, 1.0000, 0.8906
  , 0.1250, 1.0000, 0.8750
  , 0.1406, 1.0000, 0.8594
  , 0.1563, 1.0000, 0.8438
  , 0.1719, 1.0000, 0.8281
  , 0.1875, 1.0000, 0.8125
  , 0.2031, 1.0000, 0.7969
  , 0.2188, 1.0000, 0.7813
  , 0.2344, 1.0000, 0.7656
  , 0.2500, 1.0000, 0.7500
  , 0.2656, 1.0000, 0.7344
  , 0.2813, 1.0000, 0.7188
  , 0.2969, 1.0000, 0.7031
  , 0.3125, 1.0000, 0.6875
  , 0.3281, 1.0000, 0.6719
  , 0.3438, 1.0000, 0.6563
  , 0.3594, 1.0000, 0.6406
  , 0.3750, 1.0000, 0.6250
  , 0.3906, 1.0000, 0.6094
  , 0.4063, 1.0000, 0.5938
  , 0.4219, 1.0000, 0.5781
  , 0.4375, 1.0000, 0.5625
  , 0.4531, 1.0000, 0.5469
  , 0.4688, 1.0000, 0.5313
  , 0.4844, 1.0000, 0.5156
  , 0.5000, 1.0000, 0.5000
  , 0.5156, 1.0000, 0.4844
  , 0.5313, 1.0000, 0.4688
  , 0.5469, 1.0000, 0.4531
  , 0.5625, 1.0000, 0.4375
  , 0.5781, 1.0000, 0.4219
  , 0.5938, 1.0000, 0.4063
  , 0.6094, 1.0000, 0.3906
  , 0.6250, 1.0000, 0.3750
  , 0.6406, 1.0000, 0.3594
  , 0.6563, 1.0000, 0.3438
  , 0.6719, 1.0000, 0.3281
  , 0.6875, 1.0000, 0.3125
  , 0.7031, 1.0000, 0.2969
  , 0.7188, 1.0000, 0.2813
  , 0.7344, 1.0000, 0.2656
  , 0.7500, 1.0000, 0.2500
  , 0.7656, 1.0000, 0.2344
  , 0.7813, 1.0000, 0.2188
  , 0.7969, 1.0000, 0.2031
  , 0.8125, 1.0000, 0.1875
  , 0.8281, 1.0000, 0.1719
  , 0.8438, 1.0000, 0.1563
  , 0.8594, 1.0000, 0.1406
  , 0.8750, 1.0000, 0.1250
  , 0.8906, 1.0000, 0.1094
  , 0.9063, 1.0000, 0.0938
  , 0.9219, 1.0000, 0.0781
  , 0.9375, 1.0000, 0.0625
  , 0.9531, 1.0000, 0.0469
  , 0.9688, 1.0000, 0.0313
  , 0.9844, 1.0000, 0.0156
  , 1.0000, 1.0000, 0
  , 1.0000, 0.9844, 0
  , 1.0000, 0.9688, 0
  , 1.0000, 0.9531, 0
  , 1.0000, 0.9375, 0
  , 1.0000, 0.9219, 0
  , 1.0000, 0.9063, 0
  , 1.0000, 0.8906, 0
  , 1.0000, 0.8750, 0
  , 1.0000, 0.8594, 0
  , 1.0000, 0.8438, 0
  , 1.0000, 0.8281, 0
  , 1.0000, 0.8125, 0
  , 1.0000, 0.7969, 0
  , 1.0000, 0.7813, 0
  , 1.0000, 0.7656, 0
  , 1.0000, 0.7500, 0
  , 1.0000, 0.7344, 0
  , 1.0000, 0.7188, 0
  , 1.0000, 0.7031, 0
  , 1.0000, 0.6875, 0
  , 1.0000, 0.6719, 0
  , 1.0000, 0.6563, 0
  , 1.0000, 0.6406, 0
  , 1.0000, 0.6250, 0
  , 1.0000, 0.6094, 0
  , 1.0000, 0.5938, 0
  , 1.0000, 0.5781, 0
  , 1.0000, 0.5625, 0
  , 1.0000, 0.5469, 0
  , 1.0000, 0.5313, 0
  , 1.0000, 0.5156, 0
  , 1.0000, 0.5000, 0
  , 1.0000, 0.4844, 0
  , 1.0000, 0.4688, 0
  , 1.0000, 0.4531, 0
  , 1.0000, 0.4375, 0
  , 1.0000, 0.4219, 0
  , 1.0000, 0.4063, 0
  , 1.0000, 0.3906, 0
  , 1.0000, 0.3750, 0
  , 1.0000, 0.3594, 0
  , 1.0000, 0.3438, 0
  , 1.0000, 0.3281, 0
  , 1.0000, 0.3125, 0
  , 1.0000, 0.2969, 0
  , 1.0000, 0.2813, 0
  , 1.0000, 0.2656, 0
  , 1.0000, 0.2500, 0
  , 1.0000, 0.2344, 0
  , 1.0000, 0.2188, 0
  , 1.0000, 0.2031, 0
  , 1.0000, 0.1875, 0
  , 1.0000, 0.1719, 0
  , 1.0000, 0.1563, 0
  , 1.0000, 0.1406, 0
  , 1.0000, 0.1250, 0
  , 1.0000, 0.1094, 0
  , 1.0000, 0.0938, 0
  , 1.0000, 0.0781, 0
  , 1.0000, 0.0625, 0
  , 1.0000, 0.0469, 0
  , 1.0000, 0.0313, 0
  , 1.0000, 0.0156, 0
  , 1.0000, 0, 0
  , 0.9844, 0, 0
  , 0.9688, 0, 0
  , 0.9531, 0, 0
  , 0.9375, 0, 0
  , 0.9219, 0, 0
  , 0.9063, 0, 0
  , 0.8906, 0, 0
  , 0.8750, 0, 0
  , 0.8594, 0, 0
  , 0.8438, 0, 0
  , 0.8281, 0, 0
  , 0.8125, 0, 0
  , 0.7969, 0, 0
  , 0.7813, 0, 0
  , 0.7656, 0, 0
  , 0.7500, 0, 0
  , 0.7344, 0, 0
  , 0.7188, 0, 0
  , 0.7031, 0, 0
  , 0.6875, 0, 0
  , 0.6719, 0, 0
  , 0.6563, 0, 0
  , 0.6406, 0, 0
  , 0.6250, 0, 0
  , 0.6094, 0, 0
  , 0.5938, 0, 0
  , 0.5781, 0, 0
  , 0.5625, 0, 0
  , 0.5469, 0, 0
  , 0.5313, 0, 0
  , 0.5156, 0, 0
  , 0.5000, 0, 0
};

const float new_color_table[30][3] = {
  0,     0,     255.0
  ,0,     255.0, 0
  ,255.0, 0, 0
  ,0,     255.0, 255.0
  ,255.0, 255.0, 0
  ,255.0,     0, 255.0

  ,0,     0,     204.0
  ,0,     204.0, 0
  ,204.0, 0, 0
  ,0,     204.0, 204.0
  ,204.0, 204.0, 0
  ,204.0,     0, 204.0

  ,0,     0,     153.0
  ,0,     153.0, 0
  ,153.0, 0, 0
  ,0,     153.0, 153.0
  ,153.0, 153.0, 0
  ,153.0,     0, 153.0

  ,0,     0,      102.0
  ,0,     102.0, 0
  ,102.0, 0, 0
  ,0,     102.0, 102.0
  ,102.0, 102.0, 0
  ,102.0,     0, 102.0

  ,0,     0,      51.0
  ,0,     51.0, 0
  ,51.0, 0, 0
  ,0,     51.0, 51.0
  ,51.0, 51.0, 0
  ,51.0,     0, 51.0

};

const float color_table2[6][3] = {
  255.0,     0,       0
 ,0,     255.0,       0
 ,0,         0,     255.0
 ,0,     255.0,     255.0
 ,255.0, 255.0,       0
 ,255.0,     0,     255.0
};

void draw_box(PointCloudPtr box_cloud, Visualizer& vs, float r, float g, float b, const char* id);
void draw_rect(PointCloudPtr rect_cloud, Visualizer& vs, float r, float g, float b, const char* id);

#endif // UTILITY_H