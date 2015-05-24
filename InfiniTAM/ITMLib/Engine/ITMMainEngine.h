// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../ITMLib.h"
#include "../Utils/ITMLibSettings.h"

#include "../../Common/visualizer.h"//hao modified it
#include "../../ObjPreSegment/object_segmentation.h"//hao modified it
#include "../../ObjPreSegment/scene_seg.h"//hao modified it
#include <pcl/io/ply_io.h>//hao modified it
//#include <pcl/io/vtk_io.h>//hao modified it
//#include <pcl/io/vtk_lib_io.h>//hao modified it
#include <vector>//hao modified it
#include "../../Common/KDtree.h"//hao modified it
#include "../../Common/CUDA_KDtree.h"//hao modified it
#include "../../ChangeDetection/detect_change.h"//hao modified it
//#include <pcl/octree/octree.h>//hao modified it
#include <map> //hao modified it

//ccjn modified it
#include "../Utils/MarchingCubeUtils.h"

using namespace std;

/** \mainpage
This is the API reference documentation for InfiniTAM. For a general
overview additional documentation can be found in the included Technical
Report.

For use of ITMLib in your own project, the class
@ref ITMLib::Engine::ITMMainEngine should be the main interface and entry
point to the library.
*/

namespace ITMLib
{
  namespace Engine
  {
    /** \brief
    Main engine, that instantiates all the other engines and
    provides a simplified interface to them.

    This class is the main entry point to the ITMLib library
    and basically performs the whole KinectFusion algorithm.
    It stores the latest image internally, as well as the 3D
    world model and additionally it keeps track of the camera
    pose.

    The intended use is as follows:
    -# Create an ITMMainEngine specifying the internal settings,
    camera parameters and image sizes
    -# Get the pointer to the internally stored images with
    @ref GetView() and write new image information to that
    memory
    -# Call the method @ref ProcessFrame() to track the camera
    and integrate the new information into the world model
    -# Optionally access the rendered reconstruction or another
    image for visualisation using @ref GetImage()
    -# Iterate the above three steps for each image in the
    sequence

    To access the internal information, look at the member
    variables @ref trackingState and @ref scene.
    */
    class ITMMainEngine
    {
    private:
      ITMLibSettings *settings;

      bool hasStartedObjectReconstruction;
      bool fusionActive;

      ITMSceneReconstructionEngine<ITMVoxel,ITMVoxelIndex> *sceneRecoEngine;
      ITMTracker *trackerPrimary, *trackerSecondary;
      ITMLowLevelEngine *lowLevelEngine;
      ITMSwappingEngine<ITMVoxel,ITMVoxelIndex> *swappingEngine;
      ITMVisualisationEngine<ITMVoxel,ITMVoxelIndex> *visualisationEngine;
      ITMVisualisationState *visualisationState;
    public:
      enum GetImageType
      {
        InfiniTAM_IMAGE_ORIGINAL_RGB,
        InfiniTAM_IMAGE_ORIGINAL_DEPTH,
        InfiniTAM_IMAGE_SCENERAYCAST,
        InfiniTAM_IMAGE_SCENERAYCAST_FREECAMERA
      };

      /// Pointer to the current model of the 3D scene
      ITMScene<ITMVoxel,ITMVoxelIndex> *scene;
      /// Pointer for storing the current input frame
      ITMView *view;
      /// Pointer to the current camera pose and additional tracking information
      ITMTrackingState *trackingState;
      ITMTrackingState *trackingStateTem;//hao modified it

      MyPointCloud_RGB_NORMAL myCloudOne;//hao modified it
      MyPointCloud_RGB_NORMAL myCloudTwo;//hao modified it
      ushort idCount;//hao modified it




      MyPointCloud_RGB_NORMAL_ID globalCloud;//hao modified it
      map<ushort, ObjectAttri> objectMap;//hao modified it
      void changeObjectIds(const vector<ushort> &oldIDs, const vector<ushort> &newIds, const vector<uchar> &newRs, const vector<uchar> &newGs, const vector<uchar> &newBs);//hao modified it
      void getGlobalCloud(bool withNormals = true);//hao modified it
      void changeVoxelValue(ITMVoxel *voxels, const ITMHashEntry *hashTable, Point_RGB pt, ushort objectId);//hao modified it
      void updateVoxelsValues(PointCloudPtr_RGB cloud, vector<ushort> objectIds);//hao modified it
      void segmentGlobal();//hao modified it
      void segmentPortionInGlobal(const vector<ushort> &objectIds);//hao modified it
      void getIntsObjectsIds(const ushort targetObjectId, vector<ushort> &objectIds);//hao modified it
      void refineSegment();//hao modified it
      void interactedSegment();//hao modified it
      void preWorkForIntSeg();//hao modified it
      void switchShowModel(uchar mode);//hao modified it



      /// Gives access to the current input frame
      ITMView* GetView() { return view; }

      ///ccjn modified it , for saving triangles 
      vector<myTriangle> g_triangles;

      /// Process the frame accessed with @ref GetView()
      void ProcessFrame(short segFlag);//hao modified it. 0:just fusion 1:over-segmentation 2:segment objects 3:update segment objects

      /// Get a result image as output
      void GetImage(ITMUChar4Image *out, GetImageType getImageType, bool useColour, ITMPose *pose = NULL, ITMIntrinsics *intrinsics = NULL);
      void GetDepthAndColorImageForPause(ITMUChar4Image *out, GetImageType getImageType, bool useColour, ITMPose *pose = NULL, ITMIntrinsics *intrinsics = NULL);//hao modified it

      void SaveAll();

      void savePoints();//hao modified it
      void saveViewPoints();//hao modified it
      void saveViewPoints(ITMTrackingState *trackingState);//hao modified it
      void overSegmentView();//hao modified it
      void segmentView();//hao modified it
      void updateSegmentView();//hao modified it
     
      void detectChange();//hao modified it

      void saveMesh(); //ccjn modified it

      /// switch for turning intergration on/off
      void turnOnIntegration();
      void turnOffIntegration();

      /** \brief Constructor
      Ommitting a separate image size for the depth images
      will assume same resolution as for the RGB images.
      */
      ITMMainEngine(const ITMLibSettings *settings, const ITMRGBDCalib *calib, Vector2i imgSize_rgb, Vector2i imgSize_d = Vector2i(-1,-1));
      ~ITMMainEngine();
    };
  }
}

