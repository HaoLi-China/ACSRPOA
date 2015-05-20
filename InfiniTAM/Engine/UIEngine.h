// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#pragma once

#include "../Utils/SocketClient.h"//hao modified it
#include "../ITMLib/Engine/ITMMainEngine.h"
#include "../ITMLib/Utils/ITMLibSettings.h"
#include "../Utils/FileUtils.h"
#include "../Utils/NVTimer.h"
#include "../Utils/InteractionCompute.h"//hao modified it

#include <vector>//hao modified it

using namespace std;

#include "ImageSourceEngine.h"

namespace InfiniTAM
{
	namespace Engine
	{
		class UIEngine
		{
			static UIEngine* instance;

			ITMLibSettings *internalSettings;
			ImageSourceEngine *imageSource;
			
			StopWatchInterface *timer;

		private: // For UI layout
			static const int NUM_WIN = 3;
			Vector4f winReg[NUM_WIN]; // (x1, y1, x2, y2)
			Vector2i winSize;
			uint textureId[NUM_WIN];
			ITMUChar4Image *outImage[NUM_WIN];
			ITMMainEngine::GetImageType outImageType[NUM_WIN];

			bool freeviewActive;
			bool colourActive;
			bool intergrationActive;
			ITMPose freeviewPose;
			ITMIntrinsics freeviewIntrinsics;
      int show_mode;

			int mouseState;
			Vector2i mouseLastClick;

			int currentFrameNo; bool isRecording;
		public:
			static UIEngine* Instance(void) {
				if (instance == NULL) instance = new UIEngine();
				return instance;
			}

      enum MainLoopAction
      {
        PROCESS_PAUSED, DEPTH_PAUSED, PROCESS_FRAME, PROCESS_VIDEO, PROCESS_EXIT, SAVE_TO_DISK, OVER_SEG_FRAME, SEG_FRAME, UPDATE_SEG_FRAME
      }mainLoopAction;//hao modified it

      ITMMainEngine *mainEngine;//hao modified it

			static void glutDisplayFunction();
			static void glutIdleFunction();
			static void glutKeyUpFunction(unsigned char key, int x, int y);
			static void glutMouseButtonFunction(int button, int state, int x, int y);
			static void glutMouseMoveFunction(int x, int y);
			static void glutMouseWheelFunction(int button, int dir, int x, int y);

			const Vector2i & getWindowSize(void) const
			{ return winSize; }

			float processedTime;
			int processedFrameNo;
			char *outFolder;
			bool needsRefresh;
			ITMUChar4Image *saveImage;

			void Initialise(int & argc, char** argv, ITMLibSettings *internalSettings, ImageSourceEngine *imageSource, ITMMainEngine *mainEngine, const char *outFolder);//hao modified it
			void Shutdown();
      void resetEngine();//hao modified it

			void Run();
			void ProcessFrame(short segFlag);//hao modified it. 0:just fusion 1:over-segmentation 2:segment objects 3:update segment objects

			void GetScreenshot(ITMUChar4Image *dest) const;
			void SaveScreenshot(const char *filename) const;

      void autoReconstruct();//hao modified it
		};
	}
}
