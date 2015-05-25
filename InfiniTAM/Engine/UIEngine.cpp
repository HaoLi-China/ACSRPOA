// Copyright 2014 Isis Innovation Limited and the authors of InfiniTAM

#include "UIEngine.h"

#include <string.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#ifdef FREEGLUT
#include <GL/freeglut.h>
#else
#if (!defined USING_CMAKE) && (defined _MSC_VER)
#pragma comment(lib, "glut64")
#endif
#endif

#include "../Utils/FileUtils.h"

using namespace InfiniTAM::Engine;
UIEngine* UIEngine::instance;

static void safe_glutBitmapString(void *font, const char *str)
{
  size_t len = strlen(str);
  for (size_t x = 0; x < len; ++x) {
    glutBitmapCharacter(font, str[x]);
  }
}

//hao modified it
void UIEngine::glutDisplayFunction()
{
  UIEngine *uiEngine = UIEngine::Instance();

  if(uiEngine->mainLoopAction == DEPTH_PAUSED){
    for (int w = 0; w < NUM_WIN; w++) uiEngine->mainEngine->GetDepthAndColorImageForPause(uiEngine->outImage[w], uiEngine->outImageType[w], false);
  }
  else{
    // get updated images from processing thread
    if (uiEngine->freeviewActive) 
      uiEngine->mainEngine->GetImage(uiEngine->outImage[0], uiEngine->outImageType[0], uiEngine->colourActive, &uiEngine->freeviewPose, &uiEngine->freeviewIntrinsics);
    else uiEngine->mainEngine->GetImage(uiEngine->outImage[0], uiEngine->outImageType[0], false);

    for (int w = 1; w < NUM_WIN; w++) uiEngine->mainEngine->GetImage(uiEngine->outImage[w], uiEngine->outImageType[w], false);
  }

  // do the actual drawing
  glClear(GL_COLOR_BUFFER_BIT);
  glColor3f(1.0f, 1.0f, 1.0f);
  glEnable(GL_TEXTURE_2D);

  ITMUChar4Image** showImgs = uiEngine->outImage;
  Vector4f *winReg = uiEngine->winReg;
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  {
    glLoadIdentity();
    glOrtho(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    {
      glEnable(GL_TEXTURE_2D);
      for (int w = 0; w < NUM_WIN; w++)	{// Draw each sub window
        glBindTexture(GL_TEXTURE_2D, uiEngine->textureId[w]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, showImgs[w]->noDims.x, showImgs[w]->noDims.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, showImgs[w]->GetData(false));
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glBegin(GL_QUADS); {
          glTexCoord2f(0, 1); glVertex2f(winReg[w][0], winReg[w][1]); // glVertex2f(0, 0);
          glTexCoord2f(1, 1); glVertex2f(winReg[w][2], winReg[w][1]); // glVertex2f(1, 0);
          glTexCoord2f(1, 0); glVertex2f(winReg[w][2], winReg[w][3]); // glVertex2f(1, 1);
          glTexCoord2f(0, 0); glVertex2f(winReg[w][0], winReg[w][3]); // glVertex2f(0, 1);
        }
        glEnd();
      }
      glDisable(GL_TEXTURE_2D);
    }
    glPopMatrix();
  }
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();

  //glColor3f(1.0f, 0.0f, 0.0f); glRasterPos2f(0.85f, -0.962f);

  //char str[200]; sprintf(str, "%04.2lf", uiEngine->processedTime);
  //safe_glutBitmapString(GLUT_BITMAP_HELVETICA_18, (const char*)str);
  ////hao modified it
  //{
  //  glRasterPos2f(-0.95f, -0.93f);
  //  sprintf(str, "r - reset \t n - next frame \t o - over-segmentation \t p - segmentation \t b - all frames \t u - update segmentation \t v - save points in current frame");
  //  safe_glutBitmapString(GLUT_BITMAP_HELVETICA_12, (const char*)str);

  //  glRasterPos2f(-0.95f, -0.98f);
  //  if (ITMVoxel::hasColorInformation)
  //  {//hao modified it
  //    sprintf(str, "d - detect change \t a - auto reconstruct \t m - save mesh \t e - exit \t f - %s \t c - %s", uiEngine->freeviewActive?"follow camera":"free viewpoint", uiEngine->colourActive?"stop using colour":"use colour");
  //  }
  //  else
  //  {//hao modified it
  //    sprintf(str, "d - detect change \t a - auto reconstruct \t m - save mesh \t e/esc - exit \t f - %s \t t - turn fusion %s", uiEngine->freeviewActive ? "follow camera" : "free viewpoint", uiEngine->intergrationActive ? "off" : "on");
  //  }
  //  safe_glutBitmapString(GLUT_BITMAP_HELVETICA_12, (const char*)str);
  //}

  glutSwapBuffers();
  uiEngine->needsRefresh = false;
}

//hao modified it
void UIEngine::glutIdleFunction()
{
  UIEngine *uiEngine = UIEngine::Instance();

  switch (uiEngine->mainLoopAction)
  {
  case PROCESS_FRAME:
    uiEngine->ProcessFrame(0); uiEngine->processedFrameNo++;
    uiEngine->mainLoopAction = PROCESS_PAUSED;
    uiEngine->needsRefresh = true;
    break;
  case OVER_SEG_FRAME:
    uiEngine->ProcessFrame(1); uiEngine->processedFrameNo++;
    uiEngine->mainLoopAction = DEPTH_PAUSED;
    uiEngine->needsRefresh = true;
    break;
  case SEG_FRAME:
    uiEngine->ProcessFrame(2); uiEngine->processedFrameNo++;
    uiEngine->mainLoopAction = DEPTH_PAUSED;
    uiEngine->needsRefresh = true;
    break;
  case INTERACTED_SEG:
    uiEngine->ProcessFrame(3); uiEngine->processedFrameNo++;
    uiEngine->mainLoopAction = PROCESS_PAUSED;
    uiEngine->needsRefresh = true;
    break;
  case GLOBAL_SEG:
    uiEngine->ProcessFrame(4); uiEngine->processedFrameNo++;
    uiEngine->mainLoopAction = PROCESS_PAUSED;
    uiEngine->needsRefresh = true;
    break;
  case REFINED_SEG:
    uiEngine->ProcessFrame(5); uiEngine->processedFrameNo++;
    uiEngine->mainLoopAction = PROCESS_PAUSED;
    uiEngine->needsRefresh = true;
    break;
  case PROCESS_VIDEO:
    uiEngine->ProcessFrame(0); uiEngine->processedFrameNo++;
    uiEngine->needsRefresh = true;
    break;
    //case SAVE_TO_DISK:
    //	if (!uiEngine->actionDone)
    //	{
    //		char outFile[255];

    //		ITMUChar4Image *saveImage = uiEngine->saveImage;

    //		glReadBuffer(GL_BACK);
    //		glReadPixels(0, 0, saveImage->noDims.x, saveImage->noDims.x, GL_RGBA, GL_UNSIGNED_BYTE, (unsigned char*)saveImage->GetData(false));
    //		sprintf(outFile, "%s/out_%05d.ppm", uiEngine->outFolder, uiEngine->processedFrameNo);

    //		SaveImageToFile(saveImage, outFile, true);

    //		uiEngine->actionDone = true;
    //	}
    //	break;
  case PROCESS_EXIT:
#ifdef FREEGLUT
    glutLeaveMainLoop();
#else
    exit(0);
#endif
    break;
  case PROCESS_PAUSED:
    break;
  case DEPTH_PAUSED:
    {
      if (!uiEngine->imageSource->hasMoreImages()) break;
      uiEngine->imageSource->getImages(uiEngine->mainEngine->view);  
      uiEngine->needsRefresh = true;
      break;
    }
  default:
    break;
  }

  if (uiEngine->needsRefresh) {
    glutPostRedisplay();
  }
}

//hao modified it
void UIEngine::glutKeyUpFunction(unsigned char key, int x, int y)
{
  UIEngine *uiEngine = UIEngine::Instance();

  switch (key)
  {
  case 'a':
    printf("auto reconstruct ...\n");
    UIEngine::Instance()->autoReconstruct();
    break;
  case 'b':
    printf("processing input source ...\n");
    uiEngine->mainLoopAction = UIEngine::PROCESS_VIDEO;
    break;
  case 'c':
    uiEngine->colourActive = !uiEngine->colourActive;
    uiEngine->needsRefresh = true;
    break;
  case 'd':
    printf("detect change ...\n");
    uiEngine->mainEngine->detectChange();
    break;
  case 'e':
    if (uiEngine->show_mode == 0)
    {
      uiEngine->outImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
      uiEngine->outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
      uiEngine->outImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;

      uiEngine->show_mode = 1;
    }
    else if (uiEngine->show_mode == 1)
    {
      uiEngine->outImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;
      uiEngine->outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
      uiEngine->outImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;

      uiEngine->show_mode = 2;
    }
    else if (uiEngine->show_mode == 2)
    {
      uiEngine->outImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
      uiEngine->outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
      uiEngine->outImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;

      uiEngine->show_mode = 0;
    }
    uiEngine->needsRefresh = true;
    break;
  case 'f':
    if (uiEngine->freeviewActive)
    {
      uiEngine->outImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
      uiEngine->outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;

      uiEngine->freeviewActive = false;
    }
    else
    {
      uiEngine->outImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST_FREECAMERA;
      uiEngine->outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;

      uiEngine->freeviewPose.SetFrom(uiEngine->mainEngine->trackingState->pose_d);
      uiEngine->freeviewIntrinsics = uiEngine->mainEngine->GetView()->calib->intrinsics_d;
      uiEngine->outImage[0]->ChangeDims(uiEngine->mainEngine->GetView()->depth->noDims);
      uiEngine->freeviewActive = true;
    }
    uiEngine->needsRefresh = true;
    break;
  case 'g':
    printf("segment global points ...\n");
    uiEngine->mainLoopAction = UIEngine::GLOBAL_SEG;
    break;
  case 'h':
    printf("DEPTH_PAUSED ...\n");
    uiEngine->mainLoopAction = UIEngine::DEPTH_PAUSED;
    uiEngine->needsRefresh = true;
    break;
  case 'i':
    printf("interacted segment ...\n");
    uiEngine->mainLoopAction = UIEngine::INTERACTED_SEG;
    break;
  case 'j':
    printf("preWorkForIntSeg start ...\n");
    uiEngine->mainLoopAction = UIEngine::DEPTH_PAUSED;
    uiEngine->mainEngine->preWorkForIntSeg();
    break;
  case 'k':
    printf("segment frame ...\n");
    uiEngine->mainLoopAction = UIEngine::SEG_FRAME;
    break;
  case 'm'://ccjn modified it
    printf("save mesh ...\n");
    uiEngine->mainLoopAction = UIEngine::PROCESS_PAUSED;
    uiEngine->mainEngine->saveMesh();
    break;
  case 'n':
    printf("processing one frame ...\n");
    uiEngine->mainLoopAction = UIEngine::PROCESS_FRAME;
    break;
  case 'o':
    printf("over segment objects ...\n");
    uiEngine->mainLoopAction = UIEngine::OVER_SEG_FRAME;
    break;
  case 'p':
    printf("PROCESS_PAUSED ...\n");
    uiEngine->mainLoopAction = UIEngine::PROCESS_PAUSED;
    uiEngine->needsRefresh = true;
    break;
  case 'r':
    printf("reset ...\n");
    UIEngine::Instance()->resetEngine();
    break;
  case 's':
    {
      printf("save points ...\n");
      uiEngine->mainLoopAction = UIEngine::PROCESS_PAUSED;
      uiEngine->mainEngine->savePoints();
    }
    break;
  case 't':
    uiEngine->intergrationActive = !uiEngine->intergrationActive;
    if (uiEngine->intergrationActive) uiEngine->mainEngine->turnOnIntegration();
    else uiEngine->mainEngine->turnOffIntegration();
    break;
  case 'u':
    printf("refine segment ...\n");
    uiEngine->mainLoopAction = UIEngine::REFINED_SEG;
    break;
  case 'v':
    printf("save current view ...\n");
    uiEngine->mainEngine->saveViewPoints();
    break;
  case 'w':
    printf("save current view w...\n");
    uiEngine->mainEngine->saveViewPoints(uiEngine->mainEngine->trackingStateTem);
    break;
  case '0':
    printf("show object seg result ...\n");
    UIEngine::Instance()->mainEngine->switchShowModel(0);
    uiEngine->mainLoopAction = UIEngine::PROCESS_FRAME;
    break;
  case '1':
    printf("show confidence graph ...\n");
    UIEngine::Instance()->mainEngine->switchShowModel(1);
    uiEngine->mainLoopAction = UIEngine::PROCESS_FRAME;
    break;
  case '2'://just for test
    {
      /* CInteractionCompute cic(UIEngine::Instance()->mainEngine->cPointCloudAnalysis);

      if(cic.vecObjectHypo[0].objectness < 200){
      break;
      }

      Eigen::Vector3f position_under_kinect;
      Eigen::Vector3f direction;
      cic.getTouchPointAndDir(0, position_under_kinect, direction, true);*/
      break;
    }
  case 27: // esc key
    printf("exiting ...\n");
    uiEngine->mainLoopAction = UIEngine::PROCESS_EXIT;
    break;
  default:
    break;
  }
}

void UIEngine::glutMouseButtonFunction(int button, int state, int x, int y)
{
  UIEngine *uiEngine = UIEngine::Instance();

  if (state == GLUT_DOWN)
  {
    switch (button)
    {
    case GLUT_LEFT_BUTTON: uiEngine->mouseState = 1; break;
    case GLUT_MIDDLE_BUTTON: uiEngine->mouseState = 3; break;
    case GLUT_RIGHT_BUTTON: uiEngine->mouseState = 2; break;
    default: break;
    }
    uiEngine->mouseLastClick.x = x;
    uiEngine->mouseLastClick.y = y;
  }
  else if (state == GLUT_UP) uiEngine->mouseState = 0;
}

static inline Matrix3f createRotation(const Vector3f & _axis, float angle)
{
  Vector3f axis = normalize(_axis);
  float si = sinf(angle);
  float co = cosf(angle);

  Matrix3f ret;
  ret.setIdentity();

  ret *= co;
  for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) ret.at(c,r) += (1.0f - co) * axis[c] * axis[r];

  Matrix3f skewmat;
  skewmat.setZeros();
  skewmat.at(1,0) = -axis.z;
  skewmat.at(0,1) =  axis.z;
  skewmat.at(2,0) =  axis.y;
  skewmat.at(0,2) = -axis.y;
  skewmat.at(2,1) =  axis.x;
  skewmat.at(1,2) = -axis.x;
  skewmat *= si;
  ret += skewmat;

  return ret;
}

void UIEngine::glutMouseMoveFunction(int x, int y)
{
  UIEngine *uiEngine = UIEngine::Instance();

  if (!uiEngine->freeviewActive) return;

  Vector2i movement;
  movement.x = x - uiEngine->mouseLastClick.x;
  movement.y = y - uiEngine->mouseLastClick.y;
  uiEngine->mouseLastClick.x = x;
  uiEngine->mouseLastClick.y = y;

  if ((movement.x == 0) && (movement.y == 0)) return;

  static const float scale_rotation = 0.005f;
  static const float scale_translation = 0.0025f;

  switch (uiEngine->mouseState)
  {
  case 1:
    {
      // left button: rotation
      Vector3f axis((float)-movement.y, (float)-movement.x, 0.0f);
      float angle = scale_rotation * sqrtf((float)(movement.x * movement.x + movement.y*movement.y));
      Matrix3f rot = createRotation(axis, angle);
      uiEngine->freeviewPose.R = rot * uiEngine->freeviewPose.R;
      uiEngine->freeviewPose.T = rot * uiEngine->freeviewPose.T;
      uiEngine->freeviewPose.SetParamsFromModelView();
      uiEngine->freeviewPose.SetModelViewFromParams();
      uiEngine->needsRefresh = true;
      break;
    }
  case 2:
    {
      // right button: translation in x and y direction
      uiEngine->freeviewPose.T.x += scale_translation * movement.x;
      uiEngine->freeviewPose.T.y += scale_translation * movement.y;
      uiEngine->freeviewPose.SetParamsFromModelView();
      uiEngine->freeviewPose.SetModelViewFromParams();
      uiEngine->needsRefresh = true;
      break;
    }
  case 3:
    {
      // middle button: translation along z axis
      uiEngine->freeviewPose.T.z += scale_translation * movement.y;
      uiEngine->freeviewPose.SetParamsFromModelView();
      uiEngine->freeviewPose.SetModelViewFromParams();
      uiEngine->needsRefresh = true;
      break;
    }
  default: break;
  }
}

void UIEngine::glutMouseWheelFunction(int button, int dir, int x, int y)
{
  UIEngine *uiEngine = UIEngine::Instance();

  static const float scale_translation = 0.05f;

  if (dir > 0) uiEngine->freeviewPose.T.z -= scale_translation;
  else uiEngine->freeviewPose.T.z += scale_translation;

  uiEngine->freeviewPose.SetParamsFromModelView();
  uiEngine->freeviewPose.SetModelViewFromParams();
  uiEngine->needsRefresh = true;
}

void UIEngine::Initialise(int & argc, char** argv, ITMLibSettings *internalSettings, ImageSourceEngine *imageSource, ITMMainEngine *mainEngine, const char *outFolder)
{
  this->show_mode = 0;//hao modified it
  this->freeviewActive = false;
  this->colourActive = false;
  this->intergrationActive = true;

  this->internalSettings = internalSettings;
  this->imageSource = imageSource;
  this->mainEngine = mainEngine;
  {
    size_t len = strlen(outFolder);
    this->outFolder = new char[len + 1];
    strcpy(this->outFolder, outFolder);
  }

  //Vector2i winSize;
  //int textHeight = 30; // Height of text area
  //winSize.x = 2 * MAX(imageSource->getRGBImageSize().x, imageSource->getDepthImageSize().x);
  //winSize.y = MAX(imageSource->getRGBImageSize().y, imageSource->getDepthImageSize().y) + textHeight;
  //float h1 = textHeight / (float)winSize.y, h2 = (1.f + h1) / 2;
  //winReg[0] = Vector4f(0, h1, 0.5, 1); // Main render
  //winReg[1] = Vector4f(0.5, h2, 0.75, 1); // Side sub window 0
  //winReg[2] = Vector4f(0.75, h2, 1, 1); // Side sub window 1
  //winReg[3] = Vector4f(0.5, h1, 0.75, h2); // Side sub window 2
  //winReg[4] = Vector4f(0.75, h1, 1, h2); // Side sub window 3

  int textHeight = 30; // Height of text area
  //winSize.x = (int)(1.5f * (float)MAX(imageSource->getImageSize().x, imageSource->getDepthImageSize().x));
  //winSize.y = MAX(imageSource->getRGBImageSize().y, imageSource->getDepthImageSize().y) + textHeight;
  winSize.x = (int)(1.5f * (float)(imageSource->getDepthImageSize().x));
  //winSize.y = imageSource->getDepthImageSize().y + textHeight;
  winSize.y = imageSource->getDepthImageSize().y;//hao modified it
  //float h1 = textHeight / (float)winSize.y, h2 = (1.f + h1) / 2;
  float h1 = 0, h2 = (1.f + h1) / 2;
  winReg[0] = Vector4f(0.0f, h1, 0.665f, 1.0f);   // Main render
  winReg[1] = Vector4f(0.665f, h2, 1.0f, 1.0f);   // Side sub window 0
  winReg[2] = Vector4f(0.665f, h1, 1.0f, h2);     // Side sub window 2

  this->isRecording = false; 
  this->currentFrameNo = 0;

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(winSize.x, winSize.y);
  glutCreateWindow("ACSROA");//hao modified it
  glGenTextures(NUM_WIN, textureId);

  glutDisplayFunc(UIEngine::glutDisplayFunction);
  glutKeyboardUpFunc(UIEngine::glutKeyUpFunction);
  glutMouseFunc(UIEngine::glutMouseButtonFunction);
  glutMotionFunc(UIEngine::glutMouseMoveFunction);
  glutIdleFunc(UIEngine::glutIdleFunction);

#ifdef FREEGLUT
  glutMouseWheelFunc(UIEngine::glutMouseWheelFunction);
  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, 1);
#endif

  for (int w = 0; w < NUM_WIN; w++)
    outImage[w] = new ITMUChar4Image(imageSource->getDepthImageSize(), false);

  saveImage = new ITMUChar4Image(imageSource->getDepthImageSize(), false);

  outImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
  outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
  outImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;
  //outImageType[3] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
  //outImageType[4] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;

  mainLoopAction = PROCESS_PAUSED;
  mouseState = 0;
  needsRefresh = false;
  processedFrameNo = 0;
  processedTime = 0.0f;

  sdkCreateTimer(&timer);

  printf("initialised.\n");
}

//hao modified it
void UIEngine::resetEngine()
{
  this->show_mode = 0;//hao modified it
  this->freeviewActive = false;
  this->colourActive = false;
  this->intergrationActive = true;

  delete mainEngine;
  mainEngine = new ITMMainEngine(internalSettings, &imageSource->calib, imageSource->getRGBImageSize(), imageSource->getDepthImageSize());

  {
    size_t len = strlen(outFolder);
    this->outFolder = new char[len + 1];
    strcpy(this->outFolder, outFolder);
  }

  int textHeight = 30; // Height of text area
  winSize.x = (int)(1.5f * (float)(imageSource->getDepthImageSize().x));
  winSize.y = imageSource->getDepthImageSize().y + textHeight;
  //float h1 = textHeight / (float)winSize.y, h2 = (1.f + h1) / 2;
  float h1 = 0, h2 = (1.f + h1) / 2;
  winReg[0] = Vector4f(0.0f, h1, 0.665f, 1.0f);   // Main render
  winReg[1] = Vector4f(0.665f, h2, 1.0f, 1.0f);   // Side sub window 0
  winReg[2] = Vector4f(0.665f, h1, 1.0f, h2);     // Side sub window 2

  this->isRecording = false; 
  this->currentFrameNo = 0;

  for (int w = 0; w < NUM_WIN; w++)
    outImage[w] = new ITMUChar4Image(imageSource->getDepthImageSize(), false);

  saveImage = new ITMUChar4Image(imageSource->getDepthImageSize(), false);

  outImageType[0] = ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST;
  outImageType[1] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH;
  outImageType[2] = ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB;

  mainLoopAction = PROCESS_PAUSED;
  mouseState = 0;
  needsRefresh = false;
  processedFrameNo = 0;
  processedTime = 0.0f;

  sdkCreateTimer(&timer);
}

void UIEngine::SaveScreenshot(const char *filename) const
{
  ITMUChar4Image screenshot(getWindowSize());
  GetScreenshot(&screenshot);
  SaveImageToFile(&screenshot, filename, true);
}

void UIEngine::GetScreenshot(ITMUChar4Image *dest) const
{
  glReadPixels(0, 0, dest->noDims.x, dest->noDims.y, GL_RGBA, GL_UNSIGNED_BYTE, dest->GetData(false));
}

//hao modified it
void UIEngine::ProcessFrame(short segFlag)
{
  if (!imageSource->hasMoreImages()) return;
  imageSource->getImages(mainEngine->view);

  sdkResetTimer(&timer); sdkStartTimer(&timer);

  if (isRecording)
  {
    char str[250];

    sprintf(str, "%s/%04d.pgm", outFolder, currentFrameNo);
    SaveImageToFile(mainEngine->view->rawDepth, str);

    sprintf(str, "%s/%04d.ppm", outFolder, currentFrameNo);
    SaveImageToFile(mainEngine->view->rgb, str);
  }

  //actual processing on the mailEngine
  mainEngine->ProcessFrame(segFlag);

  sdkStopTimer(&timer); processedTime = sdkGetTimerValue(&timer);

  currentFrameNo++;
}

void UIEngine::Run() { glutMainLoop(); }
void UIEngine::Shutdown() 
{ 
  sdkDeleteTimer(&timer);

  for (int w = 0; w < NUM_WIN; w++)
    delete outImage[w]; 
  delete[] outFolder;
  delete saveImage; 
  delete instance; 
}

//hao modified it
DWORD _stdcall autoscan(LPVOID lpParameter)
  //void* autoscan(void* argc) 
{  
  SOCKET sockClient;

  //first step
  open_socket(sockClient);

  //second step
  float pos_left_arm[7];
  //float pos_right_arm[7];
  //float pos_right_arm[] = {0.564, -0.328, 0.213, -1.222, 2.373, -2.014, -3.376};
  //float pos_right_arm[] = {0.564, 0.168, -0.040, -1.959, -3.629, -1.960, 2.948};
  Eigen::Vector3f head_focus = UIEngine::Instance()->mainEngine->robotpose.head_focus;
  //Eigen::Vector3f head_focus(1.5, 0.5, 0.6);
  float torso_up = UIEngine::Instance()->mainEngine->robotpose.torso_up;
  init_robot_pose(sockClient, UIEngine::Instance()->mainEngine->robotpose.pos_left_arm, UIEngine::Instance()->mainEngine->robotpose.pos_right_arm, head_focus, torso_up);

  //third step
  printf("processing input source ...\n");
  UIEngine::Instance()->mainLoopAction = UIEngine::PROCESS_VIDEO;

  //fourth step
  float down_value = UIEngine::Instance()->mainEngine->robotpose.down_value;
  float up_value = UIEngine::Instance()->mainEngine->robotpose.up_value;
  up_down_right_scanner(sockClient, down_value, up_value);

  //fifth step
  printf("processing one frame ...\n");
  UIEngine::Instance()->mainLoopAction = UIEngine::PROCESS_FRAME;

  //sixth step
  printf("save current view ...\n");
  UIEngine::Instance()->mainEngine->saveViewPoints();

  //seventh step
  printf("segment objects ...\n");
  UIEngine::Instance()->mainLoopAction = UIEngine::SEG_FRAME;

  while(UIEngine::Instance()->mainLoopAction == UIEngine::SEG_FRAME){
    Sleep(5000);   
  }

  ////eighth step
  //float pos_left_arm_ins[7];
  //init_left_arm_pose(pos_left_arm_ins);

  //fourteenth step
  //init_left_arm_pose(pos_left_arm_ins);

  //eighth step
  //Eigen::Vector3f position_under_kinect(0.80, 0.40, 0.780);
  //Eigen::Vector3f direction(1, 0, 0);
  Eigen::Vector3f position;
  Eigen::Vector3f direction;

  Eigen::Vector3f position_under_kinect;
  Eigen::Vector3f direction_under_kinect;

  //CInteractionCompute cic(UIEngine::Instance()->mainEngine->cPointCloudAnalysis);

  ////for(int i=0; i<cic.vecObjectHypo.size(); i++){
  //for(int i=0; i<1; i++){
  //  if(cic.vecObjectHypo[i].objectness < 200){
  //    break;
  //  }

  //  cic.getTouchPointAndDir(i, position_under_kinect, direction_under_kinect, false);

  //  get_l_touch_point_and_dir(sockClient, position_under_kinect, direction_under_kinect, position, direction);
  //  //test_calibration_result(sockClient, position_under_kinect, direction_under_kinect);
  //  set_head_pose(sockClient, position);
  //  l_push_object(sockClient, position, direction);

  //  l_take_back(sockClient, position, direction);

  //  //ninth step
  //  init_left_arm_pose(sockClient, pos_left_arm);

  //  //tenth step
  //  printf("processing input source ...\n");
  //  UIEngine::Instance()->mainLoopAction = UIEngine::PROCESS_VIDEO;

  //  //eleventh step
  //  up_down_right_scanner(sockClient, down_value, up_value);

  //  //twelfth step
  //  printf("save current view w...\n");
  //  UIEngine::Instance()->mainEngine->saveViewPoints(UIEngine::Instance()->mainEngine->trackingStateTem);

  //  //thirteenth step
  //  printf("segment objects ...\n");
  //  UIEngine::Instance()->mainLoopAction = UIEngine::UPDATE_SEG_FRAME;

  //  while(UIEngine::Instance()->mainLoopAction == UIEngine::UPDATE_SEG_FRAME){
  //    Sleep(5000);   
  //  }
  //}

  //sixteenth step
  close_socket(sockClient);

  return 0;
}  

//hao modified it
void UIEngine::autoReconstruct(){
  CreateThread(NULL,0,autoscan,NULL,0,NULL);
}