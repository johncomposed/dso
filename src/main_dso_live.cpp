/**
* This file is part of DSO.
* 
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/




#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"


// #include <ros/ros.h>
// #include <sensor_msgs/image_encodings.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/CameraInfo.h>
// #include <geometry_msgs/PoseStamped.h>
// #include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"


std::string calib = "";
std::string vignetteFile = "";
std::string gammaFile = "";
std::string vidFile="";
bool useSampleOutput=false;

using namespace dso;

void parseArgument(char* arg)
{
  int option;
  char buf[1000];

  if(1==sscanf(arg,"sampleoutput=%d",&option))
  {
    if(option==1)
    {
      useSampleOutput = true;
      printf("USING SAMPLE OUTPUT WRAPPER!\n");
    }
    return;
  }

  if(1==sscanf(arg,"quiet=%d",&option))
  {
    if(option==1)
    {
      setting_debugout_runquiet = true;
      printf("QUIET MODE, I'll shut up!\n");
    }
    return;
  }


  if(1==sscanf(arg,"nolog=%d",&option))
  {
    if(option==1)
    {
      setting_logStuff = false;
      printf("DISABLE LOGGING!\n");
    }
    return;
  }

  if(1==sscanf(arg,"nogui=%d",&option))
  {
    if(option==1)
    {
      disableAllDisplay = true;
      printf("NO GUI!\n");
    }
    return;
  }
  if(1==sscanf(arg,"nomt=%d",&option))
  {
    if(option==1)
    {
      multiThreading = false;
      printf("NO MultiThreading!\n");
    }
    return;
  }
  if(1==sscanf(arg,"calib=%s",buf))
  {
    calib = buf;
    printf("loading calibration from %s!\n", calib.c_str());
    return;
  }
  if(1==sscanf(arg,"vignette=%s",buf))
  {
    vignetteFile = buf;
    printf("loading vignette from %s!\n", vignetteFile.c_str());
    return;
  }

  if(1==sscanf(arg,"gamma=%s",buf))
  {
    gammaFile = buf;
    printf("loading gammaCalib from %s!\n", gammaFile.c_str());
    return;
  }

  if(1==sscanf(arg,"vidfile=%s",buf))
  {
    vidFile = buf;
    printf("loading video file from %s!\n", vidFile.c_str());
    return;
  }

  printf("could not parse argument \"%s\"!!\n", arg);
}




FullSystem* fullSystem = 0;
Undistort* undistorter = 0;
int frameID = 0;

void vidCb(const cv::Mat img, double timestamp) {
  // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  // assert(cv_ptr->image.type() == CV_8U);
  // assert(cv_ptr->image.channels() == 1);
  printf("Timestamp:: %f\n", timestamp);


  if(setting_fullResetRequested) {
    printf("Setting Ful Reset Requested\n");

    std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
    delete fullSystem;
    for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
    fullSystem = new FullSystem();
    fullSystem->linearizeOperation=false;
    fullSystem->outputWrapper = wraps;
    if(undistorter->photometricUndist != 0) {
      printf("If  happened?\n");
      fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
    }
    setting_fullResetRequested=false;
  }

  printf("minImg: %d %d\n", (int)img.cols, (int)img.rows);

  MinimalImageB minImg((int)img.cols, (int)img.rows,(unsigned char*)img.data);
  ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1,0, 1.0f);
  undistImg->timestamp=timestamp;


  fullSystem->addActiveFrame(undistImg, frameID);



  printf("If ur seeing this its 2 late - it works\n");
  frameID++;
  delete undistImg;
}





int main( int argc, char** argv ) {
  // ros::init(argc, argv, "dso_live");

  for(int i=1; i<argc;i++) parseArgument(argv[i]);

  // Let's try opening after we parse the args
  cv::VideoCapture cap(vidFile); 
  if(!cap.isOpened())  // check if we succeeded
      return -1;

  // cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
  // cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);


  setting_desiredImmatureDensity = 1000;
  setting_desiredPointDensity = 1200;
  setting_minFrames = 5;
  setting_maxFrames = 7;
  setting_maxOptIterations=4;
  setting_minOptIterations=1;
  setting_logStuff = false;
  setting_kfGlobalWeight = 1.3;


  printf("MODE WITH CALIBRATION, but without exposure times!\n");
  setting_photometricCalibration = 2;
  setting_affineOptModeA = 0;
  setting_affineOptModeB = 0;



    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

    setGlobalCalib(
            (int)undistorter->getSize()[0],
            (int)undistorter->getSize()[1],
            undistorter->getK().cast<float>());


    fullSystem = new FullSystem();
    fullSystem->linearizeOperation=false;

    IOWrap::PangolinDSOViewer* viewer = 0;
    if(!disableAllDisplay) {
      viewer = new IOWrap::PangolinDSOViewer(
          (int)undistorter->getSize()[0],
          (int)undistorter->getSize()[1]
        );
      fullSystem->outputWrapper.push_back(viewer);

    }

    if(useSampleOutput)
        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());


    if(undistorter->photometricUndist != 0)
      fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

    // ros::NodeHandle nh;
    // ros::Subscriber imgSub = nh.subscribe("image", 1, &vidCb);


    // To make macOS happy?
    std::thread runthread([&]() {
      cv::Mat edges;
      cv::namedWindow("edges",1);
      while(true) {
          cv::Mat frame;

          // http://stackoverflow.com/questions/35910547/opencv-get-frame-by-timestamp
          // int timestamp = (int)cap.get(CV_CAP_PROP_POS_MSEC); 
          double timestamp = (double)std::time(nullptr);

          cap >> frame; // get a new frame from camera
          cv::cvtColor(frame, edges, CV_BGR2GRAY);
          
          vidCb(frame, timestamp);

          if(cv::waitKey(30) >= 0) {
            printf("More than 30 wait, breaking\n");
            break;
          }
      }

    });


    if(viewer != 0) viewer->run();

    runthread.join();


    fullSystem->printResult("result1.txt");
    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper) {
        ow->join();
        delete ow;
    }
    delete undistorter;
    delete fullSystem;

  return 0;
}

