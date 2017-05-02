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

#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/ImageDisplay.h"


#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/DatasetReader.h"
#include "util/globalCalib.h"

#include "util/NumType.h"
#include "FullSystem/FullSystem.h"
#include "OptimizationBackend/MatrixAccumulators.h"
#include "FullSystem/PixelSelector2.h"

#include "IOWrapper/Pangolin/PangolinDSOViewer.h"

#include "opencv2/opencv.hpp"



std::string vignette = "";
std::string gammaCalib = "";
std::string source = "";
std::string calib = "";
double rescale = 1;
int start=0;
int end=100000;
float playbackSpeed=0;  // 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.
bool video=false;



using namespace dso;


void my_exit_handler(int s) {
  printf("Caught signal %d\n",s);
  exit(1);
}

void exitThread() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  while(true) pause();
}


void setDefaults() {
  printf("DEFAULT settings:\n"
  "- %s real-time enforcing\n"
  "- 2000 active points\n"
  "- 5-7 active frames\n"
  "- 1-6 LM iteration each KF\n"
  "- original image resolution\n", playbackSpeed==0 ? "no " : "1x");


  setting_desiredImmatureDensity = 1500;
  setting_desiredPointDensity = 2000;
  setting_minFrames = 5;
  setting_maxFrames = 7;
  setting_maxOptIterations=6;
  setting_minOptIterations=1;


  // printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
  // setting_photometricCalibration = 0;
  // setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
  // setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).

  setting_logStuff = false;
}




void parseArgument(char* arg) {
  int option;
  float foption;
  char buf[1000];


  if(1==sscanf(arg,"quiet=%d",&option)) {
    if(option==1) {
      setting_debugout_runquiet = true;
      printf("QUIET MODE, I'll shut up!\n");
    }
    return;
  }

  if(1==sscanf(arg,"rec=%d",&option)) {
    if(option==0) {
      disableReconfigure = true;
      printf("DISABLE RECONFIGURE!\n");
    }
    return;
  }


  if(1==sscanf(arg,"nolog=%d",&option)) {
    if(option==1) {
      setting_logStuff = false;
      printf("DISABLE LOGGING!\n");
    }
    return;
  }

  if(1==sscanf(arg,"nogui=%d",&option)) {
    if(option==1) {
      disableAllDisplay = true;
      printf("NO GUI!\n");
    }
    return;
  }
  if(1==sscanf(arg,"nomt=%d",&option)) {
    if(option==1) {
      multiThreading = false;
      printf("NO MultiThreading!\n");
    }
    return;
  }

  if(1==sscanf(arg,"start=%d",&option)) {
    start = option;
    printf("START AT %d!\n",start);
    return;
  }
  if(1==sscanf(arg,"end=%d",&option)) {
    end = option;
    printf("END AT %d!\n",start);
    return;
  }

  if(1==sscanf(arg,"files=%s",buf)) {
    source = buf;
    printf("loading data from %s!\n", source.c_str());
    return;
  }


  if(1==sscanf(arg,"video=%d",&option)) {
    video = (bool) option;
    printf("Is the source a video: %d!\n", video);
    return;
  }


  if(1==sscanf(arg,"calib=%s",buf)) {
    calib = buf;
    printf("loading calibration from %s!\n", calib.c_str());
    return;
  }

  if(1==sscanf(arg,"vignette=%s",buf)) {
    vignette = buf;
    printf("loading vignette from %s!\n", vignette.c_str());
    return;
  }

  if(1==sscanf(arg,"gamma=%s",buf)) {
    gammaCalib = buf;
    printf("loading gammaCalib from %s!\n", gammaCalib.c_str());
    return;
  }

  if(1==sscanf(arg,"rescale=%f",&foption)) {
    rescale = foption;
    printf("RESCALE %f!\n", rescale);
    return;
  }

  if(1==sscanf(arg,"speed=%f",&foption)) {
    playbackSpeed = foption;
    printf("PLAYBACK SPEED %f!\n", playbackSpeed);
    return;
  }

  if(1==sscanf(arg,"save=%d",&option)) {
    if(option==1) {
      debugSaveImages = true;
      if(42==system("rm -rf images_out")) printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
      if(42==system("mkdir images_out")) printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
      if(42==system("rm -rf images_out")) printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
      if(42==system("mkdir images_out")) printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
      printf("SAVE IMAGES!\n");
    }
    return;
  }

  printf("could not parse argument \"%s\"!!!!\n", arg);
}




// void vidCb(const cv::Mat img, double timestamp) {

// }











int main( int argc, char** argv ) {
  for(int i=1; i<argc;i++) parseArgument(argv[i]);

  setDefaults();

  // hook crtl+C.
  boost::thread exThread = boost::thread(exitThread);
  ImageFolderReader* reader;



  reader = new ImageFolderReader(source,calib, gammaCalib, vignette);
  reader->setGlobalCalibration();


  if(setting_photometricCalibration > 0 && reader->getPhotometricGamma() == 0) {
    printf("ERROR: dont't have photometric calibation. Need to use commandline options mode=1 or mode=2 ");
    exit(1);
  }


  int linc = 1;
  FullSystem* fullSystem = new FullSystem();
  fullSystem->setGammaFunction(reader->getPhotometricGamma());
  fullSystem->linearizeOperation = (playbackSpeed==0);


  IOWrap::PangolinDSOViewer* viewer = 0;
  if(!disableAllDisplay) {
    viewer = new IOWrap::PangolinDSOViewer(wG[0],hG[0], false);
    fullSystem->outputWrapper.push_back(viewer);
  }


  // to make MacOS happy: run this in dedicated thread -- and use this one to run the GUI.
  std::thread runthread([&]() {
    std::vector<int> idsToPlay;
    std::vector<double> timesToPlayAt;
    for(int i=start;i>= 0 && i< reader->getNumImages() && linc*i < linc*end;i+=linc) {
      idsToPlay.push_back(i);
      if(timesToPlayAt.size() == 0) {
        timesToPlayAt.push_back((double)0);
      }
      else {
        double tsThis = reader->getTimestamp(idsToPlay[idsToPlay.size()-1]);
        double tsPrev = reader->getTimestamp(idsToPlay[idsToPlay.size()-2]);
        timesToPlayAt.push_back(timesToPlayAt.back() +  fabs(tsThis-tsPrev)/playbackSpeed);
      }
    }


    struct timeval tv_start;
    gettimeofday(&tv_start, NULL);
    clock_t started = clock();
    double sInitializerOffset=0;


    for(int ii=0;ii<(int)idsToPlay.size(); ii++) {
      if(!fullSystem->initialized) {  // if not initialized: reset start time. 
        gettimeofday(&tv_start, NULL);
        started = clock();
        sInitializerOffset = timesToPlayAt[ii];
      }

      int i = idsToPlay[ii];


      ImageAndExposure* img = reader->getImage(i);



      bool skipFrame=false;
      if(playbackSpeed!=0) {
        struct timeval tv_now; gettimeofday(&tv_now, NULL);
        double sSinceStart = sInitializerOffset + ((tv_now.tv_sec-tv_start.tv_sec) + (tv_now.tv_usec-tv_start.tv_usec)/(1000.0f*1000.0f));

        if(sSinceStart < timesToPlayAt[ii])
          usleep((int)((timesToPlayAt[ii]-sSinceStart)*1000*1000));
        else if(sSinceStart > timesToPlayAt[ii]+0.5+0.1*(ii%2)) {
          printf("SKIPFRAME %d (play at %f, now it is %f)!\n", ii, timesToPlayAt[ii], sSinceStart);
          skipFrame=true;
        }
      }


      // printf("--- JW Skip Frame %d ---\n", skipFrame);

      if(!skipFrame) fullSystem->addActiveFrame(img, i);

      delete img;

      if(fullSystem->initFailed || setting_fullResetRequested) {
        if(ii < 250 || setting_fullResetRequested) {
          printf("RESETTING!\n");

          std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
          delete fullSystem;

          for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();

          fullSystem = new FullSystem();
          fullSystem->setGammaFunction(reader->getPhotometricGamma());
          fullSystem->linearizeOperation = (playbackSpeed==0);


          fullSystem->outputWrapper = wraps;

          setting_fullResetRequested=false;
        }
      }

      if(fullSystem->isLost) {
          printf("LOST!!\n");
          break;
      }

    }
    fullSystem->blockUntilMappingIsFinished();
    clock_t ended = clock();
    struct timeval tv_end;
    gettimeofday(&tv_end, NULL);


    fullSystem->printResult("result.txt");

    int numFramesProcessed = abs(idsToPlay[0]-idsToPlay.back());
    double numSecondsProcessed = fabs(reader->getTimestamp(idsToPlay[0])-reader->getTimestamp(idsToPlay.back()));
    double MilliSecondsTakenSingle = 1000.0f*(ended-started)/(float)(CLOCKS_PER_SEC);
    double MilliSecondsTakenMT = sInitializerOffset + ((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f);
    printf("\n======================"
        "\n%d Frames (%.1f fps)"
        "\n%.2fms per frame (single core); "
        "\n%.2fms per frame (multi core); "
        "\n%.3fx (single core); "
        "\n%.3fx (multi core); "
        "\n======================\n\n",
        numFramesProcessed, numFramesProcessed/numSecondsProcessed,
        MilliSecondsTakenSingle/numFramesProcessed,
        MilliSecondsTakenMT / (float)numFramesProcessed,
        1000 / (MilliSecondsTakenSingle/numSecondsProcessed),
        1000 / (MilliSecondsTakenMT / numSecondsProcessed));

    if(setting_logStuff) {
      std::ofstream tmlog;
      tmlog.open("logs/time.txt", std::ios::trunc | std::ios::out);
      tmlog << 1000.0f*(ended-started)/(float)(CLOCKS_PER_SEC*reader->getNumImages()) << " "
          << ((tv_end.tv_sec-tv_start.tv_sec)*1000.0f + (tv_end.tv_usec-tv_start.tv_usec)/1000.0f) / (float)reader->getNumImages() << "\n";
      tmlog.flush();
      tmlog.close();
    }

  });


  if(viewer != 0) viewer->run();
  runthread.join();

  for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper) {
    ow->join();
    delete ow;
  }



  printf("DELETE FULLSYSTEM!\n");
  delete fullSystem;

  printf("DELETE READER!\n");
  delete reader;

  printf("EXIT NOW!\n");
  return 0;
}






/*
OLD LIVE

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



*/
