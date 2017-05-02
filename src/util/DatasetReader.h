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


#pragma once
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/globalCalib.h"

#include <functional>
#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include <cmath>

#include "util/Undistort.h"
#include "IOWrapper/ImageRW.h"
// maybe also ImageRW_OpenCV

#if HAS_ZIPLIB
	#include "zip.h"
#endif

#include <boost/thread.hpp>
#include "opencv2/opencv.hpp"


using namespace dso;



inline int getdir (std::string dir, std::vector<std::string> &files) {
  DIR *dp;
  struct dirent *dirp;
  if((dp  = opendir(dir.c_str())) == NULL) {
      return -1;
  }

  while ((dirp = readdir(dp)) != NULL) {
  	std::string name = std::string(dirp->d_name);

  	if(name != "." && name != "..")
  		files.push_back(name);
  }
  closedir(dp);


  std::sort(files.begin(), files.end());

  if(dir.at( dir.length() - 1 ) != '/') dir = dir+"/";
	for(unsigned int i=0;i<files.size();i++) {
		if(files[i].at(0) != '/')
			files[i] = dir + files[i];
	}

    return files.size();
}


struct PrepImageItem {
	int id;
	bool isQueud;
	ImageAndExposure* pt;

	inline PrepImageItem(int _id) {
		id=_id;
		isQueud = false;
		pt=0;
	}

	inline void release() {
		if(pt!=0) delete pt;
		pt=0;
	}
};



class VideoReader {
public:
  Undistort* undistort;
  cv::VideoCapture cap;

  VideoReader(std::string path, std::string calibFile, std::string gammaFile, std::string vignetteFile) {
    this->path = path;
    this->calibfile = calibFile;

    cap = cv::VideoCapture();
    undistort = Undistort::getUndistorterForFile(calibFile, gammaFile, vignetteFile);

    widthOrg = undistort->getOriginalSize()[0];
    heightOrg = undistort->getOriginalSize()[1];
    width=undistort->getSize()[0];
    height=undistort->getSize()[1];


    //TODO: This should be generated and not depentant on times.txt
    loadTimestamps();


    //TODO: getVid 
  }
  ~VideoReader() {
    delete undistort;
  };

  Eigen::VectorXf getOriginalCalib() {
    return undistort->getOriginalParameter().cast<float>();
  }
  Eigen::Vector2i getOriginalDimensions() {
    return  undistort->getOriginalSize();
  }

  void getCalibMono(Eigen::Matrix3f &K, int &w, int &h) {
    K = undistort->getK().cast<float>();
    w = undistort->getSize()[0];
    h = undistort->getSize()[1];
  }

  void setGlobalCalibration() {
    int w_out, h_out;
    Eigen::Matrix3f K;
    getCalibMono(K, w_out, h_out);
    setGlobalCalib(w_out, h_out, K);
  }

  inline float* getPhotometricGamma() {
    if(undistort==0 || undistort->photometricUndist==0) return 0;
    return undistort->photometricUndist->getG();
  }

  int numFramesProcessed() {
    return 1000; //TODO
  }

  double numSecondsProcessed() {
    return 1000.2; //TODO
  }

  int getNumImages() {
    return 100000; // TODO
  }

  int makeOffset(int index) {
    return 0; //TODO
  }

  double getTimestamp(int id) {
    if(timestamps.size()==0) return id*0.1f;
    if(id >= (int)timestamps.size()) return 0;
    if(id < 0) return 0;
    return timestamps[id];
  }

  void prepVideo(int playbackSpeed=0, int start=0, int end=100000) {
    printf("playback speed: %d\n", playbackSpeed);
    cap.open(path);


  }


  void readVideo(std::function<bool (ImageAndExposure*, int)> func) {
    cv::Mat edges;
    int index = 0;
    cv::namedWindow("edges",1);

    while(true) {
      cv::Mat frame;
      // http://stackoverflow.com/questions/35910547/opencv-get-frame-by-timestamp
      // int timestamp = (int)cap.get(CV_CAP_PROP_POS_MSEC); 
      // double timestamp = (double)std::time(nullptr);

      cap >> frame; // get a new frame from camera
      cv::cvtColor(frame, edges, CV_BGR2GRAY); // Grayscale
      MinimalImageB minImg((int)frame.cols, (int)frame.rows,(unsigned char*)frame.data);

      ImageAndExposure* img = undistort->undistort<unsigned char>(&minImg, 1,0, 1.0f);
      img->timestamp = getTimestamp(index);

      bool continuepls = func(img, index);
      if(!continuepls) break;

      // if(cv::waitKey(30) >= 0) {
      //   printf("More than 30 wait, breaking\n");
      //   break;
      // }

      index += 1;
    }
  }


private:
  std::vector<double> timestamps;
  std::vector<float> exposures;

  int width, height;
  int widthOrg, heightOrg;

  std::string path;
  std::string calibfile;


  inline void loadTimestamps() {
    std::ifstream tr;
    std::string timesFile = path.substr(0,path.find_last_of('/')) + "/times.txt";
    tr.open(timesFile.c_str());
    while(!tr.eof() && tr.good()) {
      std::string line;
      char buf[1000];
      tr.getline(buf, 1000);

      int id;
      double stamp;
      float exposure = 0;

      if(3 == sscanf(buf, "%d %lf %f", &id, &stamp, &exposure)) {
        timestamps.push_back(stamp);
        exposures.push_back(exposure);
      }

      else if(2 == sscanf(buf, "%d %lf", &id, &stamp)) {
        timestamps.push_back(stamp);
        exposures.push_back(exposure);
      }
    }
    tr.close();

    // check if exposures are correct, (possibly skip)
    bool exposuresGood = ((int)exposures.size()==(int)getNumImages()) ;
    for(int i=0;i<(int)exposures.size();i++) {
      if(exposures[i] == 0) {
        // fix!
        float sum=0,num=0;
        if(i>0 && exposures[i-1] > 0) {sum += exposures[i-1]; num++;}
        if(i+1<(int)exposures.size() && exposures[i+1] > 0) {sum += exposures[i+1]; num++;}

        if(num>0)
          exposures[i] = sum/num;
      }

      if(exposures[i] == 0) exposuresGood=false;
    }


    if((int)getNumImages() != (int)timestamps.size()) {
      printf("set timestamps and exposures to zero!\n");
      exposures.clear();
      timestamps.clear();
    }

    if((int)getNumImages() != (int)exposures.size() || !exposuresGood) {
      printf("set EXPOSURES to zero!\n");
      exposures.clear();
    }

    printf("got %d images and %d timestamps and %d exposures.!\n", (int)getNumImages(), (int)timestamps.size(), (int)exposures.size());
  }  

};






class ImageFolderReader {
public:
  Undistort* undistort;  // undistorter. [0] always exists, [1-2] only when MT is enabled.

	ImageFolderReader(std::string path, std::string calibFile, std::string gammaFile, std::string vignetteFile) {
		this->path = path;
		this->calibfile = calibFile;

    #if HAS_ZIPLIB
    		ziparchive=0;
    		databuffer=0;
    #endif

		isZipped = (path.length()>4 && path.substr(path.length()-4) == ".zip");

		if(isZipped) {
      #if HAS_ZIPLIB
      			int ziperror=0;
      			ziparchive = zip_open(path.c_str(),  ZIP_RDONLY, &ziperror);
      			if(ziperror!=0) {
      				printf("ERROR %d reading archive %s!\n", ziperror, path.c_str());
      				exit(1);
      			}

      			files.clear();
      			int numEntries = zip_get_num_entries(ziparchive, 0);
      			for(int k=0;k<numEntries;k++) {
      				const char* name = zip_get_name(ziparchive, k,  ZIP_FL_ENC_STRICT);
      				std::string nstr = std::string(name);
      				if(nstr == "." || nstr == "..") continue;
      				files.push_back(name);
      			}

      			printf("got %d entries and %d files!\n", numEntries, (int)files.size());
      			std::sort(files.begin(), files.end());
      #else
      			printf("ERROR: cannot read .zip archive, as compile without ziplib!\n");
      			exit(1);
      #endif
		} else getdir(path, files);


		undistort = Undistort::getUndistorterForFile(calibFile, gammaFile, vignetteFile);

		widthOrg = undistort->getOriginalSize()[0];
		heightOrg = undistort->getOriginalSize()[1];
		width=undistort->getSize()[0];
		height=undistort->getSize()[1];


		// load timestamps if possible.
		loadTimestamps();
		printf("ImageFolderReader: got %d files in %s!\n", (int)files.size(), path.c_str());

	}
	~ImageFolderReader() {
    #if HAS_ZIPLIB
    		if(ziparchive!=0) zip_close(ziparchive);
    		if(databuffer!=0) delete databuffer;
    #endif


		delete undistort;
	};

	Eigen::VectorXf getOriginalCalib() {
		return undistort->getOriginalParameter().cast<float>();
	}
	Eigen::Vector2i getOriginalDimensions() {
		return  undistort->getOriginalSize();
	}

	void getCalibMono(Eigen::Matrix3f &K, int &w, int &h) {
		K = undistort->getK().cast<float>();
		w = undistort->getSize()[0];
		h = undistort->getSize()[1];
	}

	void setGlobalCalibration() {
		int w_out, h_out;
		Eigen::Matrix3f K;
		getCalibMono(K, w_out, h_out);
		setGlobalCalib(w_out, h_out, K);
	}

  inline float* getPhotometricGamma() {
    if(undistort==0 || undistort->photometricUndist==0) return 0;
    return undistort->photometricUndist->getG();
  }

  int numFramesProcessed() {
    return abs(idsToPlay[0] - idsToPlay.back());
  }

  double numSecondsProcessed() {
    return fabs(getTimestamp(idsToPlay[0]) - getTimestamp(idsToPlay.back()));
  }

	int getNumImages() {
		return files.size();
	}

  int makeOffset(int index) {
    return timesToPlayAt[index];
  }

  double getTimestamp(int id) {
    if(timestamps.size()==0) return id*0.1f;
    if(id >= (int)timestamps.size()) return 0;
    if(id < 0) return 0;
    return timestamps[id];
  }

  ImageAndExposure* getImage(int id, bool forceLoadDirectly=false) {
    return getImage_internal(id, 0);
  }

  void prepVideo(int playbackSpeed=0, int start=0, int end=100000) {
    printf("playback speed: %d\n", playbackSpeed);

    for(int i=start;i>= 0 && i< getNumImages() && i < end;i+=1) {
      idsToPlay.push_back(i);

      if(timesToPlayAt.size() == 0) {
        timesToPlayAt.push_back((double)0);
      } else {
        double tsThis = getTimestamp(idsToPlay[idsToPlay.size()-1]);
        double tsPrev = getTimestamp(idsToPlay[idsToPlay.size()-2]);
        timesToPlayAt.push_back(timesToPlayAt.back() +  fabs(tsThis-tsPrev)/playbackSpeed);
      }
    }
  }



  void readVideo(std::function<bool (ImageAndExposure*, int)> func) {
    for(int ii=0;ii<(int)idsToPlay.size(); ii++) {
      ImageAndExposure* img = getImage_internal(idsToPlay[ii], 0);
      bool continuepls = func(img, ii);
      if (!continuepls) break;
      
      // the original code deleted img and idk why
      // delete img;
    }
  }



private:
  std::vector<ImageAndExposure*> preloadedImages;
  std::vector<std::string> files;
  std::vector<double> timestamps;
  std::vector<float> exposures;

  std::vector<int> idsToPlay;
  std::vector<double> timesToPlayAt;

  int width, height;
  int widthOrg, heightOrg;

  std::string path;
  std::string calibfile;

  bool isZipped;

  #if HAS_ZIPLIB
    zip_t* ziparchive;
    char* databuffer;
  #endif



	MinimalImageB* getImageRaw_internal(int id, int unused) {
		if(!isZipped) {
			// CHANGE FOR ZIP FILE
			return IOWrap::readImageBW_8U(files[id]);
		}
		else {
      #if HAS_ZIPLIB
  			if(databuffer==0) databuffer = new char[widthOrg*heightOrg*6+10000];
  			zip_file_t* fle = zip_fopen(ziparchive, files[id].c_str(), 0);
  			long readbytes = zip_fread(fle, databuffer, (long)widthOrg*heightOrg*6+10000);

  			if(readbytes > (long)widthOrg*heightOrg*6) {
  				printf("read %ld/%ld bytes for file %s. increase buffer!!\n", readbytes,(long)widthOrg*heightOrg*6+10000, files[id].c_str());
  				delete[] databuffer;
  				databuffer = new char[(long)widthOrg*heightOrg*30];
  				fle = zip_fopen(ziparchive, files[id].c_str(), 0);
  				readbytes = zip_fread(fle, databuffer, (long)widthOrg*heightOrg*30+10000);

  				if(readbytes > (long)widthOrg*heightOrg*30) {
  					printf("buffer still to small (read %ld/%ld). abort.\n", readbytes,(long)widthOrg*heightOrg*30+10000);
  					exit(1);
  				}
  			}

  			return IOWrap::readStreamBW_8U(databuffer, readbytes);
      #else
  			printf("ERROR: cannot read .zip archive, as compile without ziplib!\n");
  			exit(1);
      #endif
		}
	}


	ImageAndExposure* getImage_internal(int id, int unused) {
		MinimalImageB* minimg = getImageRaw_internal(id, 0);
		ImageAndExposure* ret2 = undistort->undistort<unsigned char>(
				minimg,
				(exposures.size() == 0 ? 1.0f : exposures[id]),
				(timestamps.size() == 0 ? 0.0 : timestamps[id]));
		delete minimg;
		return ret2;
	}

	inline void loadTimestamps() {
		std::ifstream tr;
		std::string timesFile = path.substr(0,path.find_last_of('/')) + "/times.txt";
		tr.open(timesFile.c_str());
		while(!tr.eof() && tr.good()) {
			std::string line;
			char buf[1000];
			tr.getline(buf, 1000);

			int id;
			double stamp;
			float exposure = 0;

			if(3 == sscanf(buf, "%d %lf %f", &id, &stamp, &exposure)) {
				timestamps.push_back(stamp);
				exposures.push_back(exposure);
			}

			else if(2 == sscanf(buf, "%d %lf", &id, &stamp)) {
				timestamps.push_back(stamp);
				exposures.push_back(exposure);
			}
		}
		tr.close();

		// check if exposures are correct, (possibly skip)
		bool exposuresGood = ((int)exposures.size()==(int)getNumImages()) ;
		for(int i=0;i<(int)exposures.size();i++) {
			if(exposures[i] == 0) {
				// fix!
				float sum=0,num=0;
				if(i>0 && exposures[i-1] > 0) {sum += exposures[i-1]; num++;}
				if(i+1<(int)exposures.size() && exposures[i+1] > 0) {sum += exposures[i+1]; num++;}

				if(num>0)
					exposures[i] = sum/num;
			}

			if(exposures[i] == 0) exposuresGood=false;
		}


		if((int)getNumImages() != (int)timestamps.size()) {
			printf("set timestamps and exposures to zero!\n");
			exposures.clear();
			timestamps.clear();
		}

		if((int)getNumImages() != (int)exposures.size() || !exposuresGood) {
			printf("set EXPOSURES to zero!\n");
			exposures.clear();
		}

		printf("got %d images and %d timestamps and %d exposures.!\n", (int)getNumImages(), (int)timestamps.size(), (int)exposures.size());
	}

};

