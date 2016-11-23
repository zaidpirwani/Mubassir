/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "aruco.h"
using namespace cv;
using namespace aruco;

#define CornerMarkers 4
#define NumberOfMarkerMaps 2+CornerMarkers


#include <raspicam/raspicam_cv.h>
raspicam::RaspiCam_Cv PiVideoCapturer;

string TheMarkerMapConfigFile[NumberOfMarkerMaps];
bool The3DInfoAvailable = false;
float TheMarkerSize[NumberOfMarkerMaps] = {-1,-1};
VideoCapture TheVideoCapturer;
Mat TheInputImage, TheInputImageCopy;
CameraParameters TheCameraParameters;
MarkerMap TheMarkerMapConfig[NumberOfMarkerMaps];
MarkerDetector TheMarkerDetector[NumberOfMarkerMaps];
MarkerMapPoseTracker TheMSPoseTracker[NumberOfMarkerMaps];
void cvTackBarEvents(int pos, void *);
double ThresParam1, ThresParam2;
int iThresParam1, iThresParam2;
int waitTime = 10;
std::map<int,cv::Mat> frame_pose_map[NumberOfMarkerMaps];//set of poses and the frames they were detected

class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};

void savePCDFile(string fpath,const aruco::MarkerMap &ms,const std::map<int,cv::Mat> frame_pose_map)throw(std::exception) ;
void savePosesToFile(string filename,const std::map<int,cv::Mat> &fmp);


/************************************
 *
 *
 *
 *
 ************************************/



void processKey(char k) {
    switch (k) {
    case 's':
        if (waitTime == 0)
            waitTime = 10;
        else
            waitTime = 0;
        break;

    }
}

/************************************
 *
 *
 *
 *
 ************************************/
int main(int argc, char **argv) {
    try {
        CmdLineParser cml(argc,argv);
        if (argc < 3+1|| cml["-h"]) {
            cerr << "Invalid number of arguments" << endl;
            cerr << "Usage: (in.avi|live) marksetconfig1.yml  marksetconfig2.yml  cornerMarker1.yml  cornerMarker2.yml  cornerMarker3.yml  cornerMarker4.yml  [optional_arguments] \n\t[-c camera_intrinsics.yml] \n\t[-s[n=1,2] marker_size] \n\t[-sC corner_marker_size] \n\t[-pcd[n=1,2] out_pcd_file_with_camera_poses] \n\t[-poses[n=1,2] out_file_with_poses] \n\t[-corner <corner_refinement_method> (0: LINES(default),1 SUBPIX) ][-h]" << endl;
            return false;
        }
		int z=0;
		
		for(z=0; z<NumberOfMarkerMaps; z++){
			switch(z){
				case 0:
					TheMarkerMapConfig[z].readFromFile(argv[2]);
					TheMarkerMapConfigFile[z] = argv[2];
					TheMarkerSize[z] = stof( cml("-s1","1")); break;
				case 1:
					TheMarkerMapConfig[z].readFromFile(argv[3]);
					TheMarkerMapConfigFile[z] = argv[3];
					TheMarkerSize[z] = stof( cml("-s2","1")); break;
				case 2: case 3: case 4: case 5:
					TheMarkerMapConfig[z].readFromFile(argv[z+2]);
					TheMarkerMapConfigFile[z] = argv[z+2];
					TheMarkerSize[z] = stof( cml("-sC","1")); break;
				default:
					throw std::runtime_error("Error reading MarkerConfig Files"); break;
			}
		}
        // read from camera or from  file
        if (string(argv[1])== "live") {
            PiVideoCapturer.set ( CV_CAP_PROP_FORMAT, CV_8UC1 );
			PiVideoCapturer.set ( CV_CAP_PROP_FRAME_WIDTH,  800 ); // 800 
			PiVideoCapturer.set ( CV_CAP_PROP_FRAME_HEIGHT, 600 );  // 600 
            PiVideoCapturer.open();
            //TheVideoCapturer.open(0);
        } else TheVideoCapturer.open(argv[1]);
        // check video is open
        if (!PiVideoCapturer.isOpened())  throw std::runtime_error("Could not open video");

        // read first image to get the dimensions
        //TheVideoCapturer >> TheInputImage;
        PiVideoCapturer.grab();
		PiVideoCapturer.retrieve(TheInputImage);

        // read camera parameters if passed
        if (cml["-c"]) {
            TheCameraParameters.readFromXMLFile(cml("-c"));
            TheCameraParameters.resize(TheInputImage.size());
        }
		z=0;
        //prepare the detector
        string dict=TheMarkerMapConfig[z].getDictionary();//see if the dictrionary is already indicated in the configuration file. It should!
        if(dict.empty()) dict="ARUCO";
		for(z=0; z<NumberOfMarkerMaps; z++){
			TheMarkerDetector[z].setDictionary(dict);///DO NOT FORGET THAT!!! Otherwise, the ARUCO dictionary will be used by default!
			if (stoi(cml("-corner","0"))==0)
				TheMarkerDetector[z].setCornerRefinementMethod(MarkerDetector::LINES);
			else{
				MarkerDetector::Params params=TheMarkerDetector[z].getParams();
				params._cornerMethod=MarkerDetector::SUBPIX;
				params._subpix_wsize= (15./2000.)*float(TheInputImage.cols) ;//search corner subpix in a 5x5 widow area
				TheMarkerDetector[z].setParams(params);
			}
		
			//prepare the pose tracker if possible
			//if the camera parameers are avaiable, and the markerset can be expressed in meters, then go

			if ( TheMarkerMapConfig[z].isExpressedInPixels() && TheMarkerSize[z]>0)
				TheMarkerMapConfig[z]=TheMarkerMapConfig[z].convertToMeters(TheMarkerSize[z]);
	cout<<"TheCameraParameters.isValid()="<<TheCameraParameters.isValid()<<" "<<TheMarkerMapConfig[z].isExpressedInMeters()<<endl;
			if (TheCameraParameters.isValid() && TheMarkerMapConfig[z].isExpressedInMeters()  )
				TheMSPoseTracker[z].setParams(TheCameraParameters,TheMarkerMapConfig[z]);

			TheMarkerDetector[z].getThresholdParams(ThresParam1, ThresParam2);
		}


        // Create gui

        //cv::namedWindow("thres", 1);
        cv::namedWindow("in", 1);

        iThresParam1 = ThresParam1;
        iThresParam2 = ThresParam2;
        cv::createTrackbar("ThresParam1", "in", &iThresParam1, 13, cvTackBarEvents);
        cv::createTrackbar("ThresParam2", "in", &iThresParam2, 13, cvTackBarEvents);
        char key = 0;
        int index = 0;
        // capture until press ESC or until the end of the video
        cout<<"Press 's' to start/stop video"<<endl;
        do {
            PiVideoCapturer.retrieve(TheInputImage);
            TheInputImage.copyTo(TheInputImageCopy);
            index++; // number of images captured
            // Detection of the board
			for(z=0; z<NumberOfMarkerMaps; z++){
				vector<aruco::Marker> detected_markers=TheMarkerDetector[z].detect(TheInputImage);
				//print the markers detected that belongs to the markerset
				 for(auto idx:TheMarkerMapConfig[z].getIndices(detected_markers))
					 detected_markers[idx].draw(TheInputImageCopy, Scalar(0, 0, 255), 2);
				 //detect 3d info if possible
				 if (TheMSPoseTracker[z].isValid()){
					  if ( TheMSPoseTracker[z].estimatePose(detected_markers)){
						 aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopy,  TheCameraParameters,TheMSPoseTracker[z].getRvec(),TheMSPoseTracker[z].getTvec(),TheMarkerMapConfig[z][0].getMarkerSize()*2);
						 frame_pose_map[z].insert(make_pair(index,TheMSPoseTracker[z].getRTMatrix() ));
						 cout<<"pose "<< z <<" rt="<<TheMSPoseTracker[z].getRvec()<<" "<<TheMSPoseTracker[z].getTvec()<<endl;
					  }
				}
			}
            // show input with augmented information and  the thresholded image
            cv::imshow("in", TheInputImageCopy);
            //cv::imshow("thres",TheMarkerDetector.getThresholdedImage());

            key = cv::waitKey(waitTime); // wait for key to be pressed
            processKey(key);

        } while (key != 27 && PiVideoCapturer.grab() );



		for(z=0; z<(NumberOfMarkerMaps-CornerMarkers); z++){
			//save a beatiful pcd file (pcl library) showing the results (you can use pcl_viewer to see it)
			if (cml["-pcd1"] || cml["-pcd2"]){
				switch(z){
					case 0: savePCDFile(cml("-pcd1"),TheMarkerMapConfig[z],frame_pose_map[z]); break;
					case 1: savePCDFile(cml("-pcd2"),TheMarkerMapConfig[z],frame_pose_map[z]); break;
					throw std::runtime_error("Error saving PCD File"); break;
				}
			}
			//save the poses to a file in tum rgbd data format
			if (cml["-poses1"] || cml["-poses2"]){
				switch(z){
					case 0: savePosesToFile(cml("-poses1"),frame_pose_map[z]); break;
					case 1: savePosesToFile(cml("-poses2"),frame_pose_map[z]); break;
					throw std::runtime_error("Error saving POSES File"); break;
				}
			}
		}

    } catch (std::exception &ex)

    {
        cout << "Exception :" << ex.what() << endl;
    }
}
/************************************
 *
 *
 *
 *
 ************************************/

void cvTackBarEvents(int pos, void *) {
    (void)(pos);
    if (iThresParam1 < 3)
        iThresParam1 = 3;
    if (iThresParam1 % 2 != 1)
        iThresParam1++;
    if (ThresParam2 < 1)
        ThresParam2 = 1;
    ThresParam1 = iThresParam1;
    ThresParam2 = iThresParam2;
	int z=0;
	for(z=0; z<NumberOfMarkerMaps; z++)
		TheMarkerDetector[z].setThresholdParams(ThresParam1, ThresParam2);




    //detect, print, get pose, and print

	/*
   //detect
    vector<aruco::Marker> detected_markers=TheMarkerDetector.detect(TheInputImage);
    //print the markers detected that belongs to the markerset
     for(auto idx:TheMarkerMapConfig.getIndices(detected_markers))
         detected_markers[idx].draw(TheInputImageCopy, Scalar(0, 0, 255), 2);
     //detect 3d info if possible
     if (TheMSPoseTracker.isValid()){
         TheMSPoseTracker.estimatePose(detected_markers);
         aruco::CvDrawingUtils::draw3dAxis(TheInputImageCopy,  TheCameraParameters,TheMSPoseTracker.getRvec(),TheMSPoseTracker.getTvec(),TheMarkerMapConfig[0].getMarkerSize()*2);
    }
	*/


    cv::imshow("in", TheInputImageCopy);
    //cv::imshow("thres",TheMarkerDetector.getThresholdedImage());
}

inline float SIGN(float x) {return (x >= 0.0f) ? +1.0f : -1.0f;}
inline float NORM(float a, float b, float c, float d) {return sqrt(a * a + b * b + c * c + d * d);}


void  getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,float &qx,float &qy,float &qz,float &qw,float &tx,float &ty,float &tz){
    //get the 3d part of matrix and get quaternion
    assert(M_in.total()==16);
    cv::Mat M;M_in.convertTo(M,CV_32F);
     //use now eigen
    float r11=M.at<float>(0,0);
    float r12=M.at<float>(0,1);
    float r13=M.at<float>(0,2);
    float r21=M.at<float>(1,0);
    float r22=M.at<float>(1,1);
    float r23=M.at<float>(1,2);
    float r31=M.at<float>(2,0);
    float r32=M.at<float>(2,1);
    float r33=M.at<float>(2,2);



    double  q0 = ( r11 + r22 + r33 + 1.0f) / 4.0f;
    double  q1 = ( r11 - r22 - r33 + 1.0f) / 4.0f;
    double     q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
    double     q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
    if(q0 < 0.0f) q0 = 0.0f;
    if(q1 < 0.0f) q1 = 0.0f;
    if(q2 < 0.0f) q2 = 0.0f;
    if(q3 < 0.0f) q3 = 0.0f;
    q0 = sqrt(q0);
    q1 = sqrt(q1);
    q2 = sqrt(q2);
    q3 = sqrt(q3);
    if(q0 >= q1 && q0 >= q2 && q0 >= q3) {
        q0 *= +1.0f;
        q1 *= SIGN(r32 - r23);
        q2 *= SIGN(r13 - r31);
        q3 *= SIGN(r21 - r12);
    } else if(q1 >= q0 && q1 >= q2 && q1 >= q3) {
        q0 *= SIGN(r32 - r23);
        q1 *= +1.0f;
        q2 *= SIGN(r21 + r12);
        q3 *= SIGN(r13 + r31);
    } else if(q2 >= q0 && q2 >= q1 && q2 >= q3) {
        q0 *= SIGN(r13 - r31);
        q1 *= SIGN(r21 + r12);
        q2 *= +1.0f;
        q3 *= SIGN(r32 + r23);
    } else if(q3 >= q0 && q3 >= q1 && q3 >= q2) {
        q0 *= SIGN(r21 - r12);
        q1 *= SIGN(r31 + r13);
        q2 *= SIGN(r32 + r23);
        q3 *= +1.0f;
    } else {
        cerr<<"Coding error"<<endl;
    }
    double r = NORM(q0, q1, q2, q3);
    qx =q0/ r;
    qy =q1/ r;
    qz =q2/ r;
    qw =q3/ r;



    tx=M.at<float>(0,3);
    ty=M.at<float>(1,3);
    tz=M.at<float>(2,3);


}
void savePosesToFile(string filename,const std::map<int,cv::Mat> &fmp){
    std::ofstream file(filename);
    float qx,qy,qz,qw,tx,ty,tz;
    for(auto frame:fmp){
        if ( !frame.second.empty()){
                getQuaternionAndTranslationfromMatrix44(frame.second,qx,qy,qz,qw,tx,ty,tz);
                file<<frame.first<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<endl;
        }
    }
}
