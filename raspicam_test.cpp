#include <string>
#include <iostream>
#include <ctime>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/highgui/highgui.hpp>
#include <raspicam/raspicam_cv.h>

using namespace std;
using namespace cv;
using namespace aruco;

int main ( int argc,char **argv ) {
	raspicam::RaspiCam_Cv Camera;
	Camera.set ( CV_CAP_PROP_FORMAT, CV_8UC1 );
	Camera.set ( CV_CAP_PROP_FRAME_WIDTH,  800 ); //for simplicity, camera watches 8 tile space in width
	Camera.set ( CV_CAP_PROP_FRAME_HEIGHT, 600 ); //for simplicity, camera watches 6 tile space in height

	cout<<"Opening Camera..."<<endl;
	if (!Camera.open()) { cerr<<"Error opening the camera"<<endl; return -1; }

	Mat image;

	aruco::CameraParameters CamParam;
	CamParam.readFromXMLFile("picam2.yml"); //must be in the build library, along the executable

	//float MarkerSize = 0.069; //69mm
	float MarkerSize = 0.101; //127mm

	MarkerDetector MDetector;
        MDetector.setThresholdParams(7, 7);
        MDetector.setThresholdParamRange(2, 0);
	//MDetector.setDictionary("ARUCO_MIP_36h12",0.f);
        MDetector.setDictionary("ARUCO",0.f);

	while(1){
		Camera.grab();
		Camera.retrieve(image);
		vector< Marker >  Markers=MDetector.detect(image, CamParam, MarkerSize);
		
		// for each marker, draw info and its boundaries in the image
		for (unsigned int i = 0; i < Markers.size(); i++) {
			cout << "ID: " << Markers[i].id << ", [X,Y]=" << Markers[i].getCenter() << ", [Rvec]=" << Markers[i].Rvec << ", [Tvec]=" << Markers[i].Tvec << endl; //Markers[i]
			Markers[i].draw(image, Scalar(0, 0, 255), 2);
		}
		cout << endl; // blank line
		// draw a 3d cube in each marker if there is 3d info
		//if (CamParam.isValid() && MarkerSize != -1)
		//	for (unsigned int i = 0; i < Markers.size(); i++) {
		//		CvDrawingUtils::draw3dCube(image, Markers[i], CamParam);
		//	}
		// show input with augmented information
		imshow("in",image);
		waitKey(10);
	}

	cout<<"Stop camera..."<<endl;
	Camera.release();
	
	//save image 
	//imwrite("raspicam_cv_image.jpg",image);
	//cout<<"Image saved at raspicam_cv_image.jpg"<<endl;
}
