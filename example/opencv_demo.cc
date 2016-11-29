	/* (C) 2013-2016, The Regents of The University of Michigan
	All rights reserved.

	This software was developed in the APRIL Robotics Lab under the
	direction of Edwin Olson, ebolson@umich.edu. This software may be
	available under alternative licensing terms; contact the address
	above.

	   BSD
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this
	   list of conditions and the following disclaimer.
	2. Redistributions in binary form must reproduce the above copyright notice,
	   this list of conditions and the following disclaimer in the documentation
	   and/or other materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

	The views and conclusions contained in the software and documentation are those
	of the authors and should not be interpreted as representing official policies,
	either expressed or implied, of the FreeBSD Project.
	 */


#include <visp3/gui/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDot2.h>
#include <visp/vpImageIo.h>
#include <visp/vpPixelMeterConversion.h>

#include <visp/vpPose.h>
#include <visp/vpPoint.h>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "flycapture/FlyCapture2.h"
#include <iostream>

#include "opencv2/opencv.hpp"

#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "tag16h5.h"

using namespace std;
using namespace cv;
using namespace FlyCapture2;

	int main(int argc, char *argv[])
	{
		cout<<"OpenCV Version used:"<<CV_MAJOR_VERSION<<"."<<CV_MINOR_VERSION<<"."<<CV_SUBMINOR_VERSION<<endl;

		cout<<endl<<"Creating getopt"<<endl;
		getopt_t *getopt = getopt_create();

		getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
		getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
		getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
		getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
		getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
		getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
		getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
		cout<<"\nDecimation Changed\n";
		getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
		getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
		getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
		getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

		if (!getopt_parse(getopt, argc, argv, 1) ||
				getopt_get_bool(getopt, "help")) {
			printf("Usage: %s [options]\n", argv[0]);
			getopt_do_usage(getopt);
			exit(0);
		}

		//////////////////////FlyCapture
		Error error;
		Camera camera;
		CameraInfo camInfo;

		// Connect the camera
		error = camera.Connect( 0 ); // Probably I need to change here if I have 2 cameras and I want to connect to a specific one.
		if ( error != PGRERROR_OK )
		{
			cout << "Failed to connect to camera" << endl;
			return false;
		}


		// Get the camera info and print it out
		error = camera.GetCameraInfo( &camInfo );

//		cout << "GetCameraInfo:" << error << endl;
		if ( error != PGRERROR_OK )
		{
			cout << "Failed to get camera info from camera" << endl;
			return false;
		}
		cout << "Starting to capture with the camera:" << endl;
		cout << "["<< camInfo.vendorName << " "
				<< camInfo.modelName << " "
				<< camInfo.serialNumber << "]" << std::endl;
		error = camera.StartCapture();

		/////////////////////////////////////
		// Initialize webcam laptop
		//	VideoCapture cap(0);
		//	if (!cap.isOpened()) {
		//		cerr << "Couldn't open video capture device" << endl;
		//		return -1;
		//	}
		//
		//	if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
		//	{
		//		std::cout << "Bandwidth exceeded" << std::endl;
		//		return false;
		//	}
		//	else if ( error != PGRERROR_OK )
		//	{
		//		std::cout << "Failed to start image capture" << std::endl;
		//		return false;
		//	}
		//////////////////////////////////////

		// capture loop
		char key = 0;
		// Initialize tag detector with options
		apriltag_family_t *tf = NULL;
		const char *famname = getopt_get_string(getopt, "family");
		cout<<"FamilyName:"<<*famname<<endl;
		cout << "Creating apriltag_detector..." << endl;
		apriltag_detector_t *td = apriltag_detector_create();
		while(key != 'q')
		{
			// Get the image
			Image rawImage;
			Error error = camera.RetrieveBuffer( &rawImage );
			if ( error != PGRERROR_OK )
			{
				std::cout << "capture error" << std::endl;
				continue;
			}

			// AprilTAG
			if (!strcmp(famname, "tag36h11"))
				tf = tag36h11_create();
			else if (!strcmp(famname, "tag36h10"))
				tf = tag36h10_create();
			else if (!strcmp(famname, "tag36artoolkit"))
				tf = tag36artoolkit_create();
			else if (!strcmp(famname, "tag25h9"))
				tf = tag25h9_create();
			else if (!strcmp(famname, "tag25h7"))
				tf = tag25h7_create();
			else if (!strcmp(famname, "tag16h5"))
				tf = tag16h5_create();
			else {
				printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
				exit(-1);
			}
			tf->black_border = getopt_get_int(getopt, "border");

			// apriltag_detector_t *td = apriltag_detector_create();
			apriltag_detector_add_family(td, tf);
			td->quad_decimate = getopt_get_double(getopt, "decimate");
			td->quad_sigma = getopt_get_double(getopt, "blur");
			td->nthreads = getopt_get_int(getopt, "threads");
			td->debug = getopt_get_bool(getopt, "debug");
			td->refine_edges = getopt_get_bool(getopt, "refine-edges");
			td->refine_decode = getopt_get_bool(getopt, "refine-decode");
			td->refine_pose = getopt_get_bool(getopt, "refine-pose");

			Mat frame, gray;
			while (!key) {
				// Get the image
				Image rawImage;
				Error error = camera.RetrieveBuffer( &rawImage );
				if ( error != PGRERROR_OK )
				{
					std::cout << "capture error" << std::endl;
					continue;
				}

				// convert to rgb
				Image rgbImage;
				rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );

				// convert to OpenCV Mat
				unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
				cv::Mat image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

				// cap >> frame;

				frame = image;
				cvtColor(frame, gray, COLOR_BGR2GRAY);

				// Make an image_u8_t header for the Mat data
				image_u8_t im = { .width = gray.cols,
						.height = gray.rows,
						.stride = gray.cols,
						.buf = gray.data
				};

				zarray_t *detections = apriltag_detector_detect(td, &im);
				// cout << detections->data;
				cout << zarray_size(detections) << " tags detected" << endl;

				// Draw detection outlines
				for (int i = 0; i < zarray_size(detections); i++) {
					apriltag_detection_t *det;
					zarray_get(detections, i, &det);
					line(frame, Point(det->p[0][0], det->p[0][1]),
							Point(det->p[1][0], det->p[1][1]),
							Scalar(0, 0xff, 0), 2);
					line(frame, Point(det->p[0][0], det->p[0][1]),
							Point(det->p[3][0], det->p[3][1]),
							Scalar(0, 0, 0xff), 2);
					line(frame, Point(det->p[1][0], det->p[1][1]),
							Point(det->p[2][0], det->p[2][1]),
							Scalar(0xff, 0, 0), 2);
					line(frame, Point(det->p[2][0], det->p[2][1]),
							Point(det->p[3][0], det->p[3][1]),
							Scalar(0xff, 0, 0), 2);
					cout<<"["<<det->c[0]<<","<<det->c[1]<<"]"<<endl;
//                                        cout << "( " << det->p[0][0] << " )";
                                        cout << "Points: " << endl;
                                        cout << "( " << det->p[0][0] << " )";
                                        cout << "( " << det->p[0][1] << " )";
                                        stringstream ss;
					ss << det->id;
					//            cout << det->H<<endl;
					//				*det->H H[0][]
					cout << "-------------" <<endl;
					cout << "det->H->data" <<endl;
					for (int j = 0; j < det->H->ncols; ++j) {
						cout << "["<<det->H->data[j+2*j]<<","<< det->H->data[(j+1)+2*j] << ","<< det->H->data[(j+2)+2*j]<<"]" << endl;
					}
					cout << "-------------" <<endl;
					cout << det->H->data[0]<<endl;
					cout << det->H->ncols  <<endl;
					cout << det->H->nrows  <<endl;
					//				cout << det->H  <<endl;
					//matd_t *matd = matd_create(3,3);
					matd_t * ciao = new matd_t;
					double camera_matrix [9] = {687.216761, 0.000000, 1111.575057, 0.000000, 673.787664, 747.109306, 0.000000, 0.000000, 1.000000};
//					int intero = homography_to_pose(det->H,camera_matrix[1],camera_matrix[5],camera_matrix[3],camera_matrix[6])->ncols;
					//				cout << doppio << endl;
					// This is the text which is put on the image
					String text = ss.str();
					int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
					double fontscale = 1.0;
					int baseline;
					Size textsize = getTextSize(text, fontface, fontscale, 2,
							&baseline);
					putText(frame, text, Point(det->c[0]-textsize.width/2,
							det->c[1]+textsize.height/2),
							fontface, fontscale, Scalar(0xff, 0x99, 0), 2);


                                        std::vector<vpPoint> point(4);
                                        std::vector<vpImagePoint> corners(4);
                                        //Set points coordinates

                                        double dist_point_=0.086;

                                        point[0].setWorldCoordinates(-dist_point_,-dist_point_, 0) ;
                                        point[1].setWorldCoordinates(-dist_point_,dist_point_, 0) ;
                                        point[2].setWorldCoordinates(dist_point_,dist_point_, 0) ;
                                        point[3].setWorldCoordinates(dist_point_,-dist_point_,0) ;
//                                        cout << td->quad_decimate << endl ;
                                        vpHomogeneousMatrix cMo;

//                                        <px>684.1393341905</px>
//                                        <py>682.7092957507</py>
                                        double px = 684.1393341905;
                                        double py = 682.7092957507;
                                        double u0 = 1920/td->quad_decimate;
                                        double v0 = 1080/td->quad_decimate;
                                        // Create a camera parameter container
                                        vpCameraParameters cam;
                                        // Camera initialization with a perspective projection without distortion model
                                        cam.initPersProjWithoutDistortion(px,py,u0,v0);

                                        // It is also possible to print the current camera parameters
                                        std::cout << cam << std::endl;

//                                        vpBlobsTargetTracker::computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &corners, const vpCameraParameters &cam, bool &init, vpHomogeneousMatrix &cMo)


					// cout << det->p[1][0] << endl;
				}
				zarray_destroy(detections);

				imshow("Tag Detections", frame);
				if (waitKey(30) >= 0)
					break;
			}

			// cv::imshow("image", image);
			key = cv::waitKey(30);
		}

		//	Destruction of the apriltag_detector
		apriltag_detector_destroy(td);
		if (!strcmp(famname, "tag36h11"))
			tag36h11_destroy(tf);
		else if (!strcmp(famname, "tag36h10"))
			tag36h10_destroy(tf);
		else if (!strcmp(famname, "tag36artoolkit"))
			tag36artoolkit_destroy(tf);
		else if (!strcmp(famname, "tag25h9"))
			tag25h9_destroy(tf);
		else if (!strcmp(famname, "tag25h7"))
			tag25h7_destroy(tf);
		getopt_destroy(getopt);

		return 0;
	}
