/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#define PI 3.14159265
using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void RGB2r_gray(const cv::Mat& src, cv::Mat& dst);
void RGB2gb_gray(const cv::Mat& src, cv::Mat& dst);

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);
	cv::Mat report_twc_last= cv::Mat();
	cv::Mat report_twc_curr= cv::Mat();
	double  report_timestamp_last = -1;
	double  report_timestamp_curr = -1;
	cv::Mat report_twc_last2 = cv::Mat();
	cv::Mat report_twc_curr2 = cv::Mat();
	double  report_timestamp_last2 = -1;
	double  report_timestamp_curr2 = -1;






	//This should handle the rotational offset for the FOVs
	float yaw = 0.f *  PI/ 180.0f;
	//std::cout << "Initailizing-2" << endl;


	//This should handle the rotational offset for the FOVs

/*
	cv::Mat twc_offset1 = (cv::Mat_<float>(4, 4) <<
		cos(-angle), 0, sin(-angle), 0,
		0, 1, 0, 0,
		-sin(-angle), 0, cos(-angle), 0,
		0, 0, 0, 1);

	cv::Mat twc_offset2 = (cv::Mat_<float>(4, 4) <<
		cos(angle), 0, sin(angle), 0,
		0, 1, 0, 0,
		-sin(angle), 0, cos(angle), 0,
		0, 0, 0, 1);
		*/
	cv::Mat twc_offset2 = (cv::Mat_<float>(4, 4) <<
		cos(-yaw), -sin(-yaw), 0, 0,
		sin(-yaw), cos(-yaw), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);


	cv::Mat twc_offset1 = (cv::Mat_<float>(4, 4) <<
		cos(yaw), -sin(yaw), 0, 0,
		sin(yaw), cos(yaw), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);





    int nImages = vstrImageFilenames.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM2(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,false);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }


		// Main loop
		cv::Mat imLeft, imRight;



			RGB2gb_gray(im, imRight);

			RGB2r_gray(im, imLeft);

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(imRight,tframe, report_twc_last2, report_timestamp_last2, report_twc_curr2, report_timestamp_curr2, twc_offset1);

		//slam 2 last frame
		report_twc_last2 = report_twc_curr2;
		report_timestamp_last2 = report_timestamp_curr2;
		//slam 1 current frame
		report_twc_curr = SLAM.report_twc_curr;
		report_timestamp_curr = SLAM.report_timestamp_curr;;


		//std::cout << "1. Last Frame Time Stamp: " << report_timestamp_last << endl;
		//std::cout << "1. Last Frame TWC: " << report_twc_last << endl << endl;

		//std::cout << "1. Curr Frame Time Stamp: " << report_timestamp_curr << endl;
		//std::cout << "1. Curr Frame TWC: " << report_twc_curr << endl << endl;

//		SLAM2.TrackMonocular(imLeft, tframe);


  	SLAM2.TrackMonocular(imRight, tframe, report_twc_last, report_timestamp_last, report_twc_curr, report_timestamp_curr, twc_offset2);
		//slam 1 last frame
	    report_twc_last = report_twc_curr;
		report_timestamp_last = report_timestamp_curr;
		//slam 2 current frame
		report_twc_curr2 = SLAM2.report_twc_curr;
		report_timestamp_curr2 = SLAM2.report_timestamp_curr;;

	

		//std::cout << "2. Last Frame Time Stamp: " << report_timestamp_last2 << endl;
		//std::cout << "2. Last Frame TWC: " << report_twc_last2 << endl << endl;

		//std::cout << "2. Curr Frame Time Stamp: " << report_timestamp_curr2 << endl;
		//std::cout << "2. Curr Frame TWC: " << report_twc_curr2 << endl << endl;






        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();
	// Stop all threads
	SLAM2.Shutdown();
    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
   SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
	SLAM.SaveMap("PointMap.txt");
	SLAM2.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory2.txt");
	SLAM2.SaveMap("PointMap2.txt");

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

void RGB2r_gray(const cv::Mat& src, cv::Mat& dst)
{
	CV_Assert(src.type() == CV_8UC3);
	int rows = src.rows, cols = src.cols;

	dst.create(src.size(), CV_8UC1);

	if (src.isContinuous() && dst.isContinuous())
	{
		cols = rows * cols;
		rows = 1;
	}

	for (int row = 0; row < rows; row++)
	{
		const uchar* src_ptr = src.ptr<uchar>(row);
		uchar* dst_ptr = dst.ptr<uchar>(row);

		for (int col = 0; col < cols; col++)
		{
			dst_ptr[col] = (uchar)(src_ptr[2]);
			src_ptr += 3;
		}
	}
}


void RGB2gb_gray(const cv::Mat& src, cv::Mat& dst)
{
	CV_Assert(src.type() == CV_8UC3);
	int rows = src.rows, cols = src.cols;

	dst.create(src.size(), CV_8UC1);

	if (src.isContinuous() && dst.isContinuous())
	{
		cols = rows * cols;
		rows = 1;
	}

	for (int row = 0; row < rows; row++)
	{
		const uchar* src_ptr = src.ptr<uchar>(row);
		uchar* dst_ptr = dst.ptr<uchar>(row);

		for (int col = 0; col < cols; col++)
		{
			dst_ptr[col] = (uchar)(src_ptr[1] * 0.8374f + src_ptr[0] * 0.1626f);
			src_ptr += 3;
		}
	}
}