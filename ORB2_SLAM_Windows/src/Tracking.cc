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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    std::cout << endl << "Camera Parameters: " << endl;
    std::cout << "- fx: " << fx << endl;
    std::cout << "- fy: " << fy << endl;
    std::cout << "- cx: " << cx << endl;
    std::cout << "- cy: " << cy << endl;
    std::cout << "- k1: " << DistCoef.at<float>(0) << endl;
    std::cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        std::cout << "- k3: " << DistCoef.at<float>(4) << endl;
    std::cout << "- p1: " << DistCoef.at<float>(2) << endl;
    std::cout << "- p2: " << DistCoef.at<float>(3) << endl;
    std::cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        std::cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        std::cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    std::cout << endl  << "ORB Extractor Parameters: " << endl;
    std::cout << "- Number of Features: " << nFeatures << endl;
    std::cout << "- Scale Levels: " << nLevels << endl;
    std::cout << "- Scale Factor: " << fScaleFactor << endl;
    std::cout << "- Initial Fast Threshold: " << fIniThFAST << endl;

	if (sensor==System::MONOCULARMULTICOLORFILTER)
		mpIniORBextractor = new ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
		mpIniORBextractorB = new ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
		mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

	std::cout << endl << "ORB Extractor Parameters: " << endl;
	std::cout << "- Number of Features: " << nFeatures << endl;
	std::cout << "- Scale Levels: " << nLevels << endl;
	std::cout << "- Scale Factor: " << fScaleFactor << endl;
	std::cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
	std::cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
    std::cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD || sensor == System::STEREOMULTI)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        std::cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}


void Tracking::RGB2r_gray(const cv::Mat& src, cv::Mat& dst)
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


void Tracking::RGB2gb_gray(const cv::Mat& src, cv::Mat& dst)
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
			dst_ptr[col] = (uchar)(src_ptr[1] * 0.8374f + src_ptr[0] * 0.1626);
			src_ptr += 3;
		}
	}
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageStereoMulti(const cv::Mat &imMulti, const double &timestamp)
{
	mImGray = imMulti;

	if (mImGray.channels() == 3)
	{
		if (mbRGB)
		{
			cvtColor(mImGray, mImGray, CV_RGB2GRAY);
		}
		else
		{
			cvtColor(mImGray, mImGray, CV_BGR2GRAY);
		}
	}
	else if (mImGray.channels() == 4)
	{
		if (mbRGB)
		{
			cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
		}
		else
		{
			cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
		}
	}

	mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth, true);

	Track();

	return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}





cv::Mat Tracking::GrabImageStereoMultiColor_Filter(const cv::Mat &imMulti, const double &timestamp)
{

	Tracking::RGB2gb_gray(imMulti, mImGray);
	Tracking::RGB2r_gray(imMulti, mImGrayB);


	if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET || mState == CAM_A_NOT_INITIALIZED)
	{
		mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
	    //mCurrentFrame_alt = mCurrentFrame;
		mCurrentFrame_alt = Frame(mImGrayB, timestamp, mpIniORBextractorB, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
	}
	else
	{
		mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
		//mCurrentFrame_alt = mCurrentFrame;
		mCurrentFrame_alt = Frame(mImGrayB, timestamp, mpORBextractorRight, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
	}
	//Track();
	Track_Mono_Color_Filtered();


	return mCurrentFrame.mTcw.clone();
}



cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;


    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)

        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                bOK = Relocalization();
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                std::cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}



void Tracking::Track_Mono_Color_Filtered()
{
	if (mState == NO_IMAGES_YET)
	{
		mState = NOT_INITIALIZED;
	}

	mLastProcessedState = mState;

	// Get Map Mutex -> Map cannot be changed
	unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

	if (mState == NOT_INITIALIZED)
	{
	
		CAMERA_A_MonocularInitialization();

		mpFrameDrawer->Update(this);

		if (mState != OK)
			return;
	}
	else
	{
		// System is initialized. Track Frame.
		bool bOK;

		std::cout << "Starting to TRACK!!!!!!!!" << endl;

		std::cout << "MBONLYTRACKING: " << mbOnlyTracking <<  endl;
		// Initial camera pose estimation using motion model or relocalization (if tracking is lost)
		if (!mbOnlyTracking)
		{
			// Local Mapping is activated. This is the normal behaviour, unless
			// you explicitly activate the "only tracking" mode.
			std::cout << "mstate: " << mState<< endl;

			if (mState == OK)
			{
				// Local Mapping might have changed some MapPoints tracked in last frame
				CheckReplacedInLastFrame();

				if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2)
				{
					std::cout << "RefTrack Results1: " << bOK << endl;

					bOK = TrackReferenceKeyFrameMulti();
				}
				else
				{
					std::cout << "Motion MOdel Track " << endl;

					//bOK = TrackWithMotionModel();
					bOK = TrackWithMotionModelMulti();
					if (!bOK)
						std::cout << "Motion MOdel Track Failed " << endl;

						bOK = TrackReferenceKeyFrameMulti();

						std::cout << "RefTrack Results2: " << bOK << endl;


				}
			}
			else
			{
				std::cout << "Relocalizing " << endl;

				bOK = Relocalization();
			}
		}
		else
		{
			// Localization Mode: Local Mapping is deactivated

			if (mState == LOST)
			{
				bOK = Relocalization();
			}
			else
			{
				if (!mbVO)
				{
					// In last frame we tracked enough MapPoints in the map

					if (!mVelocity.empty())
					{
						bOK = TrackWithMotionModelMulti();
					}
					else
					{
						bOK = TrackReferenceKeyFrameMulti();
					}
				}
				else
				{
					// In last frame we tracked mainly "visual odometry" points.

					// We compute two camera poses, one from motion model and one doing relocalization.
					// If relocalization is sucessfull we choose that solution, otherwise we retain
					// the "visual odometry" solution.

					bool bOKMM = false;
					bool bOKReloc = false;
					vector<MapPoint*> vpMPsMM, vpMPsMM_alt;
					vector<bool> vbOutMM, vbOutMM_alt;
					cv::Mat TcwMM, TcwMM_alt;
					if (!mVelocity.empty())
					{
						bOK = TrackWithMotionModelMulti();
						vpMPsMM = mCurrentFrame.mvpMapPoints;
						vbOutMM = mCurrentFrame.mvbOutlier;
						TcwMM = mCurrentFrame.mTcw.clone();
						vpMPsMM_alt = mCurrentFrame_alt.mvpMapPoints;
						vbOutMM_alt = mCurrentFrame_alt.mvbOutlier;
						TcwMM_alt = mCurrentFrame_alt.mTcw.clone();
					}
					bOKReloc = Relocalization();

					if (bOKMM && !bOKReloc)
					{
						mCurrentFrame.SetPose(TcwMM);
						mCurrentFrame.mvpMapPoints = vpMPsMM;
						mCurrentFrame.mvbOutlier = vbOutMM;

						mCurrentFrame_alt.SetPose(TcwMM_alt);
						mCurrentFrame_alt.mvpMapPoints = vpMPsMM_alt;
						mCurrentFrame_alt.mvbOutlier = vbOutMM_alt;

						if (mbVO)
						{
							for (int i = 0; i < mCurrentFrame.N; i++)
							{
								if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
								{
									mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
								}
							}
							for (int i = 0; i < mCurrentFrame_alt.N; i++)
							{
								if (mCurrentFrame_alt.mvpMapPoints[i] && !mCurrentFrame_alt.mvbOutlier[i])
								{
									mCurrentFrame_alt.mvpMapPoints[i]->IncreaseFound();
								}
							}
						}
					}
					else if (bOKReloc)
					{
						mbVO = false;
					}

					bOK = bOKReloc || bOKMM;
				}
			}
		}

		mCurrentFrame.mpReferenceKF = mpReferenceKF;
		mCurrentFrame_alt.mpReferenceKF = mpReferenceKF_alt;


		// If we have an initial estimation of the camera pose and matching. Track the local map.
		if (!mbOnlyTracking)
		{
			if (bOK)
				bOK = TrackLocalMap();
		}
		else
		{
			// mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
			// a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
			// the camera we will use the local map again.
			if (bOK && !mbVO)
				bOK = TrackLocalMap();
		}

		if (bOK)
			mState = OK;
		else
			mState = LOST;

		// Update drawer
		mpFrameDrawer->Update(this);

		// If tracking were good, check if we insert a keyframe
		if (bOK)
		{
			// Update motion model
			if (!mLastFrame.mTcw.empty())
			{
				cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
				mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
				mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
				mVelocity = mCurrentFrame.mTcw*LastTwc;
			}
			else
				mVelocity = cv::Mat();

			mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

			// Clean VO matches
			for (int i = 0; i < mCurrentFrame.N; i++)
			{
				MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
				if (pMP)
					if (pMP->Observations() < 1)
					{
						mCurrentFrame.mvbOutlier[i] = false;
						mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
					}
			}

			// Delete temporal MapPoints
			for (list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++)
			{
				MapPoint* pMP = *lit;
				delete pMP;
			}
			mlpTemporalPoints.clear();

			// Check if we need to insert a new keyframe
			if (NeedNewKeyFrame())
				CreateNewKeyFrame();

			// We allow points with high innovation (considererd outliers by the Huber Function)
			// pass to the new keyframe, so that bundle adjustment will finally decide
			// if they are outliers or not. We don't want next frame to estimate its position
			// with those points so we discard them in the frame.
			for (int i = 0; i < mCurrentFrame.N; i++)
			{
				if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
					mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
			}
		}

		// Reset if the camera get lost soon after initialization
		if (mState == LOST)
		{
			if (mpMap->KeyFramesInMap() <= 5)
			{
				std::cout << "Track lost soon after initialisation, reseting..." << endl;
				mpSystem->Reset();
				return;
			}
		}

		if (!mCurrentFrame.mpReferenceKF)
			mCurrentFrame.mpReferenceKF = mpReferenceKF;

		mLastFrame = Frame(mCurrentFrame);
	}

	// Store frame pose information to retrieve the complete camera trajectory afterwards.
	if (!mCurrentFrame.mTcw.empty())
	{
		cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
		mlRelativeFramePoses.push_back(Tcr);
		mlpReferences.push_back(mpReferenceKF);
		mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
		mlbLost.push_back(mState == LOST);
	}
	else
	{
		// This can happen if tracking is lost
		mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
		mlpReferences.push_back(mlpReferences.back());
		mlFrameTimes.push_back(mlFrameTimes.back());
		mlbLost.push_back(mState == LOST);
	}

}



void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        std::cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }


			if (!twc_curr.empty() && !twc_last.empty())
			{            // Set Frame Poses
				
				cout << "Setting Initial Pose Offsets" << endl;
				cout << "Twc Offeset" << twc_offset <<endl<< endl;
				cout << "Twc Last Frame" << twc_last << endl << endl;

				mInitialFrame.SetPose(twc_last*twc_offset);
				cout << "Setting Initial Pose Offsets2" << endl;

				cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
				Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
				tcw.copyTo(Tcw.rowRange(0,3).col(3));
				mCurrentFrame.SetPose(Tcw);
			}
			else
			{
				mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
				cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
				Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
				tcw.copyTo(Tcw.rowRange(0, 3).col(3));
				mCurrentFrame.SetPose(Tcw);
			}

            CreateInitialMapMonocular();
        }
    }
}


void Tracking::CAMERA_A_MonocularInitialization()
{

	if (!mpInitializer)
	{
		std::cout << "Not Initilized" << endl;
		// Set Reference Frame
		if (mCurrentFrame.mvKeys.size() > 100)
		{
			mInitialFrame = Frame(mCurrentFrame);
			mInitialFrame_alt = Frame(mCurrentFrame_alt);

			mLastFrame = Frame(mCurrentFrame);
			mLastFrame_alt = Frame(mCurrentFrame_alt);

			mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
			mvbPrevMatched_alt.resize(mCurrentFrame_alt.mvKeysUn.size());

			for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
				mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

			for (size_t i = 0; i < mCurrentFrame_alt.mvKeysUn.size(); i++)
				mvbPrevMatched_alt[i] = mCurrentFrame_alt.mvKeysUn[i].pt;


			if (mpInitializer)
				delete mpInitializer;
			if (mpInitializer_alt)
				delete mpInitializer_alt;

			mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);
			mpInitializer_alt = new Initializer(mCurrentFrame_alt, 1.0, 200);
			fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
			fill(mvIniMatches_alt.begin(), mvIniMatches_alt.end(), -1);

			return;
		}
	}
	else
	{

		// Try to initialize
		if ((int)mCurrentFrame.mvKeys.size() <= 100 )
		{
			delete mpInitializer;
			delete mpInitializer_alt;
			mpInitializer = static_cast<Initializer*>(NULL);
			mpInitializer_alt = static_cast<Initializer*>(NULL);
			fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
			fill(mvIniMatches_alt.begin(), mvIniMatches_alt.end(), -1);

			return;
		}

		// Find correspondences
		ORBmatcher matcher(0.9, true);
		int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

		// Find correspondences
		ORBmatcher matcher_alt(0.9, true);
		int nmatches_alt = matcher_alt.SearchForInitialization(mInitialFrame_alt, mCurrentFrame_alt, mvbPrevMatched_alt, mvIniMatches_alt, 100);

		// Check if there are enough correspondences
		if (nmatches < 100 || nmatches_alt<100)
		{
			delete mpInitializer;
			delete mpInitializer_alt;
			mpInitializer = static_cast<Initializer*>(NULL);
			mpInitializer_alt = static_cast<Initializer*>(NULL);
			return;
		}

		cv::Mat Rcw; // Current Camera Rotation
		cv::Mat tcw; // Current Camera Translation
		cv::Mat Rcw2; // Current Camera Rotation
		cv::Mat tcw2; // Current Camera Translation
		vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
		vector<bool> vbTriangulated_alt; // Triangulated Correspondences (mvIniMatches)
		//cout << "maybe here" << endl;
		if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated) && mpInitializer_alt->Initialize(mCurrentFrame_alt, mvIniMatches_alt, Rcw2, tcw2, mvIniP3D_alt, vbTriangulated_alt))
		{
			//cout << "maybe here2" << endl;


			//cout << "maybe here3" << endl;

			for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++)
			{
				if (mvIniMatches[i] >= 0 && !vbTriangulated[i])
				{
					mvIniMatches[i] = -1;
					nmatches--;
				}
			}
			//cout << "maybe here4" << endl;

			for (size_t i = 0, iend = mvIniMatches_alt.size(); i < iend; i++)
			{
				//cout << "maybe here4a" << endl;
				//cout << "Size: " << mvIniMatches_alt.size() << "  Size: " << vbTriangulated_alt.size() << endl;
				if (mvIniMatches_alt[i] >= 0 && !vbTriangulated_alt[i])
				{
				//	cout << "maybe here4b" << endl;

					mvIniMatches_alt[i] = -1;
				//	cout << "maybe here4c" << endl;
					nmatches_alt--;
				}
			}


			//cout << "maybe here5" << endl;

			

			std::cout << "start" << endl;
 
			// Set Frame Poses
			mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

			cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
			Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
			tcw.copyTo(Tcw.rowRange(0, 3).col(3));
			mCurrentFrame.SetPose(Tcw);
			std::cout << "Initailizing-1" << endl;
			std::cout << "Tcw  = " << endl << " " << Tcw << endl << endl;

			 


			//std::cout << "Initailizing-3" << endl;

			mInitialFrame_alt.SetPose(R_y);
			//std::cout << "Initailizing-4" << endl;

			//cv::Mat Tcw_alt = cv::Mat::eye(4, 4, CV_32F);
			//Tcw_alt =;

			mCurrentFrame_alt.SetPose(Tcw * R_y);
			//std::cout << "Initailizing-5" << endl;

			//CreateInitialMapMonocular();

			//std::cout << "Initailizing" << endl;

			//CreateInitialMapMonocular_Camera_B();
			//std::cout << "Trying to get B" << endl;
			CreateInitialMapMonocular_Camera_A();
		}
	}
}



void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    std::cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        std::cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();

	if (!twc_curr.empty() && !twc_last.empty())
	{
		cout << "Correct for Pose Offsets" << endl;

		//scale factor to appy to normalize
		invMedianDepth = 1.0f / (abs(twc_curr.at<float>(0, 2)- twc_last.at<float>(0,2) + twc_curr.at<float>(0, 1) - twc_last.at<float>(0, 1) + twc_curr.at<float>(0, 0) - twc_last.at<float>(0, 0)) / 3.0f);
		Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3)*invMedianDepth;
		Tc2w = Tc2w * twc_last*twc_offset.clone();

	}

    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CreateInitialMapMonocular_Camera_B()
{
	// Create KeyFrames
	KeyFrame* pKFini_alt = new KeyFrame(mInitialFrame_alt, mpMap, mpKeyFrameDB);
	KeyFrame* pKFcur_alt = new KeyFrame(mCurrentFrame_alt, mpMap, mpKeyFrameDB);


	pKFini_alt->ComputeBoW();
	pKFcur_alt->ComputeBoW();

	// Insert KFs in the map
	mpMap->AddKeyFrame(pKFini_alt);
	mpMap->AddKeyFrame(pKFcur_alt);

	// Create MapPoints and asscoiate to keyframes
	for (size_t i = 0; i < mvIniMatches_alt.size(); i++)
	{
		if (mvIniMatches_alt[i] < 0)
			continue;

		//Create MapPoint.
		cv::Mat worldPos(mvIniP3D_alt[i]);

		MapPoint* pMP = new MapPoint(worldPos, pKFcur_alt, mpMap);

		pKFini_alt->AddMapPoint(pMP, i);
		pKFcur_alt->AddMapPoint(pMP, mvIniMatches_alt[i]);

		pMP->AddObservation(pKFini_alt, i);
		pMP->AddObservation(pKFcur_alt, mvIniMatches_alt[i]);

		pMP->ComputeDistinctiveDescriptors();
		pMP->UpdateNormalAndDepth();

		//Fill Current Frame structure
		mCurrentFrame_alt.mvpMapPoints[mvIniMatches_alt[i]] = pMP;
		mCurrentFrame_alt.mvbOutlier[mvIniMatches_alt[i]] = false;

		//Add to Map
		mpMap->AddMapPoint(pMP);
	}

	// Update Connections
	pKFini_alt->UpdateConnections();
	pKFcur_alt->UpdateConnections();

	// Bundle Adjustment
	std::cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

	Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

	// Set median depth to 1
	float medianDepth = pKFini_alt->ComputeSceneMedianDepth(2);
	float invMedianDepth = 1.0f / medianDepth;

	if (medianDepth < 0 || pKFcur_alt->TrackedMapPoints(1) < 100)
	{
		std::cout << "Wrong initialization, reseting..." << endl;
		Reset();
		return;
	}

	/*
	//This should handle the rotational offset for the FOVs
	float angle = 90.0f * 3.142f / 180.0f;
	//std::cout << "Initailizing-2" << endl;

	cv::Mat R_y_3d = (cv::Mat_<float>(3, 3) <<
		cos(angle), 0, sin(angle),
		0, 1, 0, 0,
		-sin(angle), 0, cos(angle));

	//This should handle the rotational offset for the FOVs

	cv::Mat R_y = (cv::Mat_<float>(4, 4) <<
		cos(angle), 0, sin(angle), 0,
		0, 1, 0, 0,
		-sin(angle), 0, cos(angle), 0,
		0, 0, 0, 1);
		*/

	// Scale initial baseline
	cv::Mat Tc2w = pKFcur_alt->GetPose();
	Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3)*invMedianDepth;
	pKFcur_alt->SetPose(Tc2w*R_y);
	pKFini_alt->SetPose(R_y);

	// Scale points
	vector<MapPoint*> vpAllMapPoints = pKFini_alt->GetMapPointMatches();
	for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
	{
		if (vpAllMapPoints[iMP])
		{
			MapPoint* pMP = vpAllMapPoints[iMP];
			pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
		}
	}

	mpLocalMapper->InsertKeyFrame(pKFini_alt);
	mpLocalMapper->InsertKeyFrame(pKFcur_alt);

	mCurrentFrame_alt.SetPose(pKFcur_alt->GetPose());
	mnLastKeyFrameId = mCurrentFrame_alt.mnId;
	mpLastKeyFrame = pKFcur_alt;

	mvpLocalKeyFrames.push_back(pKFcur_alt);
	mvpLocalKeyFrames.push_back(pKFini_alt);
	mvpLocalMapPoints = mpMap->GetAllMapPoints();
	mpReferenceKF = pKFcur_alt;
	mCurrentFrame_alt.mpReferenceKF = pKFcur_alt;

	mLastFrame_alt = Frame(mCurrentFrame_alt);

	mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

	mpMapDrawer->SetCurrentCameraPose(pKFcur_alt->GetPose());

	mpMap->mvpKeyFrameOrigins.push_back(pKFini_alt);

	mState = OK;
}


void Tracking::CreateInitialMapMonocular_Camera_A()
{
	//std::cout << "Initailizing 0" << endl;

	// Create KeyFrames
		//std::cout << "Initailizing 0a" << endl;
	KeyFrame* pKFcur_alt = new KeyFrame(mCurrentFrame_alt, mpMap, mpKeyFrameDB);

	//std::cout << "Initailizing 0b" << endl;
	KeyFrame* pKFini_alt = new KeyFrame(mInitialFrame_alt, mpMap, mpKeyFrameDB);

	KeyFrame* pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
	KeyFrame* pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);


	//std::cout << "Initailizing 1" << endl;

	pKFini->ComputeBoW();
	pKFcur->ComputeBoW();

	pKFini_alt->ComputeBoW();
	pKFcur_alt->ComputeBoW();

	// Insert KFs in the map
	mpMap->AddKeyFrame(pKFini_alt);
	mpMap->AddKeyFrame(pKFini);

	mpMap->AddKeyFrame(pKFcur_alt);
	mpMap->AddKeyFrame(pKFcur);

	//std::cout << "Initailizing 2" << endl;
		//This should handle the rotational offset for the FOVs
	/*
	float angle = 90 * 3.14 / 180;

	cv::Mat R_y = (cv::Mat_<float>(4, 4) <<
		cos(angle), 0, sin(angle), 0,
		0, 1, 0, 0,
		-sin(angle), 0, cos(angle), 0,
		0, 0, 0, 1);

	cv::Mat R_y_3d = (cv::Mat_<float>(3, 3) <<
		cos(angle), 0, sin(angle),
		0, 1, 0, 0,
		-sin(angle), 0, cos(angle));
	*/

	// Create MapPoints and asscoiate to keyframes alt
	for (size_t i = 0; i < mvIniMatches_alt.size(); i++)
	{
		if (mvIniMatches_alt[i] < 0)
			continue;

		//Create MapPoint.
		cv::Mat worldPos_alt(mvIniP3D_alt[i]);

		//std::cout << "World_Pos_alt = " << mvIniP3D_alt[i] << endl;

		
		MapPoint* pMP_alt = new MapPoint(R_y_3d * worldPos_alt , pKFcur, mpMap);

		pKFini_alt->AddMapPoint(pMP_alt, i);
		pKFcur_alt->AddMapPoint(pMP_alt, mvIniMatches_alt[i]);

		pMP_alt->AddObservation(pKFini_alt, i);
		pMP_alt->AddObservation(pKFcur_alt, mvIniMatches_alt[i]);

		pMP_alt->ComputeDistinctiveDescriptors();
		pMP_alt->UpdateNormalAndDepth();

		//Fill Current Frame structure
		mCurrentFrame_alt.mvpMapPoints[mvIniMatches_alt[i]] = pMP_alt;
		mCurrentFrame_alt.mvbOutlier[mvIniMatches_alt[i]] = false;

		//Add to Map
		mpMap->AddMapPoint(pMP_alt);
	}

	// Create MapPoints and asscoiate to keyframes
	for (size_t i = 0; i < mvIniMatches.size(); i++)
	{
		if (mvIniMatches[i] < 0)
			continue;

		//Create MapPoint.
		cv::Mat worldPos(mvIniP3D[i]);

		MapPoint* pMP = new MapPoint(worldPos, pKFcur, mpMap);

		pKFini->AddMapPoint(pMP, i);
		pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

		pMP->AddObservation(pKFini, i);
		pMP->AddObservation(pKFcur, mvIniMatches[i]);

		pMP->ComputeDistinctiveDescriptors();
		pMP->UpdateNormalAndDepth();

		//Fill Current Frame structure
		mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
		mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

		//Add to Map
		mpMap->AddMapPoint(pMP);
	}



	// Update Connections
	pKFini_alt->UpdateConnections();
	pKFcur_alt->UpdateConnections();
	pKFini->UpdateConnections();
	pKFcur->UpdateConnections();

	// Bundle Adjustment
	std::cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

	Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

	// Set median depth to 1
	float medianDepth = pKFini->ComputeSceneMedianDepth(2);
	float invMedianDepth = 1.0f / medianDepth;

	if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100)
	{
		std::cout << "Median: " << medianDepth<< endl;
		std::cout << "# Tracked Points: " << pKFcur->TrackedMapPoints(1) << endl;


		std::cout << "Wrong initialization, reseting..." << endl;
		Reset();
		return;
	}
	std::cout << "Initailizing 5" << endl;

	// Scale initial baseline
	cv::Mat Tc2w = pKFcur->GetPose();

	Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3)*invMedianDepth;
	pKFcur->SetPose(Tc2w);

	cv::Mat Tc2w_alt;
	Tc2w.copyTo(Tc2w_alt);
	cv::Mat Rcw = Tc2w_alt.rowRange(0, 3).colRange(0, 3);
	cv::Mat tcw = Tc2w_alt.rowRange(0, 3).col(3);



	Tc2w_alt = Tc2w * R_y;
	/*cout << "Tc2w = " << endl << " " << Tc2w << endl << endl;
	cout << "Tc2w_alt = " << endl << " " << Tc2w_alt << endl << endl;
	cout << "ini = " << endl << " " << pKFini->GetPose() << endl << endl;
	cout << "ini_alt = " << endl << " " << R_y << endl << endl;
	*/
	pKFcur_alt->SetPose(Tc2w_alt);
	//since the initial frame is assumed to be zero rotation/translation. The pose is just the rotation between the FOVs
	pKFini_alt->SetPose(R_y);
	//std::cout << "Initailizing 6" << endl;

	// Scale points
	vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
	for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
	{
		if (vpAllMapPoints[iMP])
		{
			MapPoint* pMP = vpAllMapPoints[iMP];
			pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
		}
	}
	//std::cout << "Initailizing 7" << endl;

	// Scale points
	vector<MapPoint*> vpAllMapPoints_alt = pKFini_alt->GetMapPointMatches();
	for (size_t iMP = 0; iMP < vpAllMapPoints_alt.size(); iMP++)
	{
		if (vpAllMapPoints_alt[iMP])
		{
			MapPoint* pMP_alt = vpAllMapPoints_alt[iMP];
			pMP_alt->SetWorldPos(pMP_alt->GetWorldPos()*invMedianDepth);
		}
	}
	//std::cout << "Initailizing 8" << endl;

	mpLocalMapper->InsertKeyFrame(pKFini_alt);
	mpLocalMapper->InsertKeyFrame(pKFcur_alt);
	mpLocalMapper->InsertKeyFrame(pKFini);
	mpLocalMapper->InsertKeyFrame(pKFcur);


	mCurrentFrame_alt.SetPose(pKFcur_alt->GetPose());
	mCurrentFrame.SetPose(pKFcur->GetPose());

	mnLastKeyFrameId = mCurrentFrame.mnId;
	mnLastKeyFrameId_alt = mCurrentFrame_alt.mnId;

	mpLastKeyFrame = pKFcur;
	mpLastKeyFrame_alt = pKFcur_alt;
	//std::cout << "Initailizing 9" << endl;

	mvpLocalKeyFrames.push_back(pKFcur_alt);
	mvpLocalKeyFrames.push_back(pKFini_alt);
	mvpLocalKeyFrames.push_back(pKFcur);
	mvpLocalKeyFrames.push_back(pKFini);


	mvpLocalMapPoints = mpMap->GetAllMapPoints();
	mpReferenceKF = pKFcur;
	mpReferenceKF_alt = pKFcur_alt;
	std::cout << "ALT" << mpReferenceKF_alt << endl;
	mCurrentFrame.mpReferenceKF = pKFcur;
	mCurrentFrame_alt.mpReferenceKF = pKFcur_alt;

	mLastFrame = Frame(mCurrentFrame);
	mLastFrame_alt = Frame(mCurrentFrame_alt);

	mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

	mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

	mpMap->mvpKeyFrameOrigins.push_back(pKFini);
	//std::cout << "Initailizing 10" << endl;

	mState = OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

	std::cout << "Ref Key Frame Match nmatches: " << nmatches << endl;
    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}



bool Tracking::TrackReferenceKeyFrameMulti()
{
	// Compute Bag of Words vector
	mCurrentFrame.ComputeBoW();
	mCurrentFrame_alt.ComputeBoW();
	std::cout << "Made it through BoW" << endl;
	// We perform first an ORB matching with the reference keyframe
	// If enough matches are found we setup a PnP solver
	ORBmatcher matcher(0.7, true);
	ORBmatcher matcher_alt(0.7, true);

	vector<MapPoint*> vpMapPointMatches;
	vector<MapPoint*> vpMapPointMatches_alt;


	int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);
	std::cout << "Ref Key Frame Match nmatches: " << nmatches << endl;

	int nmatches_alt = matcher_alt.SearchByBoW(mpReferenceKF_alt, mCurrentFrame_alt, vpMapPointMatches_alt);

	std::cout << "Ref Key Frame Alt Match nmatches: " << nmatches_alt << endl;

	if (nmatches < 15)
		return false;

	mCurrentFrame.mvpMapPoints = vpMapPointMatches;
	mCurrentFrame.SetPose(mLastFrame.mTcw);
	mCurrentFrame_alt.mvpMapPoints = vpMapPointMatches_alt;
	mCurrentFrame_alt.SetPose(mLastFrame_alt.mTcw);

	Optimizer::PoseOptimization(&mCurrentFrame);
	Optimizer::PoseOptimization(&mCurrentFrame_alt);

	// Discard outliers
	int nmatchesMap = 0;
	for (int i = 0; i < mCurrentFrame.N; i++)
	{
		if (mCurrentFrame.mvpMapPoints[i])
		{
			if (mCurrentFrame.mvbOutlier[i])
			{
				MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

				mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
				mCurrentFrame.mvbOutlier[i] = false;
				pMP->mbTrackInView = false;
				pMP->mnLastFrameSeen = mCurrentFrame.mnId;
				nmatches--;
			}
			else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
				nmatchesMap++;
		}
	}

	int nmatchesMap_alt = 0;
	for (int i = 0; i < mCurrentFrame_alt.N; i++)
	{
		if (mCurrentFrame_alt.mvpMapPoints[i])
		{
			if (mCurrentFrame_alt.mvbOutlier[i])
			{
				MapPoint* pMP = mCurrentFrame_alt.mvpMapPoints[i];

				mCurrentFrame_alt.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
				mCurrentFrame_alt.mvbOutlier[i] = false;
				pMP->mbTrackInView = false;
				pMP->mnLastFrameSeen = mCurrentFrame_alt.mnId;
				nmatches--;
			}
			else if (mCurrentFrame_alt.mvpMapPoints[i]->Observations() > 0)
				nmatchesMap_alt++;
		}
	}

	std::cout << "Num Matches: " << nmatchesMap << endl;
	std::cout << "Num Matches Alt: " << nmatchesMap_alt << endl;

	return nmatchesMap >= 10 || nmatchesMap_alt >=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}


void Tracking::UpdateLastFrameMulti()
{
	// Update pose according to reference keyframe
	KeyFrame* pRef = mLastFrame.mpReferenceKF;
	KeyFrame* pRef_alt = mLastFrame_alt.mpReferenceKF;
	cv::Mat Tlr = mlRelativeFramePoses.back();

	mLastFrame.SetPose(Tlr*pRef->GetPose());
	mLastFrame_alt.SetPose(Tlr*pRef_alt->GetPose());


	if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || mSensor == System::MONOCULARMULTICOLORFILTER || !mbOnlyTracking)
		return;

	// Create "visual odometry" MapPoints
	// We sort points according to their measured depth by the stereo/RGB-D sensor
	vector<pair<float, int> > vDepthIdx;
	vDepthIdx.reserve(mLastFrame.N);
	for (int i = 0; i < mLastFrame.N; i++)
	{
		float z = mLastFrame.mvDepth[i];
		if (z > 0)
		{
			vDepthIdx.push_back(make_pair(z, i));
		}
	}

	if (vDepthIdx.empty())
		return;

	sort(vDepthIdx.begin(), vDepthIdx.end());

	// We insert all close points (depth<mThDepth)
	// If less than 100 close points, we insert the 100 closest ones.
	int nPoints = 0;
	for (size_t j = 0; j < vDepthIdx.size(); j++)
	{
		int i = vDepthIdx[j].second;

		bool bCreateNew = false;

		MapPoint* pMP = mLastFrame.mvpMapPoints[i];
		if (!pMP)
			bCreateNew = true;
		else if (pMP->Observations() < 1)
		{
			bCreateNew = true;
		}

		if (bCreateNew)
		{
			cv::Mat x3D = mLastFrame.UnprojectStereo(i);
			MapPoint* pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

			mLastFrame.mvpMapPoints[i] = pNewMP;

			mlpTemporalPoints.push_back(pNewMP);
			nPoints++;
		}
		else
		{
			nPoints++;
		}

		if (vDepthIdx[j].first > mThDepth && nPoints > 100)
			break;
	}
}
bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);


    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

	int nThresh = 8;

    if(nmatches<nThresh) // threshold for enough matches
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap< nThresh;
        return nmatches> nThresh; // was 20
    }

    return nmatchesMap>= nThresh; // was 10
}

bool Tracking::TrackWithMotionModelMulti()
{
	ORBmatcher matcher(0.9, true);
	ORBmatcher matcher_alt(0.9, true);

	// Update last frame pose according to its reference keyframe
	// Create "visual odometry" points if in Localization Mode
	UpdateLastFrameMulti();

	mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
	mCurrentFrame_alt.SetPose(mVelocity*mLastFrame.mTcw*R_y);


	fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
	fill(mCurrentFrame_alt.mvpMapPoints.begin(), mCurrentFrame_alt.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));


	// Project points seen in previous frame
	int th;
	if (mSensor != System::STEREO)
		th = 15;
	else
		th = 7;
	int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);
	int nmatches_alt = matcher_alt.SearchByProjection(mCurrentFrame_alt , mCurrentFrame_alt, th, mSensor == System::MONOCULAR);

	// If few matches, uses a wider window search
	if (nmatches < 20)
	{
		fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
		nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR);
	}
	// If few matches, uses a wider window search
	if (nmatches_alt < 20)
	{
		fill(mCurrentFrame_alt.mvpMapPoints.begin(), mCurrentFrame_alt.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
		nmatches_alt = matcher_alt.SearchByProjection(mCurrentFrame_alt, mLastFrame_alt, 2 * th, mSensor == System::MONOCULAR);
	}


	int nThresh = 8;

	if (nmatches < nThresh && nmatches_alt < nThresh) // threshold for enough matches
		return false;
	else
	{
		int nmatchesMap = 0;
		int nmatchesMap_alt = 0;

		if (nmatches >= nThresh)
		{
			// Optimize frame pose with all matches
			Optimizer::PoseOptimization(&mCurrentFrame);

			// Discard outliers
			for (int i = 0; i < mCurrentFrame.N; i++)
			{
				if (mCurrentFrame.mvpMapPoints[i])
				{
					if (mCurrentFrame.mvbOutlier[i])
					{
						MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

						mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
						mCurrentFrame.mvbOutlier[i] = false;
						pMP->mbTrackInView = false;
						pMP->mnLastFrameSeen = mCurrentFrame.mnId;
						nmatches--;
					}
					else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
						nmatchesMap++;
				}
			}
		}

		if (nmatches_alt >= nThresh)
		{

			// Optimize frame pose with all matches
			Optimizer::PoseOptimization(&mCurrentFrame_alt);

			// Discard outliers
			for (int i = 0; i < mCurrentFrame_alt.N; i++)
			{
				if (mCurrentFrame_alt.mvpMapPoints[i])
				{
					if (mCurrentFrame_alt.mvbOutlier[i])
					{
						MapPoint* pMP_alt = mCurrentFrame_alt.mvpMapPoints[i];

						mCurrentFrame_alt.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
						mCurrentFrame_alt.mvbOutlier[i] = false;
						pMP_alt->mbTrackInView = false;
						pMP_alt->mnLastFrameSeen = mCurrentFrame_alt.mnId;
						nmatches--;
					}
					else if (mCurrentFrame_alt.mvpMapPoints[i]->Observations() > 0)
						nmatchesMap_alt++;
				}
			}
			
		}

			if (mbOnlyTracking)
			{
				mbVO = nmatchesMap_alt < nThresh || nmatchesMap < nThresh;

				return nmatches_alt > nThresh || nmatches > nThresh; // was 20
			}

			return nmatchesMap >= nThresh || nmatchesMap_alt >= nThresh; // was 10
		


	}
		
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    std::cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    std::cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    std::cout << " done" << endl;

    // Reset Loop Closing
    std::cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    std::cout << " done" << endl;

    // Clear BoW Database
    std::cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    std::cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
