/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef SYSTEM_H
#define SYSTEM_H


#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "ImuTypes.h"
#include "Settings.h"


namespace ORB_SLAM3
{

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev)
    {
        if(lev <= th)
            cout << str << endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};

class Viewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4,
        IMU_RGBD=5,
    };

    // File type
    enum FileType{
        TEXT_FILE=0,
        BINARY_FILE=1,
    };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    // bSynchronous (PATCHED, Arrooy fork): opt-in deterministic offline-replay
    // mode. LocalMapping/LoopClosing still run on their own std::thread (their
    // Run() bodies are untouched), but each iteration is gated by a driver
    // handshake (see LocalMapping/LoopClosing::StepOnce(), and DrainMapping()
    // below) instead of free OS scheduling -- replaying the same input then
    // always produces the same call interleaving. Off by default: live flight
    // must keep real background threads for real-time performance.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string(), const bool bSynchronous = false);

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");


    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear Atlas or the active map)
    void Reset();
    void ResetActiveMap();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();
    bool isShutDown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    void SaveTrajectoryEuRoC(const string &filename);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename);

    void SaveTrajectoryEuRoC(const string &filename, Map* pMap);
    void SaveKeyFrameTrajectoryEuRoC(const string &filename, Map* pMap);

    // Save data used for initialization debug
    void SaveDebugData(const int &iniIdx);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    // PATCHED (Arrooy fork): FrameDrawer's rendered overlay (tracked/matched
    // features colour-coded by state, KF/MP/match counts) is normally only
    // consumed by the internal Viewer thread. Exposed here so headless
    // callers (bUseViewer=false) can still get the same debug visualization.
    // Safe to call right after TrackMonocular — FrameDrawer guards its own
    // state with an internal mutex.
    cv::Mat GetFrameDrawerImage(float imageScale = 1.f);

    // PATCHED (Arrooy fork): full sparse-map snapshot for external map
    // visualization. GetTrackedMapPoints() (existing API) only returns the
    // current frame's tracked subset, not the whole map. Filters out points/
    // keyframes marked bad (culled). World frame, metres.
    std::vector<cv::Point3f> GetAllMapPointPositions();
    std::vector<cv::Point3f> GetAllKeyFramePositions();
    int GetNumMaps();

    // PATCHED (Arrooy fork): absolute IMU-frame pose + velocity of the
    // current tracked frame, once mono-inertial init has converged. Mirrors
    // OpenVINS/OKVIS2's absolute-state output for external logging (see
    // indoor-vio's OrbSlam3Slam::get_state() / vo::OdometryState). Returns
    // false (leaves outputs untouched) until mpAtlas->isImuInitialized().
    bool GetImuState(Sophus::SE3f &Twb, Eigen::Vector3f &velocityWb);

    // PATCHED (Arrooy fork): drives one deterministic LocalMapping + LoopClosing
    // iteration each (see the bSynchronous constructor arg above). No-op if the
    // System wasn't constructed with bSynchronous=true. Call after TrackMonocular
    // returns, never from inside Track() (Map::mMutexMapUpdate reentrancy).
    void DrainMapping();

    // PATCHED (Arrooy fork): explicit, on-demand binary Atlas save. Upstream
    // only saves the Atlas implicitly inside Shutdown() when the settings file
    // sets System.SaveAtlasToFile — which (a) forces the save to run before
    // trajectory export and (b) if Atlas::PreSave() crashes (a known upstream
    // fragility after mono-inertial map resets: dangling KeyFrame observation
    // pointers) takes the whole process down. Exposing it lets the caller run
    // the save LAST and, if desired, inside an isolated child process. Writes
    // <baseName>.osa (extension appended by SaveAtlas). Call after Shutdown().
    void SaveMapToFile(const std::string& baseName);

    // For debugging
    double GetTimeFromIMUInit();
    bool isLost();
    bool isFinished();

    void ChangeDataset();

    float GetImageScale();

#ifdef REGISTER_TIMES
    void InsertRectTime(double& time);
    void InsertResizeTime(double& time);
    void InsertTrackTime(double& time);
#endif

private:

    void SaveAtlas(int type);
    bool LoadAtlas(int type);

    string CalculateCheckSum(string filename, int type);

    // Input sensor
    eSensor mSensor;

    // PATCHED (Arrooy fork): see bSynchronous ctor arg / DrainMapping() above.
    bool mbSynchronous;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    //Map* mpMap;
    Atlas* mpAtlas;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;
    bool mbResetActiveMap;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Shutdown flag
    bool mbShutDown;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    // PATCHED (Arrooy fork): cached absolute IMU-frame state, snapshot under
    // mMutexState at the end of TrackMonocular (the coherent instant the
    // tracker just produced it, before LocalMapping/LoopClosing overwrite
    // mCurrentFrame cross-thread). Exposed by GetImuState() under the same lock
    // — the current-state-getter convention GetTrackingState/GetTrackedMapPoints
    // follow — so external per-frame polling never touches the live frame or
    // Frame::mpMutexImu on the hot path.
    Sophus::SE3f mImuTwb;
    Eigen::Vector3f mImuVwb = Eigen::Vector3f::Zero();
    bool mHasImuState = false;

    //
    string mStrLoadAtlasFromFile;
    string mStrSaveAtlasToFile;

    string mStrVocabularyFilePath;

    Settings* settings_;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
