/**
* This file is part of CCM-SLAM.
*
* Copyright (C): Patrik Schmuck <pschmuck at ethz dot ch> (ETH Zurich)
* For more information see <https://github.com/patriksc/CCM-SLAM>
*
* CCM-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* CCM-SLAM is based in the monocular version of ORB-SLAM2 by Raúl Mur-Artal.
* CCM-SLAM partially re-uses modules of ORB-SLAM2 in modified or unmodified condition.
* For more information see <https://github.com/raulmur/ORB_SLAM2>.
*
* CCM-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with CCM-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CSLAM_MAP_H_
#define CSLAM_MAP_H_

//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <set>
#include <mutex>
#include <sstream>
#include <queue>
#include <ctime>

//CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Datatypes.h>
#include <cslam/CentralControl.h>
#include <cslam/MapPoint.h>
#include <cslam/KeyFrame.h>
#include <cslam/Communicator.h>
#include <cslam/ORBVocabulary.h>

#ifdef DENSEMAP2
#include <dense_mapping_backend/Interfaces.h>
#endif

//Msgs
#include <ccmslam_msgs/Map.h>

using namespace std;
using namespace estd;

namespace cslam{

//forward decs
class CentralControl;
class Communicator;
class KeyFrame;
class MapPoint;
class KeyFrameDatabase;
//------------

class Map : public boost::enable_shared_from_this<Map>
{
public:
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
    typedef boost::shared_ptr<Communicator> commptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;

    struct kftimecmp{
        bool operator() (const kfptr pA, const kfptr pB) const;
    };

    struct kftimecmpsmaller{
        bool operator() (const kfptr pA, const kfptr pB) const;
    };

public:
    //---Constructor---
    Map(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, size_t MapId, eSystemState SysState);
    Map(const mapptr &pMapA, const mapptr &pMapB); //merge constructor
    void UpdateAssociatedData(); //need to be called after merge constructor. Cannot be called by merge constructor because of usage of shared_from_this()
    Map& operator=(Map& rhs);

    //---infrastructure---
    set<size_t> msuAssClients;
    void AddCCPtr(ccptr pCC);
    set<ccptr> GetCCPtrs();
    ccptr GetCCPtr(size_t nClientId);
    void SetOutdated(){unique_lock<mutex> lock(mMutexOutdated); mbOutdated=true;}
    bool GetOutdated(){unique_lock<mutex> lock(mMutexOutdated); return mbOutdated;}
    string mOdomFrame;
    size_t mMapId;
    eSystemState mSysState;

    //---Add/Erase data---
    void AddKeyFrame(kfptr pKF);
    void AddMapPoint(mpptr pMP);
    void EraseMapPoint(mpptr pMP);
    void EraseKeyFrame(kfptr pKF);
    void SetReferenceMapPoints(const std::vector<mpptr> &vpMPs);
    void ClearBadMPs();

    //---MP/KF parent ptrs
    void HandleMissingParent(size_t QueryId, size_t QueryCId, cv::Mat &T_cref_cquery, kfptr pRefKf);

    //---Setter---
    void SetCommunicator(commptr pComm) {mspComm.insert(pComm);}

    //---Getter---
    kfptr GetKfPtr(size_t KfId, size_t ClientId, bool bIgnoreMutex = false);
    kfptr GetKfPtr(idpair id){return GetKfPtr(id.first,id.second);}
    kfptr GetRandKfPtr();
    mpptr GetMpPtr(size_t MpId, size_t ClientId);
    mpptr GetMpPtr(idpair id){return GetMpPtr(id.first,id.second);}
    kfptr GetErasedKfPtr(size_t KfId, size_t ClientId);
    kfptr GetErasedKfPtr(idpair id){return GetErasedKfPtr(id.first,id.second);}
    mpptr GetErasedMpPtr(size_t MpId, size_t ClientId);
    mpptr GetErasedMpPtr(idpair id){return GetErasedMpPtr(id.first,id.second);}
    bool IsKfDeleted(size_t KfId, size_t ClientId);
    bool IsKfDeleted(idpair id){return IsKfDeleted(id.first,id.second);}
    bool IsMpDeleted(size_t MpId, size_t ClientId);
    bool IsMpDeleted(idpair id){return IsMpDeleted(id.first,id.second);}

    vector<kfptr> GetAllKeyFrames();
    vector<mpptr> GetAllMapPoints();
    std::vector<mpptr> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();
    long unsigned int GetMaxMPid();
    long unsigned int GetMaxKFidUnique();
    long unsigned int GetMaxMPidUnique();
    long unsigned int GetLastKfIdUnique();

    kfptr GetPredecessor(kfptr pKF);

    kfptr GetFromKfBuffer(size_t KfId, size_t ClientId);
    bool IsInKfBuffer(size_t KfId, size_t ClientId);
    kfptr SearchAndReturnFromKfBuffer(size_t KfId, size_t ClientId);
    void DeleteFromKfBuffer(kfptr pKF);
    mpptr GetFromMpBuffer(size_t MpId, size_t ClientId);
    bool IsInMpBuffer(size_t MpId, size_t ClientId);
    mpptr SearchAndReturnFromMpBuffer(size_t MpId, size_t ClientId);
    void DeleteFromMpBuffer(mpptr pMP);

    std::vector<mpptr> GetMvpReferenceMapPoints() {return mvpReferenceMapPoints;}
    std::map<idpair,mpptr> GetMmpMapPoints() {return mmpMapPoints;}
    std::map<idpair,kfptr> GetMmpKeyFrames() {return mmpKeyFrames;}
    std::map<idpair,mpptr> GetMmpErasedMapPoints() {return mmpErasedMapPoints;}
    std::map<idpair,kfptr> GetMmpErasedKeyFrames() {return mmpErasedKeyFrames;}

    //---data---
    vector<kfptr> mvpKeyFrameOrigins;

    //---map management---
    void MapTrimming(kfptr pKFcur);
    void FindLocalKFsByTime(kfptr pKFcur,set<kfptr>& sKfVicinity,priority_queue<int>& pqNativeKFs,list<kfptr>& lForeignKFs, int nLocalKFs);
    void PackVicinityToMsg(kfptr pKFcur, ccmslam_msgs::Map &msgMap, ccptr pCC);
    set<mpptr> mspMPsToErase;

    //---Reset---
    void clear();

    //---mutexes & sync---
    bool LockMapUpdate(){unique_lock<mutex> lock(mMutexMapUpdate); if(!mbLockMapUpdate){mbLockMapUpdate = true; return true;} else return false;}
    bool LockPointCreation(){unique_lock<mutex> lock(mMutexPointCreation); if(!mbLockPointCreation){mbLockPointCreation = true; return true;} else return false;}
    void UnLockMapUpdate(){unique_lock<mutex> lock(mMutexMapUpdate);if(mbLockMapUpdate){mbLockMapUpdate = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"Map\": Attempt to UnLock MapUpdate -- was not locked" << endl; throw estd::infrastructure_ex();}}
    void UnLockPointCreation(){unique_lock<mutex> lock(mMutexPointCreation);if(mbLockPointCreation){mbLockPointCreation = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"Map\": Attempt to UnLock PointCreation -- was not locked" << endl; throw estd::infrastructure_ex();}}

    //BA
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    void setRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbRunningGBA = true;
    }
    void unsetRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbRunningGBA = false;
    }

    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }
    void setFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbFinishedGBA = true;
    }
    void unsetFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbFinishedGBA = false;
    }

    bool isNoStartGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbNoStartGBA;
    }
    void setNoStartGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbNoStartGBA = true;
    }
    void unsetNoStartGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbNoStartGBA = false;
    }

    #ifdef DONOTINTERRUPTMERGE
    bool isMergeStepGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbMergeStepGBA;
    }
    void setMergeStepGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbMergeStepGBA = true;
    }
    void unsetMergeStepGBA(){
//        unique_lock<std::mutex> lock(mMutexGBA); //already locked when this is used
        mbMergeStepGBA = false;
    }
    #endif

    void StopGBA();

    #ifdef FINALBA
    bool isGBAinterrupted(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbGBAinterrupted;
    }
    void setGBAinterrupted(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbGBAinterrupted = true;
    }
    void unsetGBAinterrupted(){
//        unique_lock<std::mutex> lock(mMutexGBA); //already locked when this is used
        mbGBAinterrupted = false;
    }
    #endif

    void RequestBA(size_t nClientId);
    void RunGBA(idpair nLoopKF);
    set<size_t> msnFinishedAgents;

    // Write KF states out to csv (EuRoC Format)
    void WriteStateToCsv(const std::string& filename,
                         const size_t clientId);

    // add this code (2023/11/10) //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void WriteStateToCsvTrimmed(const std::string& filename,
                         const size_t clientId);
    void WriteStateToCsvTrimmedV2(const std::string& filename,
                         const size_t clientId);
    void WriteStateToCsvTrimmedV3(const std::string& filename,
                         const size_t clientId);
    void WriteStateToCsvTrimmedV4(const std::string& filename,
                         const size_t clientId);                         
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //---Map Save/Load---
    void LoadMap(const string &path_name, vocptr voc, commptr comm, dbptr kfdb, uidptr uid);
    void SaveMap(const string &path_name);

    // add this code (2023/11/5) //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void TrimMotionViolateKF(kfptr targetKF);
    //vector<kfptr> DeleteMotionViolateKF(vector<kfptr> foundKFs, kfptr targetKF) {return foundKFs;}
    int DeleteMVKF(std::vector<kfptr> foundKFs);
    void CheckMotionViolateKF_before(const size_t clientId);
    void CheckMotionViolateKF_after(const size_t clientId);

    bool MotionCheck(const size_t clientId, kfptr pKF, Eigen::Matrix4d Tws, kfptr pKF_last, kfptr pKF_last_2, float last_X, float last_Y, float last_Z, float tracking_last_X, float tracking_last_Y, float tracking_last_Z,  float X_diff, float Y_diff, float Z_diff, float tracking_X_diff, float tracking_Y_diff, float tracking_Z_diff, float degree_margin, float movement_margin);
    void SetTcwCurrent(const size_t clientId, std::map<idpair,kfptr> mmpKeyFrames);
    void SetTcwNew(const size_t clientId, std::map<idpair,kfptr> mmpKeyFrames);
    void CorrectKF(const size_t clientId, bool isGBA);
    void CorrectKF_V2(const size_t clientId, std::map<idpair,kfptr> mmpKeyFrames, bool isGBA, bool reverse);
    void CorrectKF_V3(const size_t clientId, std::map<idpair,kfptr> mmpKeyFrames, bool isGBA, bool reverse);
    int CheckMVKF(std::vector<kfptr> foundKFs);
    void SetViolateFlag(std::vector<kfptr> foundKFs);
    Eigen::Matrix4d ComputeTws(kfptr pKF);
    int DeleteMotionViolateKF(std::vector<kfptr> foundKFs);
    int TrimMVKF_loose(const size_t clientId, std::vector<kfptr> foundKFs);
    void RequestBA_fromCorrectKF(size_t nClientId);
    void RunGBA_fromCorrectKF(idpair nLoopKF);
    int badflag_counter = 0;
    int goodKF_in_a_row = 0;
    int badKF_counter = 0;
    int badKF_in_a_row = 0;
    int badKF_in_a_row_max = 0;
    bool request_GBA = false;
    bool CorrectKF_switch = true;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    #ifdef LOGGING
    bool IsInUse(){return !mbOutdated;}
    #endif

    #ifdef DEBUGGING2
    void CheckStructure();
    #endif

protected:
    //---infrastructure---
    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    std::set<ccptr> mspCC;
    set<commptr> mspComm; //comm ptrs of associated clients -- necessary to pass to MPs/KFs

    //---data---
    std::vector<mpptr> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;
    long unsigned int mnMaxMPid;
    long unsigned int mnMaxKFidUnique;
    long unsigned int mnMaxMPidUnique;
    long unsigned int mnLastKfIdUnique;

    // add this code (2023/11/5) /////////////////////////////////////////////////////////////////////////////////////////
    kfptr violate_KF_candidate;
    kfptr violate_KF;
    int violate_flag_X = 0;
    int violate_flag_Y = 0;
    int violate_flag_Z = 0;
    int violate_flag = 0;

    float last_X = 0;
    float last_Y = 0;
    float last_Z = 0;

    float tracking_last_X = 0;
    float tracking_last_Y = 0;
    float tracking_last_Z = 0;

    float max_abs_X = 0;
    float max_abs_Y = 0;
    float max_abs_Z = 0;

    float remember_X = 0;
    float remember_Y = 0;
    float remember_Z = 0;

    float remember_last_X = 0;
    float remember_last_Y = 0;
    float remember_last_Z = 0;

    int KF_check_count = 0;
    int erase_flag = 0;

    float abs_X_total = 0;
    float abs_Y_total = 0;
    float abs_Z_total = 0;

    float abs_X_average = 0;
    float abs_Y_average = 0;
    float abs_Z_average = 0;

    float X_diff = 0;
    float X_diff_1 = 0;
    float X_diff_2 = 0;
    float X_diff_3 = 0;
    float past_three_X_diff = 0;
    float remember_past_three_X_diff = 0; 

    float Y_diff = 0;
    float Y_diff_1 = 0;
    float Y_diff_2 = 0;
    float Y_diff_3 = 0;
    float past_three_Y_diff = 0;
    float remember_past_three_Y_diff = 0; 

    float Z_diff = 0;
    float Z_diff_1 = 0;
    float Z_diff_2 = 0;
    float Z_diff_3 = 0;
    float past_three_Z_diff = 0;
    float remember_past_three_Z_diff = 0;

    float tracking_X_diff = 0;
    float tracking_Y_diff = 0;
    float tracking_Z_diff = 0;

    float v1_x;
    float v1_y;
    float v1_z;
    float v2_x;
    float v2_y;
    float v2_z;

    float tracking_v1_x;
    float tracking_v1_y;
    float tracking_v1_z;
    float tracking_v2_x;
    float tracking_v2_y;
    float tracking_v2_z;

    float xy_radian;
    float yz_radian;
    float zx_radian;
    float xy_degree;
    float yz_degree;
    float zx_degree;

    float tracking_xy_radian;
    float tracking_yz_radian;
    float tracking_zx_radian;
    float tracking_xy_degree;
    float tracking_yz_degree;
    float tracking_zx_degree;

    float radian;
    float degree;

    float tracking_radian;
    float tracking_degree;

    float movement_ratio = 0;
    float tracking_movement_ratio = 0;

    int violate_check_by_three_switch = 0;
    int WriteStateToCsvTrimmed_switch = 0;
    int WriteStateToCsv_separately = 1;

    int comm_triger_flag = 0;
    int erase_counter = 0;
    int erase_counter_degree = 0;
    int erase_counter_plain_degree = 0;
    int erase_counter_movement = 0;
    int update_flag = 1;
    int map_gba_counter = 0;
    int corrected_counter = 0;
    int deleted_counter = 0;

    float margin = 1;
    float degree_margin = 1;
    float movement_margin = 1;

    cv::Mat Bef_motion;
    cv::Mat mTwcBef_last;
    cv::Mat mTwcAft_last;
    cv::Mat translation_GBABef_last;
    cv::Mat translation_GBAAft_last;
    cv::Mat mTwcBef_last_2;
    cv::Mat mTwcAft_last_2;
    cv::Mat correct_effect;

    cv::Mat RcwBef_last;
    cv::Mat tcwBef_last;
    cv::Mat RwcBef_last;
    cv::Mat OwBef_last;

    cv::Mat RcwAft_last;
    cv::Mat tcwAft_last;
    cv::Mat RwcAft_last;
    cv::Mat OwAft_last;

    cv::Mat RcwBef_last_2;
    cv::Mat tcwBef_last_2;
    cv::Mat RwcBef_last_2;
    cv::Mat OwBef_last_2;

    cv::Mat RcwAft_last_2;
    cv::Mat tcwAft_last_2;
    cv::Mat RwcAft_last_2;
    cv::Mat OwAft_last_2;

    cv::Mat mVelocity_last_Bef;
    cv::Mat mVelocity_last_Aft;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::map<idpair,mpptr> mmpMapPoints; //Performance: second container is not nice, but fast... //CHECKHERE
    std::map<idpair,kfptr> mmpKeyFrames; //Performance: second container is not nice, but fast... //CHECKHERE
    // add this code (2023/11/10) ////////////////////////////////////////////////////////////////////////////////////////
    // std::map<idpair, mpptr> mmpMapPoints_trimmed;
    // std::map<idpair, mpptr> mmpKeyFrames_trimmed;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::map<idpair,mpptr> mmpErasedMapPoints;
    std::map<idpair,kfptr> mmpErasedKeyFrames;

//    map<idpair,kfptr> mmpKfBuffer;
//    map<idpair,mpptr> mmpMpBuffer;

    bool mbOutdated;

    //---mutexes---
    std::mutex mMutexCC;
    std::mutex mMutexErased;
    std::mutex mMutexMap;
//    mutex mMutexKfBuffer; //should not be necessary, buffers only accessed by comm thread
//    mutex mMutexMpBuffer; //should not be necessary, buffers only accessed by comm thread
    bool mbLockMapUpdate;
    bool mbLockPointCreation;
    std::mutex mMutexMapUpdate;
    std::mutex mMutexPointCreation; // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexOutdated;
    std::mutex mMutexStopGBAInProgess;

    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbNoStartGBA;
    bool mbStopGBAInProgress;
    #ifdef DONOTINTERRUPTMERGE
    bool mbMergeStepGBA;
    #endif

    #ifdef FINALBA
    bool mbGBAinterrupted;
    #endif
};

} //end namespace

#endif
