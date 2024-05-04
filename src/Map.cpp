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

#include <cslam/Map.h>

#include <cslam/Database.h>

namespace cslam {

Map::Map(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, size_t MapId, eSystemState SysState)
    : mnMaxKFid(0), mnMaxMPid(0),
      mnMaxKFidUnique(0),mnMaxMPidUnique(0),
      mNh(Nh), mNhPrivate(NhPrivate),
      mMapId(MapId),mbOutdated(false),
      mSysState(SysState),
      mbLockMapUpdate(false),mbLockPointCreation(false)
      ,mnLastKfIdUnique(0)
      ,mbStopGBA(false),mbRunningGBA(false),mbFinishedGBA(false),
    #ifdef FINALBA
    mbGBAinterrupted(false),
    #endif
    mbNoStartGBA(false),mbStopGBAInProgress(false)
    #ifdef DONOTINTERRUPTMERGE
    ,mbMergeStepGBA(false)
    #endif
{
    uint myseed = time(NULL);
    // add this code (2023/11/6) ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //myseed = 1701338453;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    srand (myseed);
    std::cout << "Map " << mMapId << " rand seed: " << myseed << std::endl;

    string SysType;
    if(mSysState == eSystemState::CLIENT)
    {
        SysType = "Client";
    }
    else if(mSysState == eSystemState::SERVER)
    {
        SysType = "Server";
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Map::Map(): invalid systems state: " << mSysState << endl;
        throw infrastructure_ex();
    }

    cout << "+++++ Map " << mMapId << " Initialized +++++" << endl;
}

Map::Map(const mapptr &pMapTarget, const mapptr &pMapToFuse)
    : mNh(pMapTarget->mNh),mNhPrivate(pMapTarget->mNhPrivate),
      mMapId(pMapTarget->mMapId),mbOutdated(false),
      mbLockMapUpdate(false),mbLockPointCreation(false)
      ,mnLastKfIdUnique(pMapToFuse->GetLastKfIdUnique())
    ,mbStopGBA(false),mbRunningGBA(false),mbFinishedGBA(false),
    #ifdef FINALBA
    mbGBAinterrupted(false),
    #endif
    mbNoStartGBA(false),mbStopGBAInProgress(false)
    #ifdef DONOTINTERRUPTMERGE
    ,mbMergeStepGBA(false)
    #endif
{

    mSysState = pMapTarget->mSysState;

    string SysType;
    if(mSysState == eSystemState::CLIENT)
    {
        SysType = "Client";
    }
    else if(mSysState == eSystemState::SERVER)
    {
        SysType = "Server";
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Map::Map(): invalid systems state: " << mSysState << endl;
        throw infrastructure_ex();
    }

    //data Map A
    set<size_t> msuAssClientsA = pMapTarget->msuAssClients;
    set<size_t> msnFinishedAgentsA = pMapTarget->msnFinishedAgents;
    vector<kfptr> mvpKeyFrameOriginsA = pMapTarget->mvpKeyFrameOrigins;
    long unsigned int mnMaxKFidA = pMapTarget->GetMaxKFid();
    long unsigned int mnMaxMPidA = pMapTarget->GetMaxMPid();
    long unsigned int mnMaxKFidUniqueA = pMapTarget->GetMaxKFidUnique();
    long unsigned int mnMaxMPidUniqueA = pMapTarget->GetMaxKFidUnique();
    std::map<idpair,mpptr> mmpMapPointsA = pMapTarget->GetMmpMapPoints();
    std::map<idpair,kfptr> mmpKeyFramesA = pMapTarget->GetMmpKeyFrames();
    std::map<idpair,mpptr> mmpErasedMapPointsA = pMapTarget->GetMmpErasedMapPoints();
    std::map<idpair,kfptr> mmpErasedKeyFramesA = pMapTarget->GetMmpErasedKeyFrames();
    set<ccptr> spCCA = pMapTarget->GetCCPtrs();

    //data Map B
    set<size_t> msuAssClientsB = pMapToFuse->msuAssClients;
    set<size_t> msnFinishedAgentsB = pMapToFuse->msnFinishedAgents;
    vector<kfptr> mvpKeyFrameOriginsB = pMapToFuse->mvpKeyFrameOrigins;
    long unsigned int mnMaxKFidB = pMapToFuse->GetMaxKFid();
    long unsigned int mnMaxMPidB = pMapToFuse->GetMaxMPid();
    long unsigned int mnMaxKFidUniqueB = pMapToFuse->GetMaxKFidUnique();
    long unsigned int mnMaxMPidUniqueB = pMapToFuse->GetMaxKFidUnique();
    std::map<idpair,mpptr> mmpMapPointsB = pMapToFuse->GetMmpMapPoints();
    std::map<idpair,kfptr> mmpKeyFramesB = pMapToFuse->GetMmpKeyFrames();
    std::map<idpair,mpptr> mmpErasedMapPointsB = pMapToFuse->GetMmpErasedMapPoints();
    std::map<idpair,kfptr> mmpErasedKeyFramesB = pMapToFuse->GetMmpErasedKeyFrames();
    set<ccptr> spCCB = pMapToFuse->GetCCPtrs();

    //fill new map
    mOdomFrame = pMapTarget->mOdomFrame;

    msuAssClients.insert(msuAssClientsA.begin(),msuAssClientsA.end());
    msuAssClients.insert(msuAssClientsB.begin(),msuAssClientsB.end());
    msnFinishedAgents.insert(msnFinishedAgentsA.begin(),msnFinishedAgentsA.end());
    msnFinishedAgents.insert(msnFinishedAgentsB.begin(),msnFinishedAgentsB.end());
    mvpKeyFrameOrigins.insert(mvpKeyFrameOrigins.end(),mvpKeyFrameOriginsA.begin(),mvpKeyFrameOriginsA.end());
    mvpKeyFrameOrigins.insert(mvpKeyFrameOrigins.end(),mvpKeyFrameOriginsB.begin(),mvpKeyFrameOriginsB.end());
    mnMaxKFid = std::max(mnMaxKFidA,mnMaxKFidB);
    mnMaxMPid = std::max(mnMaxMPidA,mnMaxMPidB);
    mnMaxKFidUnique = std::max(mnMaxKFidUniqueA,mnMaxKFidUniqueB);
    mnMaxMPidUnique = std::max(mnMaxMPidUniqueA,mnMaxMPidUniqueB);
    mmpMapPoints.insert(mmpMapPointsA.begin(),mmpMapPointsA.end());
    mmpMapPoints.insert(mmpMapPointsB.begin(),mmpMapPointsB.end());
    mmpKeyFrames.insert(mmpKeyFramesA.begin(),mmpKeyFramesA.end());
    mmpKeyFrames.insert(mmpKeyFramesB.begin(),mmpKeyFramesB.end());
    mmpErasedMapPoints.insert(mmpErasedMapPointsA.begin(),mmpErasedMapPointsA.end());
    mmpErasedMapPoints.insert(mmpErasedMapPointsB.begin(),mmpErasedMapPointsB.end());
    mmpErasedKeyFrames.insert(mmpErasedKeyFramesA.begin(),mmpErasedKeyFramesA.end());
    mmpErasedKeyFrames.insert(mmpErasedKeyFramesB.begin(),mmpErasedKeyFramesB.end());
    mspCC.insert(spCCA.begin(),spCCA.end());
    mspCC.insert(spCCB.begin(),spCCB.end());

    #ifdef FINALBA
    if(pMapTarget->isGBAinterrupted() || pMapToFuse->isGBAinterrupted())
        this->mbGBAinterrupted = true;
    #endif

    for(set<ccptr>::const_iterator sit = mspCC.begin();sit!=mspCC.end();++sit)
    {
        ccptr pCC = *sit;
        mspComm.insert(pCC->mpCH->GetCommPtr());
    }

    for(set<size_t>::iterator sit = msnFinishedAgentsA.begin();sit!=msnFinishedAgentsA.end();++sit)
        cout << "Target Map Finished Agents: " << *sit << endl;

    for(set<size_t>::iterator sit = msnFinishedAgents.begin();sit!=msnFinishedAgents.end();++sit)
        cout << "Merged Map Finished Agents: " << *sit << endl;

    //----------------------------
}

void Map::UpdateAssociatedData()
{

    //replace associated maps
    for(std::map<idpair,kfptr>::iterator mit = mmpKeyFrames.begin();mit!=mmpKeyFrames.end();++mit)
    {
        kfptr pKF = mit->second;
        pKF->ReplaceMap(this->shared_from_this());
        for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
        {
            commptr pComm = *sit2;
            pKF->AddCommPtr(pComm);
        }
    }

    for(std::map<idpair,mpptr>::iterator mit = mmpMapPoints.begin();mit!=mmpMapPoints.end();++mit)
    {
        mpptr pMP = mit->second;
        pMP->ReplaceMap(this->shared_from_this());
        for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
        {
            commptr pComm = *sit2;
            pMP->AddCommPtr(pComm);
        }
    }

    for(map<idpair,kfptr>::iterator mit = mmpErasedKeyFrames.begin();mit!=mmpErasedKeyFrames.end();++mit)
    {
        kfptr pKF = mit->second;
        pKF->ReplaceMap(this->shared_from_this());
        for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
        {
            commptr pComm = *sit2;
            pKF->AddCommPtr(pComm);
        }
    }

    for(map<idpair,mpptr>::iterator mit = mmpErasedMapPoints.begin();mit!=mmpErasedMapPoints.end();++mit)
    {
        mpptr pMP = mit->second;
        pMP->ReplaceMap(this->shared_from_this());
        for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
        {
            commptr pComm = *sit2;
            pMP->AddCommPtr(pComm);
        }
    }
}

void Map::AddKeyFrame(kfptr pKF)
{
    {
        unique_lock<mutex> lock(mMutexMap);

        if(mSysState == eSystemState::CLIENT)
        {
            std::map<idpair,kfptr>::iterator mit = mmpKeyFrames.find(pKF->mId);
            if(mit != mmpKeyFrames.end())
            {
                return;
            }
            else
            {
                commptr pComm = *(mspComm.begin());
                if(!pKF->mbFromServer)
                    pComm->PassKftoComm(pKF);
            }
        }
        else if(mSysState == eSystemState::SERVER)
        {
            for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
            {
                pKF->AddCommPtr(*sit);
            }

            mnLastKfIdUnique = pKF->mUniqueId;
        }

        if(pKF->mId.first>mnMaxKFid)
            mnMaxKFid=pKF->mId.first;
        if(pKF->mUniqueId>mnMaxKFidUnique)
            mnMaxKFidUnique=pKF->mUniqueId;

        mmpKeyFrames[pKF->mId] = pKF;

        if(mSysState == SERVER) {
            if(mmpKeyFrames.size() % 10 == 0) { // modified (2023/12/4)
                cout << "KFs in Map : " << mmpKeyFrames.size() << endl;
                
                for(int it=0;it<4;++it) {
                    std::stringstream ss;
                    ss << params::stats::msOutputDir << "KF_GBA_" << it << "_KF" << mmpKeyFrames.size() << ".csv";
                    this->WriteStateToCsv(ss.str(),it);
                }
                
            }
        }
    }
}

void Map::AddMapPoint(mpptr pMP)
{
    unique_lock<mutex> lock(mMutexMap);

    if(mSysState == eSystemState::CLIENT)
    {
        std::map<idpair,mpptr>::iterator mit = mmpMapPoints.find(pMP->mId);
        if(mit != mmpMapPoints.end())
        {
            return;
        }
        else
        {
            commptr pComm = *(mspComm.begin());
            if(!pMP->mbFromServer)
            {
                pComm->PassMptoComm(pMP);
            }
        }
    }
    else if(mSysState == eSystemState::SERVER)
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            pMP->AddCommPtr(*sit);
        }
    }

    if(pMP->mId.first>mnMaxMPid)
        mnMaxMPid=pMP->mId.first;
    if(pMP->mUniqueId>mnMaxMPidUnique)
        mnMaxMPidUnique=pMP->mUniqueId;

    mmpMapPoints[pMP->mId] = pMP;
}

void Map::EraseMapPoint(mpptr pMP)
{
    unique_lock<mutex> lock(mMutexMap);

    std::map<idpair,mpptr>::iterator mit = mmpMapPoints.find(pMP->mId);
    if(mit != mmpMapPoints.end()) mmpMapPoints.erase(mit);

    if(msuAssClients.count(pMP->mId.second))
    {
        unique_lock<mutex> lock2(mMutexErased);
        mmpErasedMapPoints[pMP->mId] = pMP;
    }
}

void Map::EraseKeyFrame(kfptr pKF)
{
    if(pKF->mId.first == 0)
    {
        cout << COUTFATAL << " cannot erase Origin-KF" << endl;
        throw infrastructure_ex();
    }

    unique_lock<mutex> lock(mMutexMap);

    std::map<idpair,kfptr>::iterator mit = mmpKeyFrames.find(pKF->mId);
    if(mit != mmpKeyFrames.end()) mmpKeyFrames.erase(mit);

    if(msuAssClients.count(pKF->mId.second))
    {
        unique_lock<mutex> lock2(mMutexErased);
        mmpErasedKeyFrames[pKF->mId] = pKF;
    }
}

void Map::SaveMap(const string &path_name) {
    std::cout << "+++ Save Map to File +++" << std::endl;

    std::string kf_tmp = "/keyframes";
    std::string mp_tmp = "/mappoints";

    char cstr0[path_name.size()+1];
    char cstr[path_name.size() + kf_tmp.size()+1];
    char cstr2[path_name.size() + mp_tmp.size()+1];
    strcpy(cstr0,path_name.c_str());
    strcpy(cstr, (path_name+kf_tmp).c_str());
    strcpy(cstr2, (path_name+mp_tmp).c_str());
    std::cout << cstr0 << std::endl;
    std::cout << mkdir(cstr0,  0777);
    std::cout << mkdir(cstr,  0777);
    std::cout << mkdir(cstr2,  0777);
    std::cout << std::endl;

    std::cout << "--> Writing Keyframes to file" << std::endl;
    auto keyframes = this->GetAllKeyFrames();
    for(unsigned long int i = 0; i < keyframes.size(); i++) {
        std::ofstream fs;
        fs.open(path_name+"/keyframes/keyframes"+std::to_string(i)+".txt");
        if(fs.is_open()) {
            kfptr kfi = keyframes[i];
            std::stringstream kf_ss;
            cereal::BinaryOutputArchive oarchive(kf_ss);
            oarchive(*kfi);

            fs << kf_ss.str();
            fs.close();
        }
    }

    std::cout << "--> Writing Landmarks to file" << std::endl;
    auto mappoints = this->GetAllMapPoints();
    for(unsigned long int i = 0; i < mappoints.size(); i++) {
        std::ofstream fs;
        fs.open(path_name+"/mappoints/mappoints"+std::to_string(i)+".txt");
        if(fs.is_open()) {
            mpptr mpi = mappoints[i];
            std::stringstream mp_ss;
            cereal::BinaryOutputArchive oarchive(mp_ss);
            oarchive(*mpi);

            fs << mp_ss.str();
            fs.close();
        }
    }

    std::cout << "+++ DONE +++" << std::endl;
}

void Map::SetReferenceMapPoints(const vector<mpptr> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

vector<Map::kfptr> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);

    vector<kfptr> vpKFs;
    for(std::map<idpair,kfptr>::iterator mit_set = mmpKeyFrames.begin();mit_set!=mmpKeyFrames.end();++mit_set)
        vpKFs.push_back(mit_set->second);
    return vpKFs;
}

vector<Map::mpptr> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);

    vector<mpptr> vpMPs;
    for(std::map<idpair,mpptr>::iterator mit_set = mmpMapPoints.begin();mit_set!=mmpMapPoints.end();++mit_set)
        vpMPs.push_back(mit_set->second);
    return vpMPs;
}

void Map::LoadMap(const string &path_name, vocptr voc, commptr comm, dbptr kfdb, uidptr uid) {
    std::cout << "+++ Load Map from File +++" << std::endl;

    if(!voc) {
        std::cout << COUTFATAL << "invalid vocabulary ptr" << std::endl;
        exit(-1);
    }

    std::vector<std::string> filenames_kf, filenames_mp;
    std::vector<kfptr> keyframes;
    std::vector<mpptr> mappoints;

    map<idpair,idpair> saved_kf_ids_to_sys_ids; //maps the IDs of the saved KFs/MPs to new IDs -- systems assumes after load, all client IDs are 0.
    map<idpair,idpair> saved_mp_ids_to_sys_ids;
    size_t next_kf_id0 = 0;
    size_t next_mp_id0 = 0;

    std::string kf_tmp = "/keyframes/";
    std::string mp_tmp = "/mappoints/";
    char cstr0[path_name.size()+mp_tmp.size()+1];
    char cstr1[path_name.size()+kf_tmp.size()+1];
    strcpy(cstr0, (path_name+kf_tmp).c_str());
    strcpy(cstr1, (path_name+mp_tmp).c_str());

    // getting all filenames for the keyframes
    struct dirent *entry;
    DIR *dir = opendir(cstr0);

    if (dir == nullptr) {
        std::cout << "Directory is empty." << std::endl;
        return;
    }

    while ((entry = readdir(dir)) != nullptr) {

        if ( !strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..") ) {
            continue;
        } else {
            std::stringstream path;
            path << path_name+"/keyframes/";
            path << entry->d_name;
            filenames_kf.push_back(path.str());
        }
    }
    closedir(dir);

    // getting all filenames for the mappoints
    struct dirent *entry2;
    DIR *dir2 = opendir(cstr1);
    if (dir2 == nullptr) {
        std::cout << "Directory is empty." << std::endl;
        return;
    }

     while ((entry2 = readdir(dir2)) != nullptr) {

        if ( !strcmp(entry2->d_name, ".") || !strcmp(entry2->d_name, "..") ) {
            continue;
        } else {
            std::stringstream path;
            path << path_name+"/mappoints/";
            path << entry2->d_name;
            filenames_mp.push_back(path.str());
        }
    }
    closedir(dir);

    std::cout << "--> Loading Keyframes" << std::endl;
    for(unsigned long int i = 0; i < filenames_kf.size(); i++) {
        kfptr kf(new KeyFrame(voc,shared_from_this(),kfdb,comm,eSystemState::SERVER,uid->GetId()));
        std::stringstream buf;
        std::ifstream fs;
        fs.open(filenames_kf[i]);
        if(fs.is_open()) {
            buf << fs.rdbuf();
            cereal::BinaryInputArchive iarchive(buf);
            iarchive(*kf);
            keyframes.push_back(kf);
            if(kf->mId.second == 0) next_kf_id0 = max(next_kf_id0,kf->mId.first+1);
//            this->AddKeyFrame(kf);
            if(kf->mId.first == 0) mvpKeyFrameOrigins.push_back(kf);
            fs.close();
        } else {
            std::cout << filenames_kf[i] << std::endl;
            exit(-1);
        }
    }

    std::sort(keyframes.begin(),keyframes.end(),kftimecmpsmaller());

    std::cout << "--> Loading MapPoints" << std::endl;
    for(unsigned long int i = 0; i < filenames_mp.size(); i++) {
        mpptr mp(new MapPoint(shared_from_this(),comm,eSystemState::SERVER,uid->GetId()));
        std::stringstream buf;
        std::ifstream fs;
        fs.open(filenames_mp[i]);
        if(fs.is_open()) {
            buf << fs.rdbuf();
            cereal::BinaryInputArchive iarchive(buf);
            iarchive(*mp);
            mappoints.push_back(mp);
            if(mp->mId.second == 0) next_mp_id0 = max(next_mp_id0,mp->mId.first+1);
//            this->AddMapPoint(mp);
            fs.close();
        } else {
            std::cout << filenames_mp[i] << std::endl;
            exit(-1);
        }
    }

    std::cout << "Map consists of " << keyframes.size() << " keyframes" << std::endl;
    std::cout << "Map consists of " << mappoints.size() << " mappoints" << std::endl;

    // Re-Map if necessary
    for(auto kf : keyframes) {
        if(kf->mId.second != 0) {
            idpair new_id = make_pair(next_kf_id0++,0);
            saved_kf_ids_to_sys_ids[kf->mId] = new_id;
            kf->mId = new_id;
        }
    }
    for(auto lm : mappoints) {
        if(lm->mId.second != 0) {
            idpair new_id = make_pair(next_mp_id0++,0);
            saved_mp_ids_to_sys_ids[lm->mId] = new_id;
            lm->mId = new_id;
        }
    }
    // -------------------

    std::cout << "--> Building Connections" << std::endl;

    for(auto kf : keyframes) {
        this->AddKeyFrame(kf);
        kf->ProcessAfterLoad(saved_kf_ids_to_sys_ids);
        if(!this->msuAssClients.count(kf->mId.second))
            msuAssClients.insert(kf->mId.second);
    }

    std::cout << "----> Landmarks" << std::endl;
    for(auto lm : mappoints) {
        for(auto mit = lm->mmObservations_minimal.begin(); mit!=lm->mmObservations_minimal.end();++mit){
            size_t feat_id = mit->second;
            idpair kf_id = mit->first;

            if(kf_id.second != 0) {
                if(!saved_kf_ids_to_sys_ids.count(kf_id)) {
                    std::cout << COUTERROR << "ID ERROR" << std::endl;
                    exit(-1);
                }
                kf_id = saved_kf_ids_to_sys_ids[kf_id];
            }

            kfptr kf = this->GetKfPtr(kf_id);
            if(!kf){
                std::cout << COUTWARN << "cannot find KF" << std::endl;
                continue;
            }
            lm->AddObservation(kf,feat_id);
        }
        idpair kf_ref_id = lm->mRefKfId;

        if(kf_ref_id.second != 0) {
            if(!saved_kf_ids_to_sys_ids.count(kf_ref_id)) {
                std::cout << COUTERROR << "ID ERROR" << std::endl;
                exit(-1);
            }
            kf_ref_id = saved_kf_ids_to_sys_ids[kf_ref_id];
        }

        auto kf_ref = this->GetKfPtr(kf_ref_id);
        if(!kf_ref){
            std::cout << COUTWARN << "cannot find KF" << std::endl;
            continue;
        }
        lm->SetReferenceKeyFrame(kf_ref);
        lm->ComputeDistinctiveDescriptors();
        lm->UpdateNormalAndDepth();
        this->AddMapPoint(lm);
    }

    std::cout << "----> Keyframes" << std::endl;
    for(auto kf : keyframes) {
        for(auto mit = kf->mmMapPoints_minimal.begin(); mit!=kf->mmMapPoints_minimal.end();++mit){
            size_t feat_id = mit->first;
            idpair lm_id = mit->second;

            if(lm_id.second != 0) {
                if(!saved_mp_ids_to_sys_ids.count(lm_id)) {
                    std::cout << COUTERROR << "ID ERROR -- MP:" << lm_id.first << "|" << lm_id.second << std::endl;
                    exit(-1);
                }
                lm_id = saved_mp_ids_to_sys_ids[lm_id];
            }

            mpptr lm = this->GetMpPtr(lm_id);
            if(!lm) std::cout << COUTWARN << "requested LM " << lm_id.first << "|" << lm_id.second << " does not exist" << std::endl;
            kf->AddMapPoint(lm,feat_id);
        }
//        kf->ProcessAfterLoad();
        kf->UpdateConnections();
//        if(!kf->GetParent()) {
//            std::cout << COUTWARN << "KF " << kf->mId.first << "|" << kf->mId.second << " has no parent" << std::endl;
//        }
    }

    std::cout << "+++ DONE +++" << std::endl;
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);

    return mmpMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);

    return mmpKeyFrames.size();
}

vector<Map::mpptr> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

long unsigned int Map::GetMaxMPid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxMPid;
}

long unsigned int Map::GetMaxKFidUnique()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFidUnique;
}

long unsigned int Map::GetMaxMPidUnique()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxMPidUnique;
}

long unsigned int Map::GetLastKfIdUnique()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnLastKfIdUnique;
}

void Map::clear()
{
    mmpMapPoints.clear();
    mmpKeyFrames.clear();
    mmpErasedMapPoints.clear();
    mmpErasedKeyFrames.clear();
    mnMaxKFid = 0;
    mnMaxMPid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

Map::kfptr Map::GetKfPtr(size_t KfId, size_t ClientId, bool bIgnoreMutex) //Performance: find a better implementation for this method
{
    if(!bIgnoreMutex)
        unique_lock<mutex> lock(mMutexMap);

    idpair idp = make_pair(KfId,ClientId);
    std::map<idpair,kfptr>::iterator mit = mmpKeyFrames.find(idp);
    if(mit != mmpKeyFrames.end()) return mit->second;
    else return nullptr;
}

Map::kfptr Map::GetRandKfPtr()
{
    ccptr pCC = *(mspCC.begin());

    if(mnMaxKFid < (params::mapping::miNumRecentKFs))
        return nullptr;

    int cnt = 0;
    int MaxIts = 1;
    kfptr pKF;

    while(!pKF && cnt < MaxIts)
    {
        //KF ID
        size_t min = 0; //this KF is the query KF, it's neighbors are candidates for culling -- so 0 and 1 can be considered
        size_t max = mnMaxKFid;
        size_t id = min + (rand() % (size_t)(max - min + 1));

        //Client ID
        size_t cid = MAPRANGE;
        if(msuAssClients.size() > 1)
        {
            min = 0;
            max = msuAssClients.size() - 1;
            size_t temp = min + (rand() % (size_t)(max - min + 1));

            set<size_t>::iterator sit = msuAssClients.begin();
            for(int it = 0;it < msuAssClients.size();++it)
            {
                if(it == temp)
                {
                    cid = *sit;
                    break;
                }
                else
                    ++sit;
            }
        }
        else
        {
            cid = *(msuAssClients.begin());
        }

        pKF = this->GetKfPtr(id,cid);
        ++cnt;
    }

    return pKF;
}

Map::mpptr Map::GetMpPtr(size_t MpId, size_t ClientId)
{
    unique_lock<mutex> lock(mMutexMap);

    idpair idp = make_pair(MpId,ClientId);
    std::map<idpair,mpptr>::iterator mit = mmpMapPoints.find(idp);
    if(mit != mmpMapPoints.end()) return mit->second;
    else return nullptr;
}

Map::kfptr Map::GetErasedKfPtr(size_t KfId, size_t ClientId)
{
    unique_lock<mutex> lock(mMutexMap);

    idpair idp = make_pair(KfId,ClientId);
    std::map<idpair,kfptr>::iterator mit = mmpErasedKeyFrames.find(idp);
    if(mit != mmpErasedKeyFrames.end()) return mit->second;
    else return nullptr;
}

Map::mpptr Map::GetErasedMpPtr(size_t MpId, size_t ClientId)
{
    unique_lock<mutex> lock(mMutexMap);

    idpair idp = make_pair(MpId,ClientId);
    std::map<idpair,mpptr>::iterator mit = mmpErasedMapPoints.find(idp);
    if(mit != mmpErasedMapPoints.end()) return mit->second;
    else return nullptr;
}

bool Map::IsKfDeleted(size_t KfId, size_t ClientId)
{
    unique_lock<mutex> lock2(mMutexMap);
    unique_lock<mutex> lock(mMutexErased);

    idpair idp = make_pair(KfId,ClientId);
    std::map<idpair,kfptr>::iterator mit = mmpErasedKeyFrames.find(idp);
    if(mit != mmpErasedKeyFrames.end()) return true;
    else return false;
}

bool Map::IsMpDeleted(size_t MpId, size_t ClientId)
{
    unique_lock<mutex> lock2(mMutexMap);
    unique_lock<mutex> lock(mMutexErased);

    idpair idp = make_pair(MpId,ClientId);
    std::map<idpair,mpptr>::iterator mit = mmpErasedMapPoints.find(idp);
    if(mit != mmpErasedMapPoints.end()) return true;
    else return false;
}

void Map::AddCCPtr(ccptr pCC)
{
    unique_lock<mutex> lock(this->mMutexCC);
    mspCC.insert(pCC);

    if(pCC->mpCH->GetCommPtr()) //when this is called during init procedure, mpCH->GetCommPtr() is still nullptr
        mspComm.insert(pCC->mpCH->GetCommPtr());
}

set<Map::ccptr> Map::GetCCPtrs()
{
    unique_lock<mutex> lock(this->mMutexCC);
    return mspCC;
}

bool Map::kftimecmp::operator ()(const kfptr pA, const kfptr pB) const
{
    return pA->mdInsertStamp > pB->mdInsertStamp;
}

bool Map::kftimecmpsmaller::operator ()(const kfptr pA, const kfptr pB) const
{
    return pA->mdInsertStamp < pB->mdInsertStamp;
}

void Map::FindLocalKFsByTime(kfptr pKFcur, set<kfptr> &sKfVicinity, priority_queue<int> &pqNativeKFs, list<kfptr> &lForeignKFs, int nLocalKFs)
{
    set<kfptr,kftimecmp> spKFsort;

    for(std::map<idpair,kfptr>::iterator mit_set = mmpKeyFrames.begin();mit_set!=mmpKeyFrames.end();++mit_set)
        spKFsort.insert(mit_set->second);

    //keep n newest KFs
    {
        int nMax = spKFsort.size(); //cannot use 'spKFsort.size()' in loop header because size continuously reduces in loop itself
        for(int it=0;it<nMax;++it)
        {
            if(spKFsort.empty())
                break;

            kfptr pKFi = *(spKFsort.begin());
            spKFsort.erase(pKFi);

            if(!pKFi || pKFi->isBad())
                continue;

            sKfVicinity.insert(pKFi);

            if(pKFi->mId.second == mMapId)
                pqNativeKFs.push(pKFi->mId.first);
            else
                lForeignKFs.push_back(pKFi);

            if(sKfVicinity.size() >= nLocalKFs)
                break;
        }
    }
}

void Map::MapTrimming(kfptr pKFcur)
{
    unique_lock<mutex> lock(mMutexMap);
    unique_lock<mutex> lock2(mMutexErased);

    int nLocalKFs = params::mapping::miLocalMapSize;
    int KfLimit = params::mapping::miLocalMapSize + params::mapping::miLocalMapBuffer;

    if(mmpKeyFrames.size() <= nLocalKFs)
        return;

    set<kfptr> sKfVicinity;
    set<mpptr> sMpVicinity;
    sKfVicinity.insert(pKFcur);

    //for KF Limit
    priority_queue<int> pqNativeKFs;
    list<kfptr> lForeignKFs;

    pKFcur->UpdateConnections(true);

    this->FindLocalKFsByTime(pKFcur,sKfVicinity,pqNativeKFs,lForeignKFs,nLocalKFs);

    //add MPs included by the KFs
    for(set<kfptr>::iterator sit = sKfVicinity.begin();sit!=sKfVicinity.end();++sit)
    {
        kfptr pKFi = *sit;
        vector<mpptr> vMPs = pKFi->GetMapPointMatches();

        sMpVicinity.insert(vMPs.begin(),vMPs.end());
    }

    //find & erase KFs

    for(map<idpair,kfptr>::iterator mit = mmpKeyFrames.begin();mit!=mmpKeyFrames.end();)
    {
        kfptr pKFi = mit->second;
        bool bErase = false;

        if(!sKfVicinity.count(pKFi))
        {
            if(pKFi->CanBeForgotten() || pKFi->mId.second != this->mMapId || pKFi->isBad())
                bErase = true;
        }

        if(bErase)
        {
            if(!pKFi->isBad())
                pKFi->SetBadFlag(true);

            if(!pKFi->isBad())
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " SetBadFlag() called, but KF not set to bad" << endl;
                throw infrastructure_ex();
            }

            if(pKFi->mId.second == mMapId) mmpErasedKeyFrames[pKFi->mId] = pKFi;

            mit = mmpKeyFrames.erase(mit);
        }
        else
            ++mit;
    }

    //find & erase MPs
    for(map<idpair,mpptr>::iterator mit = mmpMapPoints.begin();mit!=mmpMapPoints.end();)
    {
        mpptr pMPi = mit->second;
        bool bErase = false;

        if(!sMpVicinity.count(pMPi))
        {
            if(pMPi->CanBeForgotten() || pMPi->mId.second != this->mMapId || pMPi->isBad())
                bErase = true;
        }

        if(bErase || pMPi->isBad())
        {
            if(!pMPi->isBad())
                pMPi->SetBadFlag(true);

            if(pMPi->mId.second == mMapId) mmpErasedMapPoints[pMPi->mId] = pMPi;

            mit = mmpMapPoints.erase(mit);

            if(mspMPsToErase.count(pMPi))
                mspMPsToErase.erase(pMPi);
        }
        else
        {
            ++mit;
        }
    }

    for(set<mpptr>::iterator sit = mspMPsToErase.begin();sit != mspMPsToErase.end();)
    {
        mpptr pMPi = *sit;

        if(pMPi)
        {
            std::map<idpair,mpptr>::iterator mit2 = mmpMapPoints.find(pMPi->mId);
            if(mit2 != mmpMapPoints.end())
                mmpMapPoints.erase(mit2);
        }

        sit = mspMPsToErase.erase(sit);
    }

    if(!mspMPsToErase.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " mspMPsToErase !empty()" << endl;
        throw infrastructure_ex();
    }

    //------------------------
    //--- Enforce KF Limit ---
    //------------------------

    if(mmpKeyFrames.size() > KfLimit)
    {
        cout << "+++++ Enforcing KF upper limit +++++" << ros::Time::now().toSec() << endl;

        set<kfptr,kftimecmpsmaller>spKFsmintime;
        for(std::map<idpair,kfptr>::iterator mit_set = mmpKeyFrames.begin();mit_set!=mmpKeyFrames.end();++mit_set)
            spKFsmintime.insert(mit_set->second);

        while(mmpKeyFrames.size() > KfLimit)
        {
            if(!lForeignKFs.empty())
            {
                kfptr pKFi = lForeignKFs.front();

                //can always be forgotten, comes from server
                if(pKFi->mId.second == mMapId)
                    mmpErasedKeyFrames[pKFi->mId] = pKFi;

                std::map<idpair,kfptr>::iterator mit = mmpKeyFrames.find(pKFi->mId);
                if(mit != mmpKeyFrames.end()) mmpKeyFrames.erase(mit);

                lForeignKFs.pop_front();

                spKFsmintime.erase(pKFi);
            }
            else if(!mmpKeyFrames.empty())
            {
                kfptr pKFi = *(spKFsmintime.begin());

                if(pKFi->mId.first == 0)
                {
                    //do nothing -- never delete 0
                    spKFsmintime.erase(pKFi);
                }
                else
                {
                    pKFi->SetBadFlag(true);

                    if(pKFi->mId.second == mMapId) mmpErasedKeyFrames[pKFi->mId] = pKFi;

                    std::map<idpair,kfptr>::iterator mit = mmpKeyFrames.find(pKFi->mId);
                    if(mit != mmpKeyFrames.end())
                        mmpKeyFrames.erase(mit);
                    else
                        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " MP ins mspKeyFrames, but not in mspKeyFrames" << endl;

                    spKFsmintime.erase(pKFi);
                }
            }
            else
                cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ": only current KF left, and the ones that cannot be erased" << endl;
        }

        //find MPs included by the KFs
        set<mpptr> spMPsToKeep;
        for(map<idpair,kfptr>::iterator mit = mmpKeyFrames.begin();mit!=mmpKeyFrames.end();++mit)
        {
            kfptr pKFi = mit->second;

            vector<mpptr> vMPs = pKFi->GetMapPointMatches();

            for(vector<mpptr>::iterator vit = vMPs.begin();vit!=vMPs.end();++vit)
            {
                mpptr pMPi = *vit;
                if(!pMPi || pMPi->isBad()) continue;

                spMPsToKeep.insert(pMPi);
            }
        }

        //delete other MPs
        for(map<idpair,mpptr>::iterator mit = mmpMapPoints.begin();mit!=mmpMapPoints.end();)
        {
            mpptr pMPi = mit->second;

            if(!spMPsToKeep.count(pMPi))
            {
                pMPi->SetBadFlag(true);

                if(pMPi->mId.second == mMapId) mmpErasedMapPoints[pMPi->mId] = pMPi;
                mit = mmpMapPoints.erase(mit);
            }
            else
                ++mit;
        }

//        std::cout << "KFs after erasing: " << mmpKeyFrames.size() << std::endl;
    }
}

void Map::PackVicinityToMsg(kfptr pKFcur, ccmslam_msgs::Map &msgMap, ccptr pCC)
{
    if(pKFcur->mId.first == 0)
        return; //we do not send the origin KFs
    //max == -1 means there is no upper limit
    int max = params::comm::server::miKfLimitToClient;

    unique_lock<mutex> lock(mMutexMap);
    unique_lock<mutex> lock2(mMutexErased);

    set<kfptr> sKfVicinity;
    set<mpptr> sMpVicinity;
    sKfVicinity.insert(pKFcur);
    set<kfptr> spKFsAddedLastIteration;
    spKFsAddedLastIteration.insert(pKFcur);

    pKFcur->UpdateConnections(true);

    //add KFs from CovGraph
    int depth = max;
    for(int it = 1;it<=depth;++it)
    {
        set<kfptr> spAdd; //do not insert into sKfVicinity while iterating it

        for(set<kfptr>::iterator sit = spKFsAddedLastIteration.begin();sit!=spKFsAddedLastIteration.end();++sit)
        {
            kfptr pKFi = *sit;
            if(pKFi->mId.first == 0) continue; //we do not send the origin KFs

            vector<KeyFrame::kfptr> vCovKfs = pKFi->GetVectorCovisibleKeyFrames();

            for(vector<kfptr>::iterator vit = vCovKfs.begin();vit!=vCovKfs.end();++vit)
            {
                kfptr pKFj = *vit;
                if(!pKFj || pKFj->isBad()) continue;
                if(pKFj->mId.first == 0) continue; //we do not send the origin KFs
                spAdd.insert(pKFj);

                if((max != -1) && (sKfVicinity.size() + spAdd.size()) >= max)
                {
                    break;
                }
            }

            if((max != -1) && (sKfVicinity.size() + spAdd.size()) >= max)
                break;
        }

        sKfVicinity.insert(spKFsAddedLastIteration.begin(),spKFsAddedLastIteration.end());
        spKFsAddedLastIteration = spAdd;

        if((max != -1) && (sKfVicinity.size() >= max))
            break;

        if(spKFsAddedLastIteration.empty())
            break;
    }

    //add MPs included by the KFs
    for(set<kfptr>::iterator sit = sKfVicinity.begin();sit!=sKfVicinity.end();++sit)
    {
        kfptr pKFi = *sit;
        vector<mpptr> vMPs = pKFi->GetMapPointMatches();

        for(vector<mpptr>::iterator vit = vMPs.begin();vit!=vMPs.end();++vit)
        {
            mpptr pMPi = *vit;
            if(!pMPi || pMPi->isBad()) continue;

            sMpVicinity.insert(pMPi);
        }
    }

    pKFcur->ConvertToMessage(msgMap,pCC->mg2oS_wcurmap_wclientmap,pKFcur); //make sure this is first KF in vector

    for(set<kfptr>::iterator sit = sKfVicinity.begin();sit!=sKfVicinity.end();++sit)
    {
        kfptr pKFi = *sit;

        if(pKFi->mId == pKFcur->mId)
            continue; //do not enter twice

        pKFi->ConvertToMessage(msgMap,pCC->mg2oS_wcurmap_wclientmap,pKFcur);
    }

    for(set<mpptr>::iterator sit = sMpVicinity.begin();sit != sMpVicinity.end();++sit)
    {
        mpptr pMPi = *sit;
        pMPi->ConvertToMessage(msgMap,pKFcur,pCC->mg2oS_wcurmap_wclientmap);
    }
}

void Map::HandleMissingParent(size_t QueryId, size_t QueryCId, Mat &T_cref_cquery, kfptr pRefKf = nullptr)
{
    //If a KF/MP gets a relative pos/pose to a KF from a msg, and it cannot find this KF, this method searches another KF that can be used

    idpair QId = make_pair(QueryId,QueryCId);
    kfptr pParent,pNewParent;

    unique_lock<mutex> lock2(mMutexMap);
    unique_lock<mutex> lock(mMutexErased);

    std::map<idpair,kfptr>::iterator mit = mmpErasedKeyFrames.find(QId);

    if(mit == mmpErasedKeyFrames.end())
    {
        //this would be a problem...
    }
    else
    {
        pParent = mit->second;
    }

    if(!pParent)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": Cannot find parent from msg" << endl;
        cout << "parent KF: " << QueryId << "|" << QueryCId << endl;
        cout << "KFs in map: " << mmpKeyFrames.size() << endl;
        throw estd::infrastructure_ex();
    }

    pNewParent = pParent->GetParent();
    cv::Mat Tcq_cnewparent = pParent->GetTcp();
    while(pNewParent->isBad())
    {
        if(!pNewParent)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": Cannot find parent from msg" << endl;
            throw estd::infrastructure_ex();
        }

        Tcq_cnewparent = Tcq_cnewparent * pNewParent->GetTcp();
        pNewParent = pNewParent->GetParent();
    }

    if(!pRefKf)
    {
        //called from constructor - needs RefKf
        pRefKf = pNewParent;
        T_cref_cquery = Tcq_cnewparent.inv();
    }
    else
    {
        //has already RefKf, an needs TF from parent specified in msg

        T_cref_cquery = pRefKf->GetPose() * pNewParent->GetPoseInverse() * Tcq_cnewparent.inv();
    }
}

Map::kfptr Map::GetPredecessor(kfptr pKF)
{
    kfptr pPred;
    size_t kfid = pKF->mId.first;
    while(!pPred)
    {
        kfid--;

        if(kfid == -1)
        {
            cout << "\033[1;31m!!!!! FATAL !!!!!\033[0m " << __func__ << __LINE__ << " cannot find predecessor" << endl;
            cout << "KF ID: " << pKF->mId.first << "|" << pKF->mId.second << endl;
            cout << "In map: " << this->mnMaxKFid << endl;
            cout << "Workaround: take first KF (id 0)" << endl;
            pPred = this->GetKfPtr(0,mMapId);
            cout << "get KF: 0|" << mMapId <<" -- nullptr? " << (int)!pPred << endl;
        }
        else
        {
            pPred = this->GetKfPtr(kfid,pKF->mId.second);
        }
    }

    return pPred;
}

void Map::ClearBadMPs()
{
    unique_lock<mutex> lock(mMutexMap);
    unique_lock<mutex> lock2(mMutexErased);

    for(set<mpptr>::iterator sit = mspMPsToErase.begin();sit != mspMPsToErase.end();)
    {
        mpptr pMPi = *sit;

        if(pMPi)
        {
            map<idpair,mpptr>::iterator mit2 = mmpMapPoints.find(pMPi->mId);
            if(mit2 != mmpMapPoints.end())
                mmpMapPoints.erase(mit2);
        }

        sit = mspMPsToErase.erase(sit);
    }
}

Map::ccptr Map::GetCCPtr(size_t nClientId)
{
    for(set<ccptr>::iterator sit = mspCC.begin();sit != mspCC.end(); ++sit)
    {
        ccptr pCC = *sit;
        if(pCC->mClientId == nClientId)
            return pCC;
    }

    cout << COUTERROR << "no ccptr found for query-ID" << endl;
    return nullptr;
}

void Map::StopGBA()
{
    {
        //prevent two handlers from stopping GBA at the same time
        unique_lock<mutex> lockStopGBAInProgress(mMutexStopGBAInProgess);
        if(mbStopGBAInProgress)
            return;
        else
            mbStopGBAInProgress = true;
    }

    cout << "Map " << mMapId << ": Stop GBA" << endl;
    if(this->isRunningGBA())
    {
        #ifdef DONOTINTERRUPTMERGE
        if(this->isMergeStepGBA())
        {
            cout << "Map " << mMapId << ": GBA stop declined -- MergeGBA" << endl;
            unique_lock<mutex> lockStopGBAInProgress(mMutexStopGBAInProgess);
            mbStopGBAInProgress = false;
            return;
        }
        #endif

        this->mbStopGBA = true;

        while(!this->isFinishedGBA())
            usleep(5000);

        this->mpThreadGBA->join();
        delete this->mpThreadGBA;
    }
    else
    {
        cout << COUTERROR << "called w/o GBA running -- Map " << mMapId << endl;
    }

    {
        unique_lock<mutex> lockStopGBAInProgress(mMutexStopGBAInProgess);
        mbStopGBAInProgress = false;
    }
    cout << "Map " << mMapId << ": GBA Stopped" << endl;
}

void Map::RequestBA(size_t nClientId)
{
    if(this->isRunningGBA())
    {
        cout << "Denied -- GBA running" << endl;
        return;
    }

    if(this->isNoStartGBA())
    {
        cout << "Denied -- NoStartGBA" << endl;
        return;
    }

    #ifdef FINALBA
    if(!this->isGBAinterrupted())
        cout << COUTERROR << "Agent " << nClientId << " requesting BA, but was not interrupted" << endl;
    #endif

    msnFinishedAgents.insert(nClientId);

    if(msnFinishedAgents.size() == msuAssClients.size())
    {
        bool b0 = false;
        bool b1 = false;
        bool b2 = false;
        bool b3 = false;

        for(set<ccptr>::iterator sit = mspCC.begin();sit!=mspCC.end();++sit)
        {
            ccptr pCC = *sit;

            if(pCC->mClientId > 3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId out of bounds (" << pCC->mClientId << ")" << endl;
            if(!(this->msuAssClients.count(pCC->mClientId))) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId in pCC but not in msuAssClients" << endl;
            switch(pCC->mClientId)
            {
                case(static_cast<size_t>(0)):
                    if(b0) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b0 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                case(static_cast<size_t>(1)):
                    if(b1) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b1 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                case(static_cast<size_t>(2)):
                    if(b2) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b2 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                case(static_cast<size_t>(3)):
                    if(b3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b3 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                default: cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId out of bounds" << endl;
            }

            pCC->mbOptActive = true;
        }

        idpair nLoopKF = make_pair(mnLastKfIdUnique,mMapId);

        this->setRunningGBA();
        this->setFinishedGBA();
        this->mbStopGBA = false;

        #ifdef DEBUGGING2
        this->CheckStructure();
        #endif

        // add this code (2023/12/5) ///////////////////////////////////////////////////////
        int check_motion_violate_KF_switch = 1; // 1:ON / 0:OFF

        if (check_motion_violate_KF_switch == 1) {
            /*
            for(int it=0;it<4;++it)
            {
                std::stringstream ss;
                ss << params::stats::msOutputDir << "KF_FINAL_" << it << "_beforeMC.csv";
                this->WriteStateToCsv(ss.str(),it);
            }
            std::cout << "KFs in Map (before Motion Check) : " << mmpKeyFrames.size() << endl;
            this->CheckMotionViolateKF_before(0);
            std::cout << "KFs in Map (after Motion Check) : " << mmpKeyFrames.size() << endl;
            std::cout << "Map: Check Motion Violate KF (before GBA) finished - continue" << std::endl;
            */
            for(int it=0;it<4;++it) // modified (2023/11/23)
            {
                std::stringstream ss;
                ss << params::stats::msOutputDir << "KF_FINAL_" << it << "_before.csv";
                this->WriteStateToCsv(ss.str(),it);
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////

        // Launch a new thread to perform Global Bundle Adjustment
        this->mpThreadGBA = new thread(&Map::RunGBA,this,nLoopKF);

        std::cout << "Map: Wait for GBA to finish" << std::endl;
        while(this->isRunningGBA()) {
            usleep(10000);
        }
        std::cout << "Map: GBA finished - continue" << std::endl;
        
        // add this code (2023/12/5) ///////////////////////////////////////////////////////
        /*
        if (check_motion_violate_KF_switch == 1) {
            for(int it=0;it<4;++it) // modified (2023/11/23)
            {
                std::stringstream ss;
                ss << params::stats::msOutputDir << "KF_FINAL_" << it << "_afterBA.csv";
                this->WriteStateToCsv(ss.str(),it);
            }
            std::cout << "KFs in Map (before Motion Check) : " << mmpKeyFrames.size() << endl;
            this->CheckMotionViolateKF_after(0);
            std::cout << "KFs in Map (after Motion Check) : " << mmpKeyFrames.size() << endl;
            std::cout << "Map: Check Motion Violate KF (after GBA) finished - continue" << std::endl;
            for(int it=0;it<4;++it)
            {
                std::stringstream ss;
                ss << params::stats::msOutputDir << "KF_FINAL_" << it << "_afterMC.csv";
                this->WriteStateToCsv(ss.str(),it);
            }
        }
        */

        for(int it=0;it<4;++it) // modified (2023/11/23)
        {
            std::stringstream ss;
            ss << params::stats::msOutputDir << "KF_FINAL_" << it << "_after.csv";
            this->WriteStateToCsv(ss.str(),it);
        }
        /////////////////////////////////////////////////////////////////////////////////////
    }
    else
    {
        cout << "msuAssClient: " << endl;
        for(set<size_t>::iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit;
        cout << endl;

        cout << "msuFinishedAgents: " << endl;
        for(set<size_t>::iterator sit = msnFinishedAgents.begin();sit!=msnFinishedAgents.end();++sit)
            cout << *sit;
        cout << endl;
    }
}

void Map::RunGBA(idpair nLoopKF)
{
    cout << "-> Starting Global Bundle Adjustment" << endl;

    // add this code (2023/12/11) ////////////////////////////////////////////////////////
    /*
    this->CheckMotionViolateKF_before(0);

    for(int it=0;it<4;++it)
    {
        std::stringstream ss;
        ss << params::stats::msOutputDir << "KF_FINAL_" << it << "_beforeBA.csv";
        this->WriteStateToCsv(ss.str(),it);
    }
    */
    //////////////////////////////////////////////////////////////////////////////////////

    /*
    // add this code (2023/12/4) //////////////////////////////////////////////////////////////////////////////////////////////
    map_gba_counter ++;

    for(int it=0;it<4;++it)
    {
        std::stringstream ss;
        ss << params::stats::msOutputDir << "KF_GBA_" << it << "_MP" << map_gba_counter << "_before.csv";
        this->WriteStateToCsv(ss.str(),it);
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    */
   this->SetTcwCurrent(0, mmpKeyFrames);

    Optimizer::MapFusionGBA(shared_from_this(),this->mMapId,params::opt::mGBAIterations,&(this->mbStopGBA),nLoopKF,true);

    #ifdef FINALBA
    if(!this->mbStopGBA)
    #endif
    {
        unique_lock<mutex> lock(this->mMutexGBA);

        this->LockMapUpdate();

        cout << "-> Global Bundle Adjustment finished" << endl;
        cout << "-> Updating map ..." << endl;

        // Correct keyframes starting at map first keyframe
        list<kfptr> lpKFtoCheck(this->mvpKeyFrameOrigins.begin(),this->mvpKeyFrameOrigins.end());

        #ifdef DEBUGGING2
        std::cout << "Map Origins: " << std::endl;
        for(list<kfptr>::iterator litcheck=lpKFtoCheck.begin();litcheck!=lpKFtoCheck.end();++litcheck)
        {
            kfptr pKFcheck = *litcheck;
            std::cout << "KF " << pKFcheck->mId.first << "|" << pKFcheck->mId.second << std::endl;
        }
        #endif

        cout << "--> Updating KFs ..." << endl;

        while(!lpKFtoCheck.empty())
        {
            kfptr pKF = lpKFtoCheck.front();
            const set<kfptr> sChilds = pKF->GetChilds();
            cv::Mat Twc = pKF->GetPoseInverse();
            for(set<kfptr>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
            {
                kfptr pChild = *sit;
                if(pChild->mBAGlobalForKF!=nLoopKF)
                {
                    cv::Mat Tchildc = pChild->GetPose()*Twc;
                    pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                    #ifdef DEBUGGING2
                    if(!(pChild->mTcwGBA.dims >= 2))
                        std::cout << COUTERROR << " KF" << pChild->mId.first << "|" << pChild->mId.second << ": !(pChild->mTcwGBA.dims >= 2)" << std::endl;
                    #endif
                    pChild->mBAGlobalForKF=nLoopKF;

                }
                lpKFtoCheck.push_back(pChild);
            }

            #ifdef DEBUGGING2
            if(!(pKF->mTcwGBA.dims >= 2))
                std::cout << COUTERROR << " KF" << pKF->mId.first << "|" << pKF->mId.second << ": !(pKF->mTcwGBA.dims >= 2)" << std::endl;
            #endif

            pKF->mTcwBefGBA = pKF->GetPose();
            #ifdef DEBUGGING2
            if(!(pKF->mTcwBefGBA.dims >= 2))
                std::cout << COUTERROR << " KF" << pKF->mId.first << "|" << pKF->mId.second << ": !(pKF->mTcwBefGBA.dims >= 2)" << std::endl;
            #endif
            pKF->SetPose(pKF->mTcwGBA,true);
            pKF->mbLoopCorrected = true;
            lpKFtoCheck.pop_front();
        }

        cout << "--> Updating MPs ..." << endl;

        // Correct MapPoints
        const vector<mpptr> vpMPs = this->GetAllMapPoints();

        for(size_t i=0; i<vpMPs.size(); i++)
        {
            mpptr pMP = vpMPs[i];

            if(pMP->isBad())
                continue;

            if(pMP->mBAGlobalForKF==nLoopKF)
            {
                // If optimized by Global BA, just update
                #ifdef DEBUGGING2
                if(!(pMP->mPosGBA.dims >= 2))
                    std::cout << COUTERROR << " MP" << pMP->mId.first << "|" << pMP->mId.second << ": !(pMP->mPosGBA.dims >= 2)" << std::endl;
                #endif
                pMP->SetWorldPos(pMP->mPosGBA,true);
                pMP->mbLoopCorrected = true;
            }
            else
            {
                // Update according to the correction of its reference keyframe
                kfptr pRefKF = pMP->GetReferenceKeyFrame();

                if(!pRefKF)
                {
                    cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": pRefKf is nullptr" << endl;
                    continue;
                }

                if(pRefKF->mBAGlobalForKF!=nLoopKF)
                    continue;

                #ifdef DEBUGGING2
                if(!(pRefKF->mTcwBefGBA.dims >= 2))
                {
                    std::cout << COUTERROR << " KF" << pRefKF->mId.first << "|" << pRefKF->mId.second << ": !(pRefKF->mTcwBefGBA.dims >= 2)" << " -- bad: " << (int)pRefKF->isBad() << std::endl;
                    std::cout << "bad? " << (int)pRefKF->isBad() << std::endl;
                    std::cout << "mBAGlobalForKF: " << pRefKF->mBAGlobalForKF.first << "|" << pRefKF->mBAGlobalForKF.second << std::endl;
                    std::cout << "nLoopKF: " << nLoopKF.first << "|" << nLoopKF.second << std::endl;
//                    kfptr pKFp = pRefKF->GetParent();
//                    std::cout << "Parent " << pKFp->mId.first << "|" << pKFp->mId.second << " -- bad: " << (int)pKFp->isBad() << std::endl;

                    std::cout << "KF Lineage: " << std::endl;
                    kfptr pKFp = pRefKF->GetParent();
                    while(pKFp)
                    {
                        std::cout << "--> " << pKFp->mId.first << "|" << pKFp->mId.second << " -- bad: " << (int)pKFp->isBad() << std::endl;
                        if(pKFp == pKFp->GetParent())
                        {
                            std::cout << "--> " << pKFp->mId.first << "|" << pKFp->mId.second << " -- bad: " << (int)pKFp->isBad() << std::endl;
                            break;
                        }
                        pKFp = pKFp->GetParent();
                    }

                    pRefKF->mTcwBefGBA = pRefKF->GetPose();
                }
                #endif

                // Map to non-corrected camera
                cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                // Backproject using corrected camera
                cv::Mat Twc = pRefKF->GetPoseInverse();
                cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                cv::Mat twc = Twc.rowRange(0,3).col(3);

                pMP->SetWorldPos(Rwc*Xc+twc,true);
                pMP->mbLoopCorrected = true;
            }
        }

        cout << "-> Map updated!" << endl;

        /*
        for(int it=0;it<4;++it)
        {
            std::stringstream ss;
            ss << params::stats::msOutputDir << "KF_FINAL_" << it << "_afterBA.csv";
            this->WriteStateToCsv(ss.str(),it);
        }
        */
        // add this code (2023/12/11) ////////////////////////////////////////////////////////
        //this->CheckMotionViolateKF_after(0);
        //this->CorrectKF(0, true);

        std::chrono::system_clock::time_point start, end;
        std::time_t time_stamp;
        ofstream ofs;
        start = std::chrono::system_clock::now();

        this->SetTcwNew(0, mmpKeyFrames);
        //this->CorrectKF_V2(0, mmpKeyFrames, true, false);
        this->CorrectKF_V3(0, mmpKeyFrames, true, false);
        //this->CorrectKF_V2(0, mmpKeyFrames, true, true);
        this->CorrectKF_V3(0, mmpKeyFrames, true, true);
        end = std::chrono::system_clock::now();
        auto time = end - start;
        auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(time).count();
        ofs.open("/home/muto/ccmslam_ws/src/ccm_slam/cslam/output/output_MC_MAPtime.txt");
        ofs << msec << "msec\n" << endl;
        ofs.close();
        //////////////////////////////////////////////////////////////////////////////////////

        #ifdef FINALBA
        this->unsetGBAinterrupted();
        #endif

        this->UnLockMapUpdate();
    }
    #ifdef FINALBA
    else
    {
        cout << COUTNOTICE << "GBA interrupted" << endl;
        this->setGBAinterrupted();
    }
    #endif
    /* IMPORTNANT CHANGE
    if(params::stats::mbWriteKFsToFile)
    {
        for(int it=0;it<4;++it)
        {
            std::stringstream ss;
            ss << params::stats::msOutputDir << "KF_GBA_" << it << "_MP" << map_gba_counter << "_after.csv";
            // add this code (2023/11/21) /////////////////////////////////////////////////////////////////////////
            std::stringstream ss_trimmed;
            std::stringstream ss_trimmed_v2;
            std::stringstream ss_trimmed_v3;
            std::stringstream ss_trimmed_v4;
            ss_trimmed << params::stats::msOutputDir << "KF_GBA_TRIMMED_" << it << ".csv";
            ss_trimmed_v2 << params::stats::msOutputDir << "KF_GBA_TRIMMED_V2_" << it << ".csv";
            ss_trimmed_v3 << params::stats::msOutputDir << "KF_GBA_TRIMMED_V3_" << it << ".csv";
            ss_trimmed_v4 << params::stats::msOutputDir << "KF_GBA_TRIMMED_V4_" << it << ".csv";
            ///////////////////////////////////////////////////////////////////////////////////////////////////////
            this->WriteStateToCsv(ss.str(),it);
            // add this code (2023/11/21) /////////////////////////////////////////////////////////////////////////
            if (WriteStateToCsvTrimmed_switch == 1) {
                //this->WriteStateToCsvTrimmed(ss_trimmed.str(), it);
                this->WriteStateToCsvTrimmedV2(ss_trimmed_v2.str(), it);
                this->WriteStateToCsvTrimmedV3(ss_trimmed_v3.str(), it);
                this->WriteStateToCsvTrimmedV4(ss_trimmed_v4.str(), it);
            }
            ///////////////////////////////////////////////////////////////////////////////////////////////////////
        }
    }
    */

    this->setFinishedGBA();
    this->unsetRunningGBA();

    for(set<ccptr>::iterator sit = mspCC.begin();sit!=mspCC.end();++sit)
    {
        ccptr pCC = *sit;
        pCC->UnLockMapping();

        pCC->mbOptActive = false;
    }

    cout << "-> Leave Thread" << endl;
}

// add this code (2023/12/14)
void Map::RequestBA_fromCorrectKF(size_t nClientId)
{
    if(this->isRunningGBA())
    {
        cout << "Denied -- GBA running" << endl;
        return;
    }

    /*
    if(this->isNoStartGBA())
    {
        cout << "Denied -- NoStartGBA" << endl;
        return;
    }

    #ifdef FINALBA
    if(!this->isGBAinterrupted())
        cout << COUTERROR << "Agent " << nClientId << " requesting BA, but was not interrupted" << endl;
    #endif
    */

    msnFinishedAgents.insert(nClientId);

    if(msnFinishedAgents.size() == msuAssClients.size())
    {
        bool b0 = false;
        bool b1 = false;
        bool b2 = false;
        bool b3 = false;

        for(set<ccptr>::iterator sit = mspCC.begin();sit!=mspCC.end();++sit)
        {
            ccptr pCC = *sit;

            if(pCC->mClientId > 3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId out of bounds (" << pCC->mClientId << ")" << endl;
            if(!(this->msuAssClients.count(pCC->mClientId))) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId in pCC but not in msuAssClients" << endl;
            switch(pCC->mClientId)
            {
                case(static_cast<size_t>(0)):
                    if(b0) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b0 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                case(static_cast<size_t>(1)):
                    if(b1) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b1 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                case(static_cast<size_t>(2)):
                    if(b2) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b2 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                case(static_cast<size_t>(3)):
                    if(b3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b3 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                default: cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId out of bounds" << endl;
            }

            pCC->mbOptActive = true;
        }

        idpair nLoopKF = make_pair(mnLastKfIdUnique,mMapId);

        this->setRunningGBA();
        this->setFinishedGBA();
        this->mbStopGBA = false;

        #ifdef DEBUGGING2
        this->CheckStructure();
        #endif

        // add this code (2023/12/5) ///////////////////////////////////////////////////////
        int check_motion_violate_KF_switch = 1; // 1:ON / 0:OFF

        if (check_motion_violate_KF_switch == 1) {
            /*
            for(int it=0;it<4;++it)
            {
                std::stringstream ss;
                ss << params::stats::msOutputDir << "KF_FINAL_" << it << "_beforeMC.csv";
                this->WriteStateToCsv(ss.str(),it);
            }
            std::cout << "KFs in Map (before Motion Check) : " << mmpKeyFrames.size() << endl;
            this->CheckMotionViolateKF_before(0);
            std::cout << "KFs in Map (after Motion Check) : " << mmpKeyFrames.size() << endl;
            std::cout << "Map: Check Motion Violate KF (before GBA) finished - continue" << std::endl;
            */
            for(int it=0;it<4;++it) // modified (2023/11/23)
            {
                std::stringstream ss;
                ss << params::stats::msOutputDir << "KF_reGBA_" << it << "_before.csv";
                this->WriteStateToCsv(ss.str(),it);
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////

        // Launch a new thread to perform Global Bundle Adjustment
        this->mpThreadGBA = new thread(&Map::RunGBA_fromCorrectKF,this,nLoopKF);

        std::cout << "Map: Wait for GBA to finish" << std::endl;
        while(this->isRunningGBA()) {
            usleep(10000);
        }
        std::cout << "Map: GBA finished - continue" << std::endl;
        
        // add this code (2023/12/5) ///////////////////////////////////////////////////////
        /*
        if (check_motion_violate_KF_switch == 1) {
            for(int it=0;it<4;++it) // modified (2023/11/23)
            {
                std::stringstream ss;
                ss << params::stats::msOutputDir << "KF_FINAL_" << it << "_afterBA.csv";
                this->WriteStateToCsv(ss.str(),it);
            }
            std::cout << "KFs in Map (before Motion Check) : " << mmpKeyFrames.size() << endl;
            this->CheckMotionViolateKF_after(0);
            std::cout << "KFs in Map (after Motion Check) : " << mmpKeyFrames.size() << endl;
            std::cout << "Map: Check Motion Violate KF (after GBA) finished - continue" << std::endl;
            for(int it=0;it<4;++it)
            {
                std::stringstream ss;
                ss << params::stats::msOutputDir << "KF_FINAL_" << it << "_afterMC.csv";
                this->WriteStateToCsv(ss.str(),it);
            }
        }
        */

        for(int it=0;it<4;++it) // modified (2023/11/23)
        {
            std::stringstream ss;
            ss << params::stats::msOutputDir << "KF_reGBA_" << it << "_after.csv";
            this->WriteStateToCsv(ss.str(),it);
        }
        /////////////////////////////////////////////////////////////////////////////////////
    }
    else
    {
        cout << "msuAssClient: " << endl;
        for(set<size_t>::iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit;
        cout << endl;

        cout << "msuFinishedAgents: " << endl;
        for(set<size_t>::iterator sit = msnFinishedAgents.begin();sit!=msnFinishedAgents.end();++sit)
            cout << *sit;
        cout << endl;
    }
}


void Map::RunGBA_fromCorrectKF(idpair nLoopKF)
{
    cout << "-> Starting Global Bundle Adjustment" << endl;

    // add this code (2023/12/11) ////////////////////////////////////////////////////////
    /*
    this->CheckMotionViolateKF_before(0);

    for(int it=0;it<4;++it)
    {
        std::stringstream ss;
        ss << params::stats::msOutputDir << "KF_FINAL_" << it << "_beforeBA.csv";
        this->WriteStateToCsv(ss.str(),it);
    }
    */
    //////////////////////////////////////////////////////////////////////////////////////

    /*
    // add this code (2023/12/4) //////////////////////////////////////////////////////////////////////////////////////////////
    map_gba_counter ++;

    for(int it=0;it<4;++it)
    {
        std::stringstream ss;
        ss << params::stats::msOutputDir << "KF_GBA_" << it << "_MP" << map_gba_counter << "_before.csv";
        this->WriteStateToCsv(ss.str(),it);
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    */

    Optimizer::MapFusionGBA(shared_from_this(),this->mMapId,params::opt::mGBAIterations,&(this->mbStopGBA),nLoopKF,true);
    /*
    #ifdef FINALBA
    if(!this->mbStopGBA)
    #endif
    */
    {
        unique_lock<mutex> lock(this->mMutexGBA);

        this->LockMapUpdate();

        cout << "-> Global Bundle Adjustment finished" << endl;
        cout << "-> Updating map ..." << endl;

        // Correct keyframes starting at map first keyframe
        list<kfptr> lpKFtoCheck(this->mvpKeyFrameOrigins.begin(),this->mvpKeyFrameOrigins.end());

        #ifdef DEBUGGING2
        std::cout << "Map Origins: " << std::endl;
        for(list<kfptr>::iterator litcheck=lpKFtoCheck.begin();litcheck!=lpKFtoCheck.end();++litcheck)
        {
            kfptr pKFcheck = *litcheck;
            std::cout << "KF " << pKFcheck->mId.first << "|" << pKFcheck->mId.second << std::endl;
        }
        #endif

        cout << "--> Updating KFs ..." << endl;

        while(!lpKFtoCheck.empty())
        {
            kfptr pKF = lpKFtoCheck.front();
            const set<kfptr> sChilds = pKF->GetChilds();
            cv::Mat Twc = pKF->GetPoseInverse();
            for(set<kfptr>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
            {
                kfptr pChild = *sit;
                if(pChild->mBAGlobalForKF!=nLoopKF)
                {
                    cv::Mat Tchildc = pChild->GetPose()*Twc;
                    pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                    #ifdef DEBUGGING2
                    if(!(pChild->mTcwGBA.dims >= 2))
                        std::cout << COUTERROR << " KF" << pChild->mId.first << "|" << pChild->mId.second << ": !(pChild->mTcwGBA.dims >= 2)" << std::endl;
                    #endif
                    pChild->mBAGlobalForKF=nLoopKF;

                }
                lpKFtoCheck.push_back(pChild);
            }

            #ifdef DEBUGGING2
            if(!(pKF->mTcwGBA.dims >= 2))
                std::cout << COUTERROR << " KF" << pKF->mId.first << "|" << pKF->mId.second << ": !(pKF->mTcwGBA.dims >= 2)" << std::endl;
            #endif

            pKF->mTcwBefGBA = pKF->GetPose();
            #ifdef DEBUGGING2
            if(!(pKF->mTcwBefGBA.dims >= 2))
                std::cout << COUTERROR << " KF" << pKF->mId.first << "|" << pKF->mId.second << ": !(pKF->mTcwBefGBA.dims >= 2)" << std::endl;
            #endif
            pKF->SetPose(pKF->mTcwGBA,true);
            pKF->mbLoopCorrected = true;
            lpKFtoCheck.pop_front();
        }

        cout << "--> Updating MPs ..." << endl;

        // Correct MapPoints
        const vector<mpptr> vpMPs = this->GetAllMapPoints();

        for(size_t i=0; i<vpMPs.size(); i++)
        {
            mpptr pMP = vpMPs[i];

            if(pMP->isBad())
                continue;

            if(pMP->mBAGlobalForKF==nLoopKF)
            {
                // If optimized by Global BA, just update
                #ifdef DEBUGGING2
                if(!(pMP->mPosGBA.dims >= 2))
                    std::cout << COUTERROR << " MP" << pMP->mId.first << "|" << pMP->mId.second << ": !(pMP->mPosGBA.dims >= 2)" << std::endl;
                #endif
                pMP->SetWorldPos(pMP->mPosGBA,true);
                pMP->mbLoopCorrected = true;
            }
            else
            {
                // Update according to the correction of its reference keyframe
                kfptr pRefKF = pMP->GetReferenceKeyFrame();

                if(!pRefKF)
                {
                    cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": pRefKf is nullptr" << endl;
                    continue;
                }

                if(pRefKF->mBAGlobalForKF!=nLoopKF)
                    continue;

                #ifdef DEBUGGING2
                if(!(pRefKF->mTcwBefGBA.dims >= 2))
                {
                    std::cout << COUTERROR << " KF" << pRefKF->mId.first << "|" << pRefKF->mId.second << ": !(pRefKF->mTcwBefGBA.dims >= 2)" << " -- bad: " << (int)pRefKF->isBad() << std::endl;
                    std::cout << "bad? " << (int)pRefKF->isBad() << std::endl;
                    std::cout << "mBAGlobalForKF: " << pRefKF->mBAGlobalForKF.first << "|" << pRefKF->mBAGlobalForKF.second << std::endl;
                    std::cout << "nLoopKF: " << nLoopKF.first << "|" << nLoopKF.second << std::endl;
//                    kfptr pKFp = pRefKF->GetParent();
//                    std::cout << "Parent " << pKFp->mId.first << "|" << pKFp->mId.second << " -- bad: " << (int)pKFp->isBad() << std::endl;

                    std::cout << "KF Lineage: " << std::endl;
                    kfptr pKFp = pRefKF->GetParent();
                    while(pKFp)
                    {
                        std::cout << "--> " << pKFp->mId.first << "|" << pKFp->mId.second << " -- bad: " << (int)pKFp->isBad() << std::endl;
                        if(pKFp == pKFp->GetParent())
                        {
                            std::cout << "--> " << pKFp->mId.first << "|" << pKFp->mId.second << " -- bad: " << (int)pKFp->isBad() << std::endl;
                            break;
                        }
                        pKFp = pKFp->GetParent();
                    }

                    pRefKF->mTcwBefGBA = pRefKF->GetPose();
                }
                #endif

                // Map to non-corrected camera
                cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                // Backproject using corrected camera
                cv::Mat Twc = pRefKF->GetPoseInverse();
                cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                cv::Mat twc = Twc.rowRange(0,3).col(3);

                pMP->SetWorldPos(Rwc*Xc+twc,true);
                pMP->mbLoopCorrected = true;
            }
        }

        cout << "-> Map updated!" << endl;

        for(int it=0;it<4;++it)
        {
            std::stringstream ss;
            ss << params::stats::msOutputDir << "KF_reGBA_" << it << "_afterBA.csv";
            this->WriteStateToCsv(ss.str(),it);
        }
        // add this code (2023/12/11) ////////////////////////////////////////////////////////
        //this->CheckMotionViolateKF_after(0);
        //this->CorrectKF(0, true);
        //this->CorrectKF_V2(0, mmpKeyFrames, true, false);
        //////////////////////////////////////////////////////////////////////////////////////

        /*
        #ifdef FINALBA
        this->unsetGBAinterrupted();
        #endif
        */

        //this->UnLockMapUpdate();
    }
    /*
    #ifdef FINALBA
    else
    {
        cout << COUTNOTICE << "GBA interrupted" << endl;
        this->setGBAinterrupted();
    }
    #endif
    */

    /* IMPORTNANT CHANGE
    if(params::stats::mbWriteKFsToFile)
    {
        for(int it=0;it<4;++it)
        {
            std::stringstream ss;
            ss << params::stats::msOutputDir << "KF_GBA_" << it << "_MP" << map_gba_counter << "_after.csv";
            // add this code (2023/11/21) /////////////////////////////////////////////////////////////////////////
            std::stringstream ss_trimmed;
            std::stringstream ss_trimmed_v2;
            std::stringstream ss_trimmed_v3;
            std::stringstream ss_trimmed_v4;
            ss_trimmed << params::stats::msOutputDir << "KF_GBA_TRIMMED_" << it << ".csv";
            ss_trimmed_v2 << params::stats::msOutputDir << "KF_GBA_TRIMMED_V2_" << it << ".csv";
            ss_trimmed_v3 << params::stats::msOutputDir << "KF_GBA_TRIMMED_V3_" << it << ".csv";
            ss_trimmed_v4 << params::stats::msOutputDir << "KF_GBA_TRIMMED_V4_" << it << ".csv";
            ///////////////////////////////////////////////////////////////////////////////////////////////////////
            this->WriteStateToCsv(ss.str(),it);
            // add this code (2023/11/21) /////////////////////////////////////////////////////////////////////////
            if (WriteStateToCsvTrimmed_switch == 1) {
                //this->WriteStateToCsvTrimmed(ss_trimmed.str(), it);
                this->WriteStateToCsvTrimmedV2(ss_trimmed_v2.str(), it);
                this->WriteStateToCsvTrimmedV3(ss_trimmed_v3.str(), it);
                this->WriteStateToCsvTrimmedV4(ss_trimmed_v4.str(), it);
            }
            ///////////////////////////////////////////////////////////////////////////////////////////////////////
        }
    }
    */

    this->setFinishedGBA();
    this->unsetRunningGBA();

    for(set<ccptr>::iterator sit = mspCC.begin();sit!=mspCC.end();++sit)
    {
        ccptr pCC = *sit;
        pCC->UnLockMapping();

        pCC->mbOptActive = false;
    }

    cout << "-> Leave Thread" << endl;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

// add this code (2023/12/5) //////////////////////////////////////////////////////////////////////////
void Map::CheckMotionViolateKF_before(const size_t clientId) {
  std::vector<kfptr> foundKFs;
  foundKFs.reserve(mmpKeyFrames.size());
  // Get all frames from the required client
  for (std::map<idpair,kfptr>::const_iterator itr = mmpKeyFrames.begin();
       itr != mmpKeyFrames.end(); ++itr) {
    kfptr pKF = itr->second;
    idpair currID = itr->first;
    if (currID.second == clientId) {
      foundKFs.push_back(pKF);
    }
  }
  if(foundKFs.empty()) //would overwrite files from other maps with empty files
    return;

  // Sort the keyframes by timestamp
  std::sort(foundKFs.begin(), foundKFs.end(), KeyFrame::compKFstamp);

  // Check if KF violate motion
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
    kfptr pKF = (*itr);
    const cv::Mat T_wc = pKF->GetPoseInverse();
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
    const Eigen::Quaterniond q(Tws.block<3,3>(0,0));

    //std::map<idpair, kfptr>::iterator mit;

    v1_x = Tws(0,3) - last_X;
    v1_y = Tws(1,3) - last_Y;
    v1_z = Tws(2,3) - last_Z;
    v2_x = X_diff;
    v2_y = Y_diff;
    v2_z = Z_diff;

    tracking_v1_x = (pKF->tracking_x) - tracking_last_X;
    tracking_v1_y = (pKF->tracking_y) - tracking_last_Y;
    tracking_v1_z = (pKF->tracking_z) - tracking_last_Z;
    tracking_v2_x = tracking_X_diff;
    tracking_v2_y = tracking_Y_diff;
    tracking_v2_z = tracking_Z_diff;

    if (KF_check_count > 10) {
        xy_radian = acos(((v1_x * v2_x) + (v1_y * v2_y)) / ((sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0)))*(sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0)))));
        yz_radian = acos(((v1_y * v2_y) + (v1_z * v2_z)) / ((sqrt(pow(v1_y, 2.0) + pow(v1_z, 2.0)))*(sqrt(pow(v2_y, 2.0) + pow(v2_z, 2.0)))));
        zx_radian = acos(((v1_z * v2_z) + (v1_x * v2_x)) / ((sqrt(pow(v1_z, 2.0) + pow(v1_x, 2.0)))*(sqrt(pow(v2_z, 2.0) + pow(v2_x, 2.0)))));
        xy_degree = xy_radian * (180/3.14);
        yz_degree = yz_radian * (180/3.14);
        zx_degree = zx_radian * (180/3.14);
        tracking_xy_radian = acos(((tracking_v1_x * tracking_v2_x) + (tracking_v1_y * tracking_v2_y)) / ((sqrt(pow(tracking_v1_x, 2.0) + pow(tracking_v1_y, 2.0)))*(sqrt(pow(tracking_v2_x, 2.0) + pow(tracking_v2_y, 2.0)))));
        tracking_yz_radian = acos(((tracking_v1_y * tracking_v2_y) + (tracking_v1_z * tracking_v2_z)) / ((sqrt(pow(tracking_v1_y, 2.0) + pow(tracking_v1_z, 2.0)))*(sqrt(pow(tracking_v2_y, 2.0) + pow(tracking_v2_z, 2.0)))));
        tracking_zx_radian = acos(((tracking_v1_z * tracking_v2_z) + (tracking_v1_x * tracking_v2_x)) / ((sqrt(pow(tracking_v1_z, 2.0) + pow(tracking_v1_x, 2.0)))*(sqrt(pow(tracking_v2_z, 2.0) + pow(tracking_v2_x, 2.0)))));
        tracking_xy_degree = tracking_xy_radian * (180/3.14);
        tracking_yz_degree = tracking_yz_radian * (180/3.14);
        tracking_zx_degree = tracking_zx_radian * (180/3.14);

        radian = acos(((v1_x * v2_x) + (v1_y * v2_y) + (v1_z * v2_z)) / ((sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0) + pow(v1_z, 2.0)))*(sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0) + pow(v2_z, 2.0)))));
        degree = radian * (180/3.14);
        tracking_radian = acos(((tracking_v1_x * tracking_v2_x) + (tracking_v1_y * tracking_v2_y) + (tracking_v1_z * tracking_v2_z)) / ((sqrt(pow(tracking_v1_x, 2.0) + pow(tracking_v1_y, 2.0) + pow(tracking_v1_z, 2.0)))*(sqrt(pow(tracking_v2_x, 2.0) + pow(tracking_v2_y, 2.0) + pow(tracking_v2_z, 2.0)))));
        tracking_degree = tracking_radian * (180/3.14);

        movement_ratio = sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0) + pow(v1_z, 2.0)) / sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0) + pow(v2_z, 2.0));
        tracking_movement_ratio = sqrt(pow(tracking_v1_x, 2.0) + pow(tracking_v1_y, 2.0) + pow(tracking_v1_z, 2.0)) / sqrt(pow(tracking_v2_x, 2.0) + pow(tracking_v2_y, 2.0) + pow(tracking_v2_z, 2.0));

        if (abs(degree - tracking_degree) > 40 * degree_margin) {
            violate_KF = (*itr);
            violate_KF->SetBadFlag();
            //pKF->SetPose(pKF->mTcwGBA,true);
            //unique_lock <mutex> lock(mMutexMap);
            //mit = mmpKeyFrames.find(pKF->mId);
            //if (mit != mmpKeyFrames.end()) mmpKeyFrames.erase(mit);
            violate_flag = 1;
            erase_counter_degree ++;
            update_flag = 0;
            degree_margin = degree_margin + 0.5;
            movement_margin = movement_margin + 0.3;
        }
        else if ((movement_ratio / tracking_movement_ratio) > (3.0 * movement_margin) || (tracking_movement_ratio / movement_ratio) > (3.0 * movement_margin)) {
            violate_KF = (*itr);
            violate_KF->SetBadFlag();
            //pKF->SetPose(pKF->mTcwGBA,true);
            //unique_lock <mutex> lock(mMutexMap);
            //mit = mmpKeyFrames.find(pKF->mId);
            //if (mit != mmpKeyFrames.end()) mmpKeyFrames.erase(mit);
            violate_flag = 1;
            erase_counter_movement ++;
            update_flag = 0;
            degree_margin = degree_margin + 0.5;
            movement_margin = movement_margin + 0.3;            
        }
        else if (abs(xy_degree - tracking_xy_degree) > 50 * degree_margin || abs(yz_degree - tracking_yz_degree) > 50 * degree_margin || abs(zx_degree - tracking_zx_degree) > 50 * degree_margin) {
            violate_KF = (*itr);
            violate_KF->SetBadFlag();
            //pKF->SetPose(pKF->mTcwGBA,true);
            //unique_lock <mutex> lock(mMutexMap);
            //mit = mmpKeyFrames.find(pKF->mId);
            //if (mit != mmpKeyFrames.end()) mmpKeyFrames.erase(mit);
            violate_flag = 1;
            erase_counter_plain_degree ++;
            update_flag = 0;
            degree_margin = degree_margin + 0.5;
            movement_margin = movement_margin + 0.3;
        }
        else {
            update_flag = 1;
            degree_margin = 1;
            movement_margin = 1;
        }
        
    }

    if (update_flag == 1) {
        X_diff = Tws(0,3) - last_X;
        Y_diff = Tws(1,3) - last_Y;
        Z_diff = Tws(2,3) - last_Z;
        last_X = Tws(0,3);
        last_Y = Tws(1,3);
        last_Z = Tws(2,3);

        tracking_X_diff = (pKF->tracking_x) - tracking_last_X;
        tracking_Y_diff = (pKF->tracking_y) - tracking_last_Y;
        tracking_Z_diff = (pKF->tracking_z) - tracking_last_Z;
        tracking_last_X = pKF->tracking_x;
        tracking_last_Y = pKF->tracking_y;
        tracking_last_Z = pKF->tracking_z; 
    }


    if (degree_margin > 6.0) {
        degree_margin = 6.0;
    }
    if (movement_margin > 3.0) {
        movement_margin = 3.0;
    }

    KF_check_count ++;
  }

  // for debug
  cout << "number of trimmed KFs (by degree) : " << erase_counter_degree << endl;
  cout << "number of trimmed KFs (by xy,yz,zx degree) : " << erase_counter_plain_degree << endl;
  cout << "number of trimmed KFs (by movement) : " << erase_counter_movement << endl;
  
  //initialize for next call
  erase_counter_degree = 0;
  erase_counter_plain_degree = 0;
  erase_counter_movement = 0;
  KF_check_count = 0;
  degree_margin = 1;
  movement_margin = 1;

}
////////////////////////////////////////////////////////////////////////////////////////////////////////

// add this code (2023/11/20) //////////////////////////////////////////////////////////////////////////
void Map::CheckMotionViolateKF_after(const size_t clientId) {
  std::vector<kfptr> foundKFs;
  foundKFs.reserve(mmpKeyFrames.size());
  // Get all frames from the required client
  for (std::map<idpair,kfptr>::const_iterator itr = mmpKeyFrames.begin();
       itr != mmpKeyFrames.end(); ++itr) {
    kfptr pKF = itr->second;
    idpair currID = itr->first;
    if (currID.second == clientId) {
      foundKFs.push_back(pKF);
    }
  }
  if(foundKFs.empty()) //would overwrite files from other maps with empty files
    return;

  // Sort the keyframes by timestamp
  std::sort(foundKFs.begin(), foundKFs.end(), KeyFrame::compKFstamp);
  
  // kfptr for motion check (add this code (2023/12/8))
  kfptr pKF_last;
  kfptr pKF_last_2;

  // Check if KF violate motion
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
    kfptr pKF = (*itr);
    const cv::Mat T_wc = pKF->GetPoseInverse();
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
    const Eigen::Quaterniond q(Tws.block<3,3>(0,0));

    float GBA_scale;
    double movement_GBABef;
    double movement_GBAAft;

    v1_x = Tws(0,3) - last_X;
    v1_y = Tws(1,3) - last_Y;
    v1_z = Tws(2,3) - last_Z;
    v2_x = X_diff;
    v2_y = Y_diff;
    v2_z = Z_diff;

    tracking_v1_x = (pKF->tracking_x) - tracking_last_X;
    tracking_v1_y = (pKF->tracking_y) - tracking_last_Y;
    tracking_v1_z = (pKF->tracking_z) - tracking_last_Z;
    tracking_v2_x = tracking_X_diff;
    tracking_v2_y = tracking_Y_diff;
    tracking_v2_z = tracking_Z_diff;

    if (KF_check_count > 10) {
        xy_radian = acos(((v1_x * v2_x) + (v1_y * v2_y)) / ((sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0)))*(sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0)))));
        yz_radian = acos(((v1_y * v2_y) + (v1_z * v2_z)) / ((sqrt(pow(v1_y, 2.0) + pow(v1_z, 2.0)))*(sqrt(pow(v2_y, 2.0) + pow(v2_z, 2.0)))));
        zx_radian = acos(((v1_z * v2_z) + (v1_x * v2_x)) / ((sqrt(pow(v1_z, 2.0) + pow(v1_x, 2.0)))*(sqrt(pow(v2_z, 2.0) + pow(v2_x, 2.0)))));
        xy_degree = xy_radian * (180/3.14);
        yz_degree = yz_radian * (180/3.14);
        zx_degree = zx_radian * (180/3.14);
        tracking_xy_radian = acos(((tracking_v1_x * tracking_v2_x) + (tracking_v1_y * tracking_v2_y)) / ((sqrt(pow(tracking_v1_x, 2.0) + pow(tracking_v1_y, 2.0)))*(sqrt(pow(tracking_v2_x, 2.0) + pow(tracking_v2_y, 2.0)))));
        tracking_yz_radian = acos(((tracking_v1_y * tracking_v2_y) + (tracking_v1_z * tracking_v2_z)) / ((sqrt(pow(tracking_v1_y, 2.0) + pow(tracking_v1_z, 2.0)))*(sqrt(pow(tracking_v2_y, 2.0) + pow(tracking_v2_z, 2.0)))));
        tracking_zx_radian = acos(((tracking_v1_z * tracking_v2_z) + (tracking_v1_x * tracking_v2_x)) / ((sqrt(pow(tracking_v1_z, 2.0) + pow(tracking_v1_x, 2.0)))*(sqrt(pow(tracking_v2_z, 2.0) + pow(tracking_v2_x, 2.0)))));
        tracking_xy_degree = tracking_xy_radian * (180/3.14);
        tracking_yz_degree = tracking_yz_radian * (180/3.14);
        tracking_zx_degree = tracking_zx_radian * (180/3.14);

        radian = acos(((v1_x * v2_x) + (v1_y * v2_y) + (v1_z * v2_z)) / ((sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0) + pow(v1_z, 2.0)))*(sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0) + pow(v2_z, 2.0)))));
        degree = radian * (180/3.14);
        tracking_radian = acos(((tracking_v1_x * tracking_v2_x) + (tracking_v1_y * tracking_v2_y) + (tracking_v1_z * tracking_v2_z)) / ((sqrt(pow(tracking_v1_x, 2.0) + pow(tracking_v1_y, 2.0) + pow(tracking_v1_z, 2.0)))*(sqrt(pow(tracking_v2_x, 2.0) + pow(tracking_v2_y, 2.0) + pow(tracking_v2_z, 2.0)))));
        tracking_degree = tracking_radian * (180/3.14);

        movement_ratio = sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0) + pow(v1_z, 2.0)) / sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0) + pow(v2_z, 2.0));
        tracking_movement_ratio = sqrt(pow(tracking_v1_x, 2.0) + pow(tracking_v1_y, 2.0) + pow(tracking_v1_z, 2.0)) / sqrt(pow(tracking_v2_x, 2.0) + pow(tracking_v2_y, 2.0) + pow(tracking_v2_z, 2.0));

        if (abs(degree - tracking_degree) > 30 * degree_margin) {
            //violate_KF = (*itr);
            //violate_KF->SetBadFlag();
            //pKF->SetPose(pKF->mTcwGBA,true);
            //pKF->SetPose(pKF->mTcwBefGBA,true);

            //cout << "DEBUG : degree_violation" << endl;

            //cout << "DEBUG : mTcwBefGBA_last" << endl;
            //cout << "DEBUG : pKF = " << pKF << endl;
            //cout << "DEBUG : pKF_last = " << pKF_last << endl;
            //cout << "DEBUG : pKF_last_2 = " << pKF_last_2 << endl;
            //cout << pKF_last->mTcwBefGBA << endl;
            
            // Twc (last KF, before GBA)
            mTwcBef_last = cv::Mat::eye(4,4,pKF_last->mTcwBefGBA.type());
            RcwBef_last = (pKF_last->mTcwBefGBA).rowRange(0,3).colRange(0,3);
            tcwBef_last = (pKF_last->mTcwBefGBA).rowRange(0,3).col(3);
            RwcBef_last = RcwBef_last.t();
            OwBef_last = -RwcBef_last*tcwBef_last;
            RwcBef_last.copyTo(mTwcBef_last.rowRange(0,3).colRange(0,3));
            OwBef_last.copyTo(mTwcBef_last.rowRange(0,3).col(3));
            //cout << "DEBUG : mTwcBefGBA_last is calculated." << endl;
            
            
            // Twc (last KF, after GBA)
            mTwcAft_last = cv::Mat::eye(4,4,pKF_last->mTcwGBA.type());
            RcwAft_last = (pKF_last->mTcwGBA).rowRange(0,3).colRange(0,3);
            tcwAft_last = (pKF_last->mTcwGBA).rowRange(0,3).col(3);
            RwcAft_last = RcwAft_last.t();
            OwAft_last = -RwcAft_last*tcwAft_last;
            RwcAft_last.copyTo(mTwcAft_last.rowRange(0,3).colRange(0,3));
            OwAft_last.copyTo(mTwcAft_last.rowRange(0,3).col(3));
            //cout << "DEBUG : mTwcAftGBA_last is calculated." << endl;

            // Twc (last_2 KF, before GBA)
            mTwcBef_last_2 = cv::Mat::eye(4,4,pKF_last_2->mTcwBefGBA.type());
            RcwBef_last_2 = (pKF_last_2->mTcwBefGBA).rowRange(0,3).colRange(0,3);
            tcwBef_last_2 = (pKF_last_2->mTcwBefGBA).rowRange(0,3).col(3);
            RwcBef_last_2 = RcwBef_last_2.t();
            OwBef_last_2 = -RwcBef_last_2*tcwBef_last_2;
            RwcBef_last_2.copyTo(mTwcBef_last_2.rowRange(0,3).colRange(0,3));
            OwBef_last_2.copyTo(mTwcBef_last_2.rowRange(0,3).col(3));
            //cout << "DEBUG : mTwcBefGBA_last_2 is calculated." << endl;

            // Twc (last_2 KF, after GBA)
            mTwcAft_last_2 = cv::Mat::eye(4,4,pKF_last_2->mTcwGBA.type());
            RcwAft_last_2 = (pKF_last_2->mTcwGBA).rowRange(0,3).colRange(0,3);
            tcwAft_last_2 = (pKF_last_2->mTcwGBA).rowRange(0,3).col(3);
            RwcAft_last_2 = RcwAft_last_2.t();
            OwAft_last_2 = -RwcAft_last_2*tcwAft_last_2;
            RwcAft_last_2.copyTo(mTwcAft_last_2.rowRange(0,3).colRange(0,3));
            OwAft_last_2.copyTo(mTwcAft_last_2.rowRange(0,3).col(3));
            //cout << "DEBUG : mTwcAftGBA_last_2 is calculated." << endl;
            
            
            mVelocity_last_Bef = (pKF_last->mTcwBefGBA) * mTwcBef_last_2;
            //cout << "DEBUG : mVelocity_last_BefGBA is calculated." << endl;
            mVelocity_last_Aft = (pKF_last->mTcwGBA) * mTwcAft_last_2;
            //cout << "DEBUG : mVelocity_last_AftGBA is calculated." << endl;


            cv::Mat mVelocity_last_Bef_Inverse = cv::Mat::eye(4,4,mVelocity_last_Bef.type());
            //cout << "DEBUG : point A" << endl;
            cv::Mat Rcw_mVelocity_last_Bef = mVelocity_last_Bef.rowRange(0,3).colRange(0,3);
            //cout << "DEBUG : point B" << endl;
            cv::Mat tcw_mVelocity_last_Bef = mVelocity_last_Bef.rowRange(0,3).col(3);
            //cout << "DEBUG : point C" << endl;
            cv::Mat Rwc_mVelocity_last_Bef = Rcw_mVelocity_last_Bef.t();
            //cout << "DEBUG : point D" << endl;
            cv::Mat Ow_mVelocity_last_Bef = -Rwc_mVelocity_last_Bef*tcw_mVelocity_last_Bef;
            //cout << "DEBUG : point E" << endl;
            Rwc_mVelocity_last_Bef.copyTo(mVelocity_last_Bef_Inverse.rowRange(0,3).colRange(0,3));
            //cout << "DEBUG : point F" << endl;
            Ow_mVelocity_last_Bef.copyTo(mVelocity_last_Bef_Inverse.rowRange(0,3).col(3));
            //cout << "DEBUG : mVelocity_last_BefGBA_Inverse is calculated." << endl;

            correct_effect = mVelocity_last_Aft * mVelocity_last_Bef_Inverse;
            //cout << "DEBUG : GBA_effect is calculated." << endl;

            Bef_motion = (pKF->mTcwBefGBA)*(mTwcBef_last);
            //cout << "DEBUG : GBABef_motion" << endl;
            //cout << GBABef_motion << endl;
            //translation_GBABef_last = (pKF_last->mTcwBefGBA.rowRange(0,3).col(3));
            //cout << "DEBUG : translation_GBABef_last" << endl;
            //cout << translation_GBABef_last << endl;
            //translation_GBAAft_last = (pKF_last->mTcwGBA.rowRange(0,3).col(3));
            //cout << "DEBUG : translation_GBAAft_last" << endl;
            //cout << translation_GBAAft_last << endl;            
            //movement_GBABef = sqrt(pow(translation_GBABef_last.at<double>(0,0), 2.0) + pow(translation_GBABef_last.at<double>(0,1), 2.0) + pow(translation_GBABef_last.at<double>(0,2), 2.0));
            //cout << "DEBUG : movement_GBABef" << endl;
            //cout << movement_GBABef << endl; 
            //movement_GBAAft = sqrt(pow(translation_GBAAft_last.at<double>(0,0), 2.0) + pow(translation_GBAAft_last.at<double>(0,1), 2.0) + pow(translation_GBAAft_last.at<double>(0,2), 2.0));
            //cout << "DEBUG : movement_GBAAft" << endl;
            //cout << movement_GBAAft << endl; 
            //GBA_scale = movement_GBAAft / movement_GBABef;
            //cout << "DEBUG :GBA_scale" << endl;
            //cout << GBA_scale << endl;
            GBA_scale = 1;

            //pKF->SetPose((GBABef_motion * GBA_effect) * (pKF_last->mTcwGBA), true);
            //cout << "DEBUG : (GBABef_motion * GBA_effect) * (pKF_last->mTcwGBA)" << endl;
            //cout << (GBABef_motion * GBA_effect) * (pKF_last->mTcwGBA) << endl;

            pKF->SetPose(Bef_motion * (pKF_last->mTcwGBA) * GBA_scale, true);
            //cout << "DEBUG : GBABef_motion * (pKF_last->mTcwGBA) * GBA_scale" << endl;
            //cout << GBABef_motion * (pKF_last->mTcwGBA) * GBA_scale << endl;
            //pKF->mbLoopCorrected = false;

            pKF->mTcwGBA = Bef_motion * (pKF_last->mTcwGBA) * GBA_scale;

            //pKF->mTcwGBA = (Bef_motion * correctt_effect) * (pKF_last->mTcwGBA);

            violate_flag = 1;
            erase_counter_degree ++;
            update_flag = 0;
            degree_margin = degree_margin + 0.2;
            movement_margin = movement_margin + 0.3;
        }
        else if ((movement_ratio / tracking_movement_ratio) > (3.0 * movement_margin) || (tracking_movement_ratio / movement_ratio) > (3.0 * movement_margin)) {
            //violate_KF = (*itr);
            //violate_KF->SetBadFlag();
            //pKF->SetPose(pKF->mTcwGBA,true);
            /*
            pKF->SetPose(pKF->mTcwBefGBA,true);
            pKF->mbLoopCorrected = false;
            pKF->mTcwGBA = pKF->mTcwBefGBA;
            violate_flag = 1;
            erase_counter_movement ++;
            update_flag = 0;
            degree_margin = degree_margin + 0.2;
            movement_margin = movement_margin + 0.3;     
            */
            //cout << "DEBUG : mTcwBefGBA_last" << endl;
            //cout << "DEBUG : pKF = " << pKF << endl;
            //cout << "DEBUG : pKF_last = " << pKF_last << endl;
            //cout << "DEBUG : pKF_last_2 = " << pKF_last_2 << endl;
            //cout << pKF_last->mTcwBefGBA << endl;
            
            // Twc (last KF, before GBA)
            mTwcBef_last = cv::Mat::eye(4,4,pKF_last->mTcwBefGBA.type());
            RcwBef_last = (pKF_last->mTcwBefGBA).rowRange(0,3).colRange(0,3);
            tcwBef_last = (pKF_last->mTcwBefGBA).rowRange(0,3).col(3);
            RwcBef_last = RcwBef_last.t();
            OwBef_last = -RwcBef_last*tcwBef_last;
            RwcBef_last.copyTo(mTwcBef_last.rowRange(0,3).colRange(0,3));
            OwBef_last.copyTo(mTwcBef_last.rowRange(0,3).col(3));
            //cout << "DEBUG : mTwcBefGBA_last is calculated." << endl;
            
            
            // Twc (last KF, after GBA)
            mTwcAft_last = cv::Mat::eye(4,4,pKF_last->mTcwGBA.type());
            RcwAft_last = (pKF_last->mTcwGBA).rowRange(0,3).colRange(0,3);
            tcwAft_last = (pKF_last->mTcwGBA).rowRange(0,3).col(3);
            RwcAft_last = RcwAft_last.t();
            OwAft_last = -RwcAft_last*tcwAft_last;
            RwcAft_last.copyTo(mTwcAft_last.rowRange(0,3).colRange(0,3));
            OwAft_last.copyTo(mTwcAft_last.rowRange(0,3).col(3));
            //cout << "DEBUG : mTwcAftGBA_last is calculated." << endl;

            // Twc (last_2 KF, before GBA)
            mTwcBef_last_2 = cv::Mat::eye(4,4,pKF_last_2->mTcwBefGBA.type());
            RcwBef_last_2 = (pKF_last_2->mTcwBefGBA).rowRange(0,3).colRange(0,3);
            tcwBef_last_2 = (pKF_last_2->mTcwBefGBA).rowRange(0,3).col(3);
            RwcBef_last_2 = RcwBef_last_2.t();
            OwBef_last_2 = -RwcBef_last_2*tcwBef_last_2;
            RwcBef_last_2.copyTo(mTwcBef_last_2.rowRange(0,3).colRange(0,3));
            OwBef_last_2.copyTo(mTwcBef_last_2.rowRange(0,3).col(3));
            //cout << "DEBUG : mTwcBefGBA_last_2 is calculated." << endl;

            // Twc (last_2 KF, after GBA)
            mTwcAft_last_2 = cv::Mat::eye(4,4,pKF_last_2->mTcwGBA.type());
            RcwAft_last_2 = (pKF_last_2->mTcwGBA).rowRange(0,3).colRange(0,3);
            tcwAft_last_2 = (pKF_last_2->mTcwGBA).rowRange(0,3).col(3);
            RwcAft_last_2 = RcwAft_last_2.t();
            OwAft_last_2 = -RwcAft_last_2*tcwAft_last_2;
            RwcAft_last_2.copyTo(mTwcAft_last_2.rowRange(0,3).colRange(0,3));
            OwAft_last_2.copyTo(mTwcAft_last_2.rowRange(0,3).col(3));
            //cout << "DEBUG : mTwcAftGBA_last_2 is calculated." << endl;
            
            
            mVelocity_last_Bef = (pKF_last->mTcwBefGBA) * mTwcBef_last_2;
            //cout << "DEBUG : mVelocity_last_BefGBA is calculated." << endl;
            mVelocity_last_Aft = (pKF_last->mTcwGBA) * mTwcAft_last_2;
            //cout << "DEBUG : mVelocity_last_AftGBA is calculated." << endl;


            cv::Mat mVelocity_last_Bef_Inverse = cv::Mat::eye(4,4,mVelocity_last_Bef.type());
            //cout << "DEBUG : point A" << endl;
            cv::Mat Rcw_mVelocity_last_Bef = mVelocity_last_Bef.rowRange(0,3).colRange(0,3);
            //cout << "DEBUG : point B" << endl;
            cv::Mat tcw_mVelocity_last_Bef = mVelocity_last_Bef.rowRange(0,3).col(3);
            //cout << "DEBUG : point C" << endl;
            cv::Mat Rwc_mVelocity_last_Bef = Rcw_mVelocity_last_Bef.t();
            //cout << "DEBUG : point D" << endl;
            cv::Mat Ow_mVelocity_last_Bef = -Rwc_mVelocity_last_Bef*tcw_mVelocity_last_Bef;
            //cout << "DEBUG : point E" << endl;
            Rwc_mVelocity_last_Bef.copyTo(mVelocity_last_Bef_Inverse.rowRange(0,3).colRange(0,3));
            //cout << "DEBUG : point F" << endl;
            Ow_mVelocity_last_Bef.copyTo(mVelocity_last_Bef_Inverse.rowRange(0,3).col(3));
            //cout << "DEBUG : mVelocity_last_BefGBA_Inverse is calculated." << endl;

            correct_effect = mVelocity_last_Aft * mVelocity_last_Bef_Inverse;
            //cout << "DEBUG : GBA_effect is calculated." << endl;

            Bef_motion = (pKF->mTcwBefGBA)*(mTwcBef_last);
            //cout << "DEBUG : GBABef_motion" << endl;
            //cout << GBABef_motion << endl;
            //translation_GBABef_last = (pKF_last->mTcwBefGBA.rowRange(0,3).col(3));
            //cout << "DEBUG : translation_GBABef_last" << endl;
            //cout << translation_GBABef_last << endl;
            //translation_GBAAft_last = (pKF_last->mTcwGBA.rowRange(0,3).col(3));
            //cout << "DEBUG : translation_GBAAft_last" << endl;
            //cout << translation_GBAAft_last << endl;            
            //movement_GBABef = sqrt(pow(translation_GBABef_last.at<double>(0,0), 2.0) + pow(translation_GBABef_last.at<double>(0,1), 2.0) + pow(translation_GBABef_last.at<double>(0,2), 2.0));
            //cout << "DEBUG : movement_GBABef" << endl;
            //cout << movement_GBABef << endl; 
            //movement_GBAAft = sqrt(pow(translation_GBAAft_last.at<double>(0,0), 2.0) + pow(translation_GBAAft_last.at<double>(0,1), 2.0) + pow(translation_GBAAft_last.at<double>(0,2), 2.0));
            //cout << "DEBUG : movement_GBAAft" << endl;
            //cout << movement_GBAAft << endl; 
            //GBA_scale = movement_GBAAft / movement_GBABef;
            //cout << "DEBUG :GBA_scale" << endl;
            //cout << GBA_scale << endl;
            GBA_scale = 1;

            //pKF->SetPose((GBABef_motion * GBA_effect) * (pKF_last->mTcwGBA), true);
            //cout << "DEBUG : (GBABef_motion * GBA_effect) * (pKF_last->mTcwGBA)" << endl;
            //cout << (GBABef_motion * GBA_effect) * (pKF_last->mTcwGBA) << endl;

            pKF->SetPose(Bef_motion * (pKF_last->mTcwGBA) * GBA_scale, true);
            //cout << "DEBUG : GBABef_motion * (pKF_last->mTcwGBA) * GBA_scale" << endl;
            //cout << GBABef_motion * (pKF_last->mTcwGBA) * GBA_scale << endl;
            //pKF->mbLoopCorrected = false;

            pKF->mTcwGBA = Bef_motion * (pKF_last->mTcwGBA) * GBA_scale;

            //pKF->mTcwGBA = (Bef_motion * correctt_effect) * (pKF_last->mTcwGBA);

            violate_flag = 1;
            erase_counter_degree ++;
            update_flag = 0;
            degree_margin = degree_margin + 0.2;
            movement_margin = movement_margin + 0.3;
        }
        else if (abs(xy_degree - tracking_xy_degree) > 50 * degree_margin || abs(yz_degree - tracking_yz_degree) > 50 * degree_margin || abs(zx_degree - tracking_zx_degree) > 50 * degree_margin) {
            //violate_KF = (*itr);
            //violate_KF->SetBadFlag();
            //pKF->SetPose(pKF->mTcwGBA,true);
            /*
            pKF->SetPose(pKF->mTcwBefGBA,true);
            pKF->mbLoopCorrected = false;
            pKF->mTcwGBA = pKF->mTcwBefGBA;
            violate_flag = 1;
            erase_counter_plain_degree ++;
            update_flag = 0;
            degree_margin = degree_margin + 0.2;
            movement_margin = movement_margin + 0.3;
            */
            //cout << "DEBUG : mTcwBefGBA_last" << endl;
            //cout << "DEBUG : pKF = " << pKF << endl;
            //cout << "DEBUG : pKF_last = " << pKF_last << endl;
            //cout << "DEBUG : pKF_last_2 = " << pKF_last_2 << endl;
            //cout << pKF_last->mTcwBefGBA << endl;
            
            // Twc (last KF, before GBA)
            mTwcBef_last = cv::Mat::eye(4,4,pKF_last->mTcwBefGBA.type());
            RcwBef_last = (pKF_last->mTcwBefGBA).rowRange(0,3).colRange(0,3);
            tcwBef_last = (pKF_last->mTcwBefGBA).rowRange(0,3).col(3);
            RwcBef_last = RcwBef_last.t();
            OwBef_last = -RwcBef_last*tcwBef_last;
            RwcBef_last.copyTo(mTwcBef_last.rowRange(0,3).colRange(0,3));
            OwBef_last.copyTo(mTwcBef_last.rowRange(0,3).col(3));
            //cout << "DEBUG : mTwcBefGBA_last is calculated." << endl;
            
            
            // Twc (last KF, after GBA)
            mTwcAft_last = cv::Mat::eye(4,4,pKF_last->mTcwGBA.type());
            RcwAft_last = (pKF_last->mTcwGBA).rowRange(0,3).colRange(0,3);
            tcwAft_last = (pKF_last->mTcwGBA).rowRange(0,3).col(3);
            RwcAft_last = RcwAft_last.t();
            OwAft_last = -RwcAft_last*tcwAft_last;
            RwcAft_last.copyTo(mTwcAft_last.rowRange(0,3).colRange(0,3));
            OwAft_last.copyTo(mTwcAft_last.rowRange(0,3).col(3));
            //cout << "DEBUG : mTwcAftGBA_last is calculated." << endl;

            // Twc (last_2 KF, before GBA)
            mTwcBef_last_2 = cv::Mat::eye(4,4,pKF_last_2->mTcwBefGBA.type());
            RcwBef_last_2 = (pKF_last_2->mTcwBefGBA).rowRange(0,3).colRange(0,3);
            tcwBef_last_2 = (pKF_last_2->mTcwBefGBA).rowRange(0,3).col(3);
            RwcBef_last_2 = RcwBef_last_2.t();
            OwBef_last_2 = -RwcBef_last_2*tcwBef_last_2;
            RwcBef_last_2.copyTo(mTwcBef_last_2.rowRange(0,3).colRange(0,3));
            OwBef_last_2.copyTo(mTwcBef_last_2.rowRange(0,3).col(3));
            //cout << "DEBUG : mTwcBefGBA_last_2 is calculated." << endl;

            // Twc (last_2 KF, after GBA)
            mTwcAft_last_2 = cv::Mat::eye(4,4,pKF_last_2->mTcwGBA.type());
            RcwAft_last_2 = (pKF_last_2->mTcwGBA).rowRange(0,3).colRange(0,3);
            tcwAft_last_2 = (pKF_last_2->mTcwGBA).rowRange(0,3).col(3);
            RwcAft_last_2 = RcwAft_last_2.t();
            OwAft_last_2 = -RwcAft_last_2*tcwAft_last_2;
            RwcAft_last_2.copyTo(mTwcAft_last_2.rowRange(0,3).colRange(0,3));
            OwAft_last_2.copyTo(mTwcAft_last_2.rowRange(0,3).col(3));
            //cout << "DEBUG : mTwcAftGBA_last_2 is calculated." << endl;
            
            
            mVelocity_last_Bef = (pKF_last->mTcwBefGBA) * mTwcBef_last_2;
            //cout << "DEBUG : mVelocity_last_BefGBA is calculated." << endl;
            mVelocity_last_Aft = (pKF_last->mTcwGBA) * mTwcAft_last_2;
            //cout << "DEBUG : mVelocity_last_AftGBA is calculated." << endl;


            cv::Mat mVelocity_last_Bef_Inverse = cv::Mat::eye(4,4,mVelocity_last_Bef.type());
            //cout << "DEBUG : point A" << endl;
            cv::Mat Rcw_mVelocity_last_Bef = mVelocity_last_Bef.rowRange(0,3).colRange(0,3);
            //cout << "DEBUG : point B" << endl;
            cv::Mat tcw_mVelocity_last_Bef = mVelocity_last_Bef.rowRange(0,3).col(3);
            //cout << "DEBUG : point C" << endl;
            cv::Mat Rwc_mVelocity_last_Bef = Rcw_mVelocity_last_Bef.t();
            //cout << "DEBUG : point D" << endl;
            cv::Mat Ow_mVelocity_last_Bef = -Rwc_mVelocity_last_Bef*tcw_mVelocity_last_Bef;
            //cout << "DEBUG : point E" << endl;
            Rwc_mVelocity_last_Bef.copyTo(mVelocity_last_Bef_Inverse.rowRange(0,3).colRange(0,3));
            //cout << "DEBUG : point F" << endl;
            Ow_mVelocity_last_Bef.copyTo(mVelocity_last_Bef_Inverse.rowRange(0,3).col(3));
            //cout << "DEBUG : mVelocity_last_BefGBA_Inverse is calculated." << endl;

            correct_effect = mVelocity_last_Aft * mVelocity_last_Bef_Inverse;
            //cout << "DEBUG : GBA_effect is calculated." << endl;

            Bef_motion = (pKF->mTcwBefGBA)*(mTwcBef_last);
            //cout << "DEBUG : GBABef_motion" << endl;
            //cout << GBABef_motion << endl;
            //translation_GBABef_last = (pKF_last->mTcwBefGBA.rowRange(0,3).col(3));
            //cout << "DEBUG : translation_GBABef_last" << endl;
            //cout << translation_GBABef_last << endl;
            //translation_GBAAft_last = (pKF_last->mTcwGBA.rowRange(0,3).col(3));
            //cout << "DEBUG : translation_GBAAft_last" << endl;
            //cout << translation_GBAAft_last << endl;            
            //movement_GBABef = sqrt(pow(translation_GBABef_last.at<double>(0,0), 2.0) + pow(translation_GBABef_last.at<double>(0,1), 2.0) + pow(translation_GBABef_last.at<double>(0,2), 2.0));
            //cout << "DEBUG : movement_GBABef" << endl;
            //cout << movement_GBABef << endl; 
            //movement_GBAAft = sqrt(pow(translation_GBAAft_last.at<double>(0,0), 2.0) + pow(translation_GBAAft_last.at<double>(0,1), 2.0) + pow(translation_GBAAft_last.at<double>(0,2), 2.0));
            //cout << "DEBUG : movement_GBAAft" << endl;
            //cout << movement_GBAAft << endl; 
            //GBA_scale = movement_GBAAft / movement_GBABef;
            //cout << "DEBUG :GBA_scale" << endl;
            //cout << GBA_scale << endl;
            GBA_scale = 1;

            //pKF->SetPose((GBABef_motion * GBA_effect) * (pKF_last->mTcwGBA), true);
            //cout << "DEBUG : (GBABef_motion * GBA_effect) * (pKF_last->mTcwGBA)" << endl;
            //cout << (GBABef_motion * GBA_effect) * (pKF_last->mTcwGBA) << endl;

            pKF->SetPose(Bef_motion * (pKF_last->mTcwGBA) * GBA_scale, true);
            //cout << "DEBUG : GBABef_motion * (pKF_last->mTcwGBA) * GBA_scale" << endl;
            //cout << GBABef_motion * (pKF_last->mTcwGBA) * GBA_scale << endl;
            //pKF->mbLoopCorrected = false;

            pKF->mTcwGBA = Bef_motion * (pKF_last->mTcwGBA) * GBA_scale;

            //pKF->mTcwGBA = (Bef_motion * correctt_effect) * (pKF_last->mTcwGBA);

            violate_flag = 1;
            erase_counter_degree ++;
            update_flag = 0;
            degree_margin = degree_margin + 0.2;
            movement_margin = movement_margin + 0.3;
        }
        else {
            update_flag = 1;
            degree_margin = 1;
            movement_margin = 1;
        }
        
    }

    if (update_flag == 1) {
        X_diff = Tws(0,3) - last_X;
        Y_diff = Tws(1,3) - last_Y;
        Z_diff = Tws(2,3) - last_Z;
        last_X = Tws(0,3);
        last_Y = Tws(1,3);
        last_Z = Tws(2,3);

        tracking_X_diff = (pKF->tracking_x) - tracking_last_X;
        tracking_Y_diff = (pKF->tracking_y) - tracking_last_Y;
        tracking_Z_diff = (pKF->tracking_z) - tracking_last_Z;
        tracking_last_X = pKF->tracking_x;
        tracking_last_Y = pKF->tracking_y;
        tracking_last_Z = pKF->tracking_z;

        pKF_last_2 = pKF_last;
        pKF_last = pKF;
    }

    if (degree_margin > 3.0) {
        degree_margin = 3.0;
    }
    if (movement_margin > 3.0) {
        movement_margin = 3.0;
    }

    KF_check_count ++;

    // update tracking_x,y,z
    cv::Mat T_wc_current = cv::Mat::eye(4,4, pKF -> mTcwGBA.type());
    cv::Mat mRwc = (pKF->mTcwGBA.rowRange(0,3).colRange(0,3).t());
    mRwc.copyTo(T_wc_current.rowRange(0,3).colRange(0,3));
    (pKF -> GetCameraCenter()).copyTo(T_wc_current.rowRange(0,3).col(3)); 

    const Eigen::Matrix4d eT_wc_current = Converter::toMatrix4d(T_wc_current);

    Eigen::Matrix4d T_SC_current;
    T_SC_current = pKF -> mT_SC;

    const Eigen::Matrix4d Tws_current = eT_wc_current * T_SC_current.inverse();

    pKF -> tracking_x = Tws_current(0,3);
    pKF -> tracking_y = Tws_current(1,3);
    pKF -> tracking_z = Tws_current(2,3);

    // for debug
    //cout << "DEBUG : CheckMotionViolateKF_after is runnning." << endl;
  }

  // for debug
  cout << "number of changed KFs (by degree) : " << erase_counter_degree << endl;
  cout << "number of changed KFs (by xy,yz,zx degree) : " << erase_counter_plain_degree << endl;
  cout << "number of changed KFs (by movement) : " << erase_counter_movement << endl;
  
  //initialize for next call
  erase_counter_degree = 0;
  erase_counter_plain_degree = 0;
  erase_counter_movement = 0;
  KF_check_count = 0;
  degree_margin = 1;
  movement_margin = 1;
  update_flag = 1;
  violate_flag = 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////

// add this code (2023/12/11) //////////////////////////////////////////////////////////////////////////
bool Map::MotionCheck(const size_t clientId, kfptr pKF, Eigen::Matrix4d Tws, kfptr pKF_last, kfptr pKF_last_2, float last_X, float last_Y, float last_Z, float tracking_last_X, float tracking_last_Y, float tracking_last_Z, float X_diff, float Y_diff, float Z_diff, float tracking_X_diff, float tracking_Y_diff, float tracking_Z_diff, float degree_margin, float movement_margin) {

  // Check if KF violate motion

    const Eigen::Quaterniond q(Tws.block<3,3>(0,0));

    v1_x = Tws(0,3) - last_X;
    v1_y = Tws(1,3) - last_Y;
    v1_z = Tws(2,3) - last_Z;
    v2_x = X_diff;
    v2_y = Y_diff;
    v2_z = Z_diff;

    tracking_v1_x = (pKF->tracking_x) - tracking_last_X;
    tracking_v1_y = (pKF->tracking_y) - tracking_last_Y;
    tracking_v1_z = (pKF->tracking_z) - tracking_last_Z;
    tracking_v2_x = tracking_X_diff;
    tracking_v2_y = tracking_Y_diff;
    tracking_v2_z = tracking_Z_diff;

    xy_radian = acos(((v1_x * v2_x) + (v1_y * v2_y)) / ((sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0)))*(sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0)))));
    yz_radian = acos(((v1_y * v2_y) + (v1_z * v2_z)) / ((sqrt(pow(v1_y, 2.0) + pow(v1_z, 2.0)))*(sqrt(pow(v2_y, 2.0) + pow(v2_z, 2.0)))));
    zx_radian = acos(((v1_z * v2_z) + (v1_x * v2_x)) / ((sqrt(pow(v1_z, 2.0) + pow(v1_x, 2.0)))*(sqrt(pow(v2_z, 2.0) + pow(v2_x, 2.0)))));
    xy_degree = xy_radian * (180/3.14);
    yz_degree = yz_radian * (180/3.14);
    zx_degree = zx_radian * (180/3.14);
    tracking_xy_radian = acos(((tracking_v1_x * tracking_v2_x) + (tracking_v1_y * tracking_v2_y)) / ((sqrt(pow(tracking_v1_x, 2.0) + pow(tracking_v1_y, 2.0)))*(sqrt(pow(tracking_v2_x, 2.0) + pow(tracking_v2_y, 2.0)))));
    tracking_yz_radian = acos(((tracking_v1_y * tracking_v2_y) + (tracking_v1_z * tracking_v2_z)) / ((sqrt(pow(tracking_v1_y, 2.0) + pow(tracking_v1_z, 2.0)))*(sqrt(pow(tracking_v2_y, 2.0) + pow(tracking_v2_z, 2.0)))));
    tracking_zx_radian = acos(((tracking_v1_z * tracking_v2_z) + (tracking_v1_x * tracking_v2_x)) / ((sqrt(pow(tracking_v1_z, 2.0) + pow(tracking_v1_x, 2.0)))*(sqrt(pow(tracking_v2_z, 2.0) + pow(tracking_v2_x, 2.0)))));
    tracking_xy_degree = tracking_xy_radian * (180/3.14);
    tracking_yz_degree = tracking_yz_radian * (180/3.14);
    tracking_zx_degree = tracking_zx_radian * (180/3.14);

    radian = acos(((v1_x * v2_x) + (v1_y * v2_y) + (v1_z * v2_z)) / ((sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0) + pow(v1_z, 2.0)))*(sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0) + pow(v2_z, 2.0)))));
    degree = radian * (180/3.14);
    tracking_radian = acos(((tracking_v1_x * tracking_v2_x) + (tracking_v1_y * tracking_v2_y) + (tracking_v1_z * tracking_v2_z)) / ((sqrt(pow(tracking_v1_x, 2.0) + pow(tracking_v1_y, 2.0) + pow(tracking_v1_z, 2.0)))*(sqrt(pow(tracking_v2_x, 2.0) + pow(tracking_v2_y, 2.0) + pow(tracking_v2_z, 2.0)))));
    tracking_degree = tracking_radian * (180/3.14);

    movement_ratio = sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0) + pow(v1_z, 2.0)) / sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0) + pow(v2_z, 2.0));
    tracking_movement_ratio = sqrt(pow(tracking_v1_x, 2.0) + pow(tracking_v1_y, 2.0) + pow(tracking_v1_z, 2.0)) / sqrt(pow(tracking_v2_x, 2.0) + pow(tracking_v2_y, 2.0) + pow(tracking_v2_z, 2.0));

    if (abs(degree - tracking_degree) >  min(tracking_degree * 2, static_cast<float>(30)) * degree_margin) {
        //cout << "Motion Violate : DEGREE" << endl;
        return false;
    }
    else if ((movement_ratio / tracking_movement_ratio) > (2.0 * movement_margin) || (tracking_movement_ratio / movement_ratio) > (2.0 * movement_margin)) {
        //cout << "Motion Violate : MOVEMENT" << endl;
        return false;
    }
    else if (abs(xy_degree - tracking_xy_degree) > min(tracking_xy_degree * 5,  static_cast<float>(40)) * degree_margin || abs(yz_degree - tracking_yz_degree) > min(tracking_xy_degree * 5, static_cast<float>(40)) * degree_margin || abs(zx_degree - tracking_zx_degree) > min(tracking_xy_degree * 5, static_cast<float>(40)) * degree_margin) {
        //cout << "Motion Violate : xy, yz, zx DEGREE" << endl;
        return false;
    }
    else {
        return true;
    }
  }


void Map::CorrectKF(const size_t clientId, bool isGBA) {
  std::vector<kfptr> foundKFs;
  foundKFs.reserve(mmpKeyFrames.size());
  // Get all frames from the required client
  for (std::map<idpair,kfptr>::const_iterator itr = mmpKeyFrames.begin();
       itr != mmpKeyFrames.end(); ++itr) {
    kfptr pKF = itr->second;
    idpair currID = itr->first;
    if (currID.second == clientId) {
      foundKFs.push_back(pKF);
    }
  }
  if(foundKFs.empty()) //would overwrite files from other maps with empty files
    return;

  // Sort the keyframes by timestamp
  std::sort(foundKFs.begin(), foundKFs.end(), KeyFrame::compKFstamp);
  
  // kfptr for motion check (add this code (2023/12/8))
  kfptr pKF_last;
  kfptr pKF_last_2;

  // Check if KF violate motion
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
    kfptr pKF = (*itr);

    const cv::Mat T_wc = pKF->GetPoseInverse();
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();

    //for debug
    //cout << pKF->Tcw_current << endl;
    
    if (KF_check_count > 10) {
        bool motion_check = this->MotionCheck(clientId, pKF, Tws, pKF_last, pKF_last_2, last_X, last_Y, last_Z, tracking_last_X, tracking_last_Y, tracking_last_Z, X_diff, Y_diff, Z_diff, tracking_X_diff, tracking_Y_diff, tracking_Z_diff, degree_margin, movement_margin);

        // for debug
        /*
        cout << "mTcwBefGBA : " << endl;
        cout << pKF->mTcwBefGBA << endl;
        cout << "mTcwGBA : " << endl;
        cout << pKF->mTcwGBA << endl;
        cout << "Tcw (GetPose()) : " << endl;
        cout << pKF->GetPose() << endl;
        cout << "Tcw_current : " << endl;
        cout << pKF->Tcw_current << endl;
        cout << endl;
        */

        //cout << "motion check is completed." << endl;

        if (motion_check == false) {
            /*
            if (isGBA == true) {
                // Twc (last KF, before GBA)
                mTwcBef_last = cv::Mat::eye(4,4,pKF_last->mTcwBefGBA.type());
                RcwBef_last = (pKF_last->mTcwBefGBA).rowRange(0,3).colRange(0,3);
                tcwBef_last = (pKF_last->mTcwBefGBA).rowRange(0,3).col(3);
                RwcBef_last = RcwBef_last.t();
                OwBef_last = -RwcBef_last*tcwBef_last;
                RwcBef_last.copyTo(mTwcBef_last.rowRange(0,3).colRange(0,3));
                OwBef_last.copyTo(mTwcBef_last.rowRange(0,3).col(3));
                
                // Twc (last KF, after GBA)
                mTwcAft_last = cv::Mat::eye(4,4,pKF_last->mTcwGBA.type());
                RcwAft_last = (pKF_last->mTcwGBA).rowRange(0,3).colRange(0,3);
                tcwAft_last = (pKF_last->mTcwGBA).rowRange(0,3).col(3);
                RwcAft_last = RcwAft_last.t();
                OwAft_last = -RwcAft_last*tcwAft_last;
                RwcAft_last.copyTo(mTwcAft_last.rowRange(0,3).colRange(0,3));
                OwAft_last.copyTo(mTwcAft_last.rowRange(0,3).col(3));

                // Twc (last_2 KF, before GBA)
                mTwcBef_last_2 = cv::Mat::eye(4,4,pKF_last_2->mTcwBefGBA.type());
                RcwBef_last_2 = (pKF_last_2->mTcwBefGBA).rowRange(0,3).colRange(0,3);
                tcwBef_last_2 = (pKF_last_2->mTcwBefGBA).rowRange(0,3).col(3);
                RwcBef_last_2 = RcwBef_last_2.t();
                OwBef_last_2 = -RwcBef_last_2*tcwBef_last_2;
                RwcBef_last_2.copyTo(mTwcBef_last_2.rowRange(0,3).colRange(0,3));
                OwBef_last_2.copyTo(mTwcBef_last_2.rowRange(0,3).col(3));

                // Twc (last_2 KF, after GBA)
                mTwcAft_last_2 = cv::Mat::eye(4,4,pKF_last_2->mTcwGBA.type());
                RcwAft_last_2 = (pKF_last_2->mTcwGBA).rowRange(0,3).colRange(0,3);
                tcwAft_last_2 = (pKF_last_2->mTcwGBA).rowRange(0,3).col(3);
                RwcAft_last_2 = RcwAft_last_2.t();
                OwAft_last_2 = -RwcAft_last_2*tcwAft_last_2;
                RwcAft_last_2.copyTo(mTwcAft_last_2.rowRange(0,3).colRange(0,3));
                OwAft_last_2.copyTo(mTwcAft_last_2.rowRange(0,3).col(3));
                
                mVelocity_last_Bef = (pKF_last->mTcwBefGBA) * mTwcBef_last_2;
                mVelocity_last_Aft = (pKF_last->mTcwGBA) * mTwcAft_last_2;

                cv::Mat mVelocity_last_Bef_Inverse = cv::Mat::eye(4,4,mVelocity_last_Bef.type());
                cv::Mat Rcw_mVelocity_last_Bef = mVelocity_last_Bef.rowRange(0,3).colRange(0,3);
                cv::Mat tcw_mVelocity_last_Bef = mVelocity_last_Bef.rowRange(0,3).col(3);
                cv::Mat Rwc_mVelocity_last_Bef = Rcw_mVelocity_last_Bef.t();
                cv::Mat Ow_mVelocity_last_Bef = -Rwc_mVelocity_last_Bef*tcw_mVelocity_last_Bef;
                Rwc_mVelocity_last_Bef.copyTo(mVelocity_last_Bef_Inverse.rowRange(0,3).colRange(0,3));
                Ow_mVelocity_last_Bef.copyTo(mVelocity_last_Bef_Inverse.rowRange(0,3).col(3));

                correct_effect = mVelocity_last_Aft * mVelocity_last_Bef_Inverse;

                Bef_motion = (pKF->mTcwBefGBA)*(mTwcBef_last);

                pKF->SetPose((Bef_motion * correct_effect) * (pKF_last->mTcwGBA), true);
            }
            
            else {
            */
            //cout << "motion correction (Loop Closing)" << endl;

            // Twc (last KF, before Loop Closure)
            mTwcBef_last = cv::Mat::eye(4,4,pKF_last->Tcw_current.type());
            RcwBef_last = (pKF_last->Tcw_current).rowRange(0,3).colRange(0,3);
            tcwBef_last = (pKF_last->Tcw_current).rowRange(0,3).col(3);
            RwcBef_last = RcwBef_last.t();
            OwBef_last = -RwcBef_last*tcwBef_last;
            RwcBef_last.copyTo(mTwcBef_last.rowRange(0,3).colRange(0,3));
            OwBef_last.copyTo(mTwcBef_last.rowRange(0,3).col(3));

            //cout << "mTwcBef_last is calculated." << endl;
            
            // Twc (last KF, after Loop Closure)
            mTwcAft_last = cv::Mat::eye(4,4,pKF_last->Tcw_new.type());
            RcwAft_last = (pKF_last->Tcw_new).rowRange(0,3).colRange(0,3);
            tcwAft_last = (pKF_last->Tcw_new).rowRange(0,3).col(3);
            RwcAft_last = RcwAft_last.t();
            OwAft_last = -RwcAft_last*tcwAft_last;
            RwcAft_last.copyTo(mTwcAft_last.rowRange(0,3).colRange(0,3));
            OwAft_last.copyTo(mTwcAft_last.rowRange(0,3).col(3));

            //cout << "mTwcAft_last is calculated." << endl;

            // Twc (last_2 KF, before Loop Closure)
            mTwcBef_last_2 = cv::Mat::eye(4,4,pKF_last_2->Tcw_current.type());
            RcwBef_last_2 = (pKF_last_2->Tcw_current).rowRange(0,3).colRange(0,3);
            tcwBef_last_2 = (pKF_last_2->Tcw_current).rowRange(0,3).col(3);
            RwcBef_last_2 = RcwBef_last_2.t();
            OwBef_last_2 = -RwcBef_last_2*tcwBef_last_2;
            RwcBef_last_2.copyTo(mTwcBef_last_2.rowRange(0,3).colRange(0,3));
            OwBef_last_2.copyTo(mTwcBef_last_2.rowRange(0,3).col(3));

            //cout << "mTwcBef_last_2 is calculated." << endl;

            // Twc (last_2 KF, after Loop Closure)
            mTwcAft_last_2 = cv::Mat::eye(4,4,pKF_last_2->Tcw_new.type());
            RcwAft_last_2 = (pKF_last_2->Tcw_new).rowRange(0,3).colRange(0,3);
            tcwAft_last_2 = (pKF_last_2->Tcw_new).rowRange(0,3).col(3);
            RwcAft_last_2 = RcwAft_last_2.t();
            OwAft_last_2 = -RwcAft_last_2*tcwAft_last_2;
            RwcAft_last_2.copyTo(mTwcAft_last_2.rowRange(0,3).colRange(0,3));
            OwAft_last_2.copyTo(mTwcAft_last_2.rowRange(0,3).col(3));

            //cout << "mTwcAft_last_2 is calculated." << endl;
            
            mVelocity_last_Bef = (pKF_last->Tcw_current) * mTwcBef_last_2;
            mVelocity_last_Aft = (pKF_last->Tcw_new) * mTwcAft_last_2;

            cv::Mat mVelocity_last_Bef_Inverse = cv::Mat::eye(4,4,mVelocity_last_Bef.type());
            cv::Mat Rcw_mVelocity_last_Bef = mVelocity_last_Bef.rowRange(0,3).colRange(0,3);
            cv::Mat tcw_mVelocity_last_Bef = mVelocity_last_Bef.rowRange(0,3).col(3);
            cv::Mat Rwc_mVelocity_last_Bef = Rcw_mVelocity_last_Bef.t();
            cv::Mat Ow_mVelocity_last_Bef = -Rwc_mVelocity_last_Bef*tcw_mVelocity_last_Bef;
            Rwc_mVelocity_last_Bef.copyTo(mVelocity_last_Bef_Inverse.rowRange(0,3).colRange(0,3));
            Ow_mVelocity_last_Bef.copyTo(mVelocity_last_Bef_Inverse.rowRange(0,3).col(3));

            //cout << "mVelocity_last_Bef_Inverse is calculated." << endl;

            correct_effect = mVelocity_last_Aft * mVelocity_last_Bef_Inverse;

            //cout << "correct_effect is calculated." << endl;

            //cout << pKF->Tcw_current << endl;
            //cout << pKF->Tcw_current.type() << endl;
            //cout << mTwcBef_last.type() << endl;

            Bef_motion = (pKF->Tcw_current)*(mTwcBef_last);

            //cout << "Bef_motion is calculated." << endl;

            pKF->SetPose((Bef_motion * correct_effect) * (pKF_last->Tcw_new), true);
            goodKF_in_a_row = 0;
            //float GBA_scale = 1;
            //pKF->SetPose(Bef_motion * (pKF_last->Tcw_new) * GBA_scale, true);
            //}
            //translation_GBABef_last = (pKF_last->mTcwBefGBA.rowRange(0,3).col(3));
            //translation_GBAAft_last = (pKF_last->mTcwGBA.rowRange(0,3).col(3));
     
            //movement_GBABef = sqrt(pow(translation_GBABef_last.at<double>(0,0), 2.0) + pow(translation_GBABef_last.at<double>(0,1), 2.0) + pow(translation_GBABef_last.at<double>(0,2), 2.0));
            //movement_GBAAft = sqrt(pow(translation_GBAAft_last.at<double>(0,0), 2.0) + pow(translation_GBAAft_last.at<double>(0,1), 2.0) + pow(translation_GBAAft_last.at<double>(0,2), 2.0));

            //GBA_scale = movement_GBAAft / movement_GBABef;
            //float GBA_scale = 1;


            //pKF->SetPose(GBABef_motion * (pKF_last->mTcwGBA) * GBA_scale, true);

            //pKF->mbLoopCorrected = false;

            //pKF->mTcwGBA = GBABef_motion * (pKF_last->mTcwGBA) * GBA_scale;

            //pKF->mTcwGBA = (GBABef_motion * GBA_effect) * (pKF_last->mTcwGBA);

            bool motion_check_again = this->MotionCheck(clientId, pKF, Tws, pKF_last, pKF_last_2, last_X, last_Y, last_Z, tracking_last_X, tracking_last_Y, tracking_last_Z, X_diff, Y_diff, Z_diff, tracking_X_diff, tracking_Y_diff, tracking_Z_diff, degree_margin, movement_margin);

            degree_margin = degree_margin * 1.8;
            movement_margin = movement_margin * 1.2;
            cout << "KF count : " << KF_check_count << endl;
            badKF_in_a_row++;

            if (badKF_in_a_row > badKF_in_a_row_max) {
                badKF_in_a_row_max = badKF_in_a_row;
            }

            if (motion_check_again == false) {
                badKF_counter++;
                pKF->violate_flag = true;
            }
            else {
                //cout << "DEBUG : KF is corrected." << endl;
                corrected_counter++;
                pKF->violate_flag = false;
            }
        }
        else {
            X_diff = Tws(0,3) - last_X;
            Y_diff = Tws(1,3) - last_Y;
            Z_diff = Tws(2,3) - last_Z;
            last_X = Tws(0,3);
            last_Y = Tws(1,3);
            last_Z = Tws(2,3);

            tracking_X_diff = (pKF->tracking_x) - tracking_last_X;
            tracking_Y_diff = (pKF->tracking_y) - tracking_last_Y;
            tracking_Z_diff = (pKF->tracking_z) - tracking_last_Z;
            tracking_last_X = pKF->tracking_x;
            tracking_last_Y = pKF->tracking_y;
            tracking_last_Z = pKF->tracking_z;

            pKF->violate_flag = false;

            pKF_last_2 = pKF_last;
            pKF_last = pKF;

            goodKF_in_a_row++;
            badKF_in_a_row = 0;
            
            if (goodKF_in_a_row >= 2) {
                pKF->update_current = true;
            }
        }
    }
    else {
        X_diff = Tws(0,3) - last_X;
        Y_diff = Tws(1,3) - last_Y;
        Z_diff = Tws(2,3) - last_Z;
        last_X = Tws(0,3);
        last_Y = Tws(1,3);
        last_Z = Tws(2,3);

        tracking_X_diff = (pKF->tracking_x) - tracking_last_X;
        tracking_Y_diff = (pKF->tracking_y) - tracking_last_Y;
        tracking_Z_diff = (pKF->tracking_z) - tracking_last_Z;
        tracking_last_X = pKF->tracking_x;
        tracking_last_Y = pKF->tracking_y;
        tracking_last_Z = pKF->tracking_z;

        pKF_last_2 = pKF_last;
        pKF_last = pKF;
    }
    
    if (degree_margin > 3.0) {
        degree_margin = 3.0;
    }
    if (movement_margin > 3.0) {
        movement_margin = 3.0;
    }

    KF_check_count ++;
    pKF->Tcw_new = pKF->GetPose();
  }

  /*
  if (badKF_counter >= (foundKFs.size()/10) && request_GBA == false) {
    cout << COUTNOTICE << "request GBA because too many KFs are bad." << endl;
    request_GBA = true;
    badKF_counter = 0;
  }
  
  else if (badKF_counter >= (foundKFs.size()/10)) {
    cout << COUTNOTICE << "All KFs are set to tracking state because GBA hasn't improve the situation." << endl;
    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
        kfptr pKF = (*itr);
        pKF->SetPose(pKF->Tcw_original, true);
        pKF -> Tcw_current = pKF -> GetPose();
    }
  }
  */
  if (badKF_counter >= (foundKFs.size()/10) && isGBA == false) {
    cout << COUTNOTICE << "All KFs are set to tracking state because too many KFs are bad." << endl;
    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
        kfptr pKF = (*itr);
        pKF->SetPose(pKF->Tcw_original, true);
        pKF -> Tcw_current = pKF -> GetPose();

        // update tracking_x,y,z
        if (!pKF->isBad()) {
            cv::Mat T_wc_current = cv::Mat::eye(4,4, (pKF->GetPose()).type());
            cv::Mat mRwc = ((pKF->GetPose()).rowRange(0,3).colRange(0,3).t());
            mRwc.copyTo(T_wc_current.rowRange(0,3).colRange(0,3));
            (pKF -> GetCameraCenter()).copyTo(T_wc_current.rowRange(0,3).col(3)); 

            const Eigen::Matrix4d eT_wc_current = Converter::toMatrix4d(T_wc_current);

            Eigen::Matrix4d T_SC_current;
            T_SC_current = pKF -> mT_SC;

            const Eigen::Matrix4d Tws_current = eT_wc_current * T_SC_current.inverse();

            pKF -> tracking_x = Tws_current(0,3);
            pKF -> tracking_y = Tws_current(1,3);
            pKF -> tracking_z = Tws_current(2,3);
        }
    }
  }
  else if ((badKF_in_a_row_max > 5 || badKF_counter >= (foundKFs.size()/10)) && isGBA == true) {
    cout << COUTNOTICE << "All KFs are set to previous state because GBA didn't work well." << endl;
    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
        kfptr pKF = (*itr);
        pKF->SetPose(pKF->Tcw_current, true);

        // update tracking_x,y,z
        if (!pKF->isBad()) {
            cv::Mat T_wc_current = cv::Mat::eye(4,4, (pKF->GetPose()).type());
            cv::Mat mRwc = ((pKF->GetPose()).rowRange(0,3).colRange(0,3).t());
            mRwc.copyTo(T_wc_current.rowRange(0,3).colRange(0,3));
            (pKF -> GetCameraCenter()).copyTo(T_wc_current.rowRange(0,3).col(3)); 

            const Eigen::Matrix4d eT_wc_current = Converter::toMatrix4d(T_wc_current);

            Eigen::Matrix4d T_SC_current;
            T_SC_current = pKF -> mT_SC;

            const Eigen::Matrix4d Tws_current = eT_wc_current * T_SC_current.inverse();

            pKF -> tracking_x = Tws_current(0,3);
            pKF -> tracking_y = Tws_current(1,3);
            pKF -> tracking_z = Tws_current(2,3);
        }
    }
  }
  else {
    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
        kfptr pKF = (*itr);
        if (pKF->violate_flag == true) {
            pKF->SetBadFlag();
            deleted_counter++;
        }
        pKF -> Tcw_current = pKF -> GetPose();
    }
  }

  /*
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
    kfptr pKF = (*itr);
    // update tracking_x,y,z
    cv::Mat T_wc_current = cv::Mat::eye(4,4, (pKF->GetPose()).type());
    cv::Mat mRwc = ((pKF->GetPose()).rowRange(0,3).colRange(0,3).t());
    mRwc.copyTo(T_wc_current.rowRange(0,3).colRange(0,3));
    (pKF -> GetCameraCenter()).copyTo(T_wc_current.rowRange(0,3).col(3)); 

    const Eigen::Matrix4d eT_wc_current = Converter::toMatrix4d(T_wc_current);

    Eigen::Matrix4d T_SC_current;
    T_SC_current = pKF -> mT_SC;

    const Eigen::Matrix4d Tws_current = eT_wc_current * T_SC_current.inverse();

    pKF -> tracking_x = Tws_current(0,3);
    pKF -> tracking_y = Tws_current(1,3);
    pKF -> tracking_z = Tws_current(2,3);
  }
  */

  // for debug
  cout << "number of corrected KFs : " << corrected_counter << endl;
  cout << "number of deleted KFs : " << deleted_counter << endl;
  
  
  //initialize for next call
  corrected_counter = 0;
  deleted_counter = 0;
  KF_check_count = 0;
  degree_margin = 1;
  movement_margin = 1;
  update_flag = 1;
  violate_flag = 0;
  goodKF_in_a_row = 0;
  badKF_counter = 0;
  badKF_in_a_row = 0;
  badKF_in_a_row_max = 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////

void Map::CorrectKF_V2(const size_t clientId, std::map<idpair,kfptr> mmpKeyFrames, bool isGBA, bool reverse) {
    
  std::vector<kfptr> foundKFs;
  foundKFs.reserve(mmpKeyFrames.size());
  // Get all frames from the required client
  for (std::map<idpair,kfptr>::const_iterator itr = mmpKeyFrames.begin();
       itr != mmpKeyFrames.end(); ++itr) {
    kfptr pKF = itr->second;
    idpair currID = itr->first;
    if (currID.second == clientId) {
      foundKFs.push_back(pKF);
    }
  }
  if(foundKFs.empty()) //would overwrite files from other maps with empty files
    return;
  
  // Sort the keyframes by timestamp
  std::sort(foundKFs.begin(), foundKFs.end(), KeyFrame::compKFstamp);
  

  if (reverse == true) {
    std::reverse(foundKFs.begin(), foundKFs.end());
  }

  // kfptr for motion check (add this code (2023/12/8))
  kfptr pKF_last;
  kfptr pKF_last_2;

  // Check if KF violate motion
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
    kfptr pKF = (*itr);

    const cv::Mat T_wc = pKF->GetPoseInverse();
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();

    //for debug
    //cout << pKF->Tcw_current << endl;
    
    if (KF_check_count > 3) {
        bool motion_check = this->MotionCheck(clientId, pKF, Tws, pKF_last, pKF_last_2, last_X, last_Y, last_Z, tracking_last_X, tracking_last_Y, tracking_last_Z, X_diff, Y_diff, Z_diff, tracking_X_diff, tracking_Y_diff, tracking_Z_diff, degree_margin, movement_margin);

        if (motion_check == false) {
            /*
            // Twc (last KF, before)
            mTwcBef_last = cv::Mat::eye(4,4,pKF_last->Tcw_current.type());
            RcwBef_last = (pKF_last->Tcw_current).rowRange(0,3).colRange(0,3);
            tcwBef_last = (pKF_last->Tcw_current).rowRange(0,3).col(3);
            RwcBef_last = RcwBef_last.t();
            OwBef_last = -RwcBef_last*tcwBef_last;
            RwcBef_last.copyTo(mTwcBef_last.rowRange(0,3).colRange(0,3));
            OwBef_last.copyTo(mTwcBef_last.rowRange(0,3).col(3));
            
            // Twc (last KF, after)
            mTwcAft_last = cv::Mat::eye(4,4,pKF_last->Tcw_new.type());
            RcwAft_last = (pKF_last->Tcw_new).rowRange(0,3).colRange(0,3);
            tcwAft_last = (pKF_last->Tcw_new).rowRange(0,3).col(3);
            RwcAft_last = RcwAft_last.t();
            OwAft_last = -RwcAft_last*tcwAft_last;
            RwcAft_last.copyTo(mTwcAft_last.rowRange(0,3).colRange(0,3));
            OwAft_last.copyTo(mTwcAft_last.rowRange(0,3).col(3));

            // Twc (last_2 KF, before)
            mTwcBef_last_2 = cv::Mat::eye(4,4,pKF_last_2->Tcw_current.type());
            RcwBef_last_2 = (pKF_last_2->Tcw_current).rowRange(0,3).colRange(0,3);
            tcwBef_last_2 = (pKF_last_2->Tcw_current).rowRange(0,3).col(3);
            RwcBef_last_2 = RcwBef_last_2.t();
            OwBef_last_2 = -RwcBef_last_2*tcwBef_last_2;
            RwcBef_last_2.copyTo(mTwcBef_last_2.rowRange(0,3).colRange(0,3));
            OwBef_last_2.copyTo(mTwcBef_last_2.rowRange(0,3).col(3));

            // Twc (last_2 KF, after)
            mTwcAft_last_2 = cv::Mat::eye(4,4,pKF_last_2->Tcw_new.type());
            RcwAft_last_2 = (pKF_last_2->Tcw_new).rowRange(0,3).colRange(0,3);
            tcwAft_last_2 = (pKF_last_2->Tcw_new).rowRange(0,3).col(3);
            RwcAft_last_2 = RcwAft_last_2.t();
            OwAft_last_2 = -RwcAft_last_2*tcwAft_last_2;
            RwcAft_last_2.copyTo(mTwcAft_last_2.rowRange(0,3).colRange(0,3));
            OwAft_last_2.copyTo(mTwcAft_last_2.rowRange(0,3).col(3));
            
            //mVelocity_last_Bef = (pKF_last->Tcw_current) * mTwcBef_last_2;
            //mVelocity_last_Aft = (pKF_last->Tcw_new) * mTwcAft_last_2;

            //cv::Mat mVelocity_last_Bef_Inverse = cv::Mat::eye(4,4,mVelocity_last_Bef.type());
            //cv::Mat Rcw_mVelocity_last_Bef = mVelocity_last_Bef.rowRange(0,3).colRange(0,3);
            //cv::Mat tcw_mVelocity_last_Bef = mVelocity_last_Bef.rowRange(0,3).col(3);
            //cv::Mat Rwc_mVelocity_last_Bef = Rcw_mVelocity_last_Bef.t();
            //cv::Mat Ow_mVelocity_last_Bef = -Rwc_mVelocity_last_Bef*tcw_mVelocity_last_Bef;
            //Rwc_mVelocity_last_Bef.copyTo(mVelocity_last_Bef_Inverse.rowRange(0,3).colRange(0,3));
            //Ow_mVelocity_last_Bef.copyTo(mVelocity_last_Bef_Inverse.rowRange(0,3).col(3));

            //correct_effect = mVelocity_last_Aft * mVelocity_last_Bef_Inverse;
            //Bef_motion = (pKF->Tcw_current)*(mTwcBef_last);

            cv::Mat Tgba = (pKF_last->Tcw_new) * mTwcAft_last;
            */

           /*
            if (isGBA == true) {
                //pKF->SetPose((Bef_motion * correct_effect) * (pKF_last->Tcw_new), true);
                pKF->SetPose(Tgba * (pKF->Tcw_current), true);
                goodKF_in_a_row = 0;
                //float GBA_scale = 1;
                //pKF->SetPose(Bef_motion * (pKF_last->Tcw_new) * GBA_scale, true);

                //GBA_scale = movement_GBAAft / movement_GBABef;
                //float GBA_scale = 1;

                //pKF->SetPose(GBABef_motion * (pKF_last->mTcwGBA) * GBA_scale, true);

                //pKF->mbLoopCorrected = false;

                //pKF->mTcwGBA = GBABef_motion * (pKF_last->mTcwGBA) * GBA_scale;

                //pKF->mTcwGBA = (GBABef_motion * GBA_effect) * (pKF_last->mTcwGBA);
            }
            */

            bool motion_check_again = this->MotionCheck(clientId, pKF, Tws, pKF_last, pKF_last_2, last_X, last_Y, last_Z, tracking_last_X, tracking_last_Y, tracking_last_Z, X_diff, Y_diff, Z_diff, tracking_X_diff, tracking_Y_diff, tracking_Z_diff, degree_margin, movement_margin);

            degree_margin = degree_margin * 1.5;
            movement_margin = movement_margin * 1.5;
            cout << "KF count : " << KF_check_count << endl;
            //badKF_in_a_row++;

            /*
            if (badKF_in_a_row > badKF_in_a_row_max) {
                badKF_in_a_row_max = badKF_in_a_row;
            }
            */

            if (motion_check_again == false) {
                badKF_counter++;
                if (isGBA == true) {
                    pKF->violate_flag = true;
                }
            }
            else {
                //cout << "DEBUG : KF is corrected." << endl;
                corrected_counter++;
                pKF->violate_flag = false;
            }
        }
        else if (pKF->violate_flag == false) {
            X_diff = Tws(0,3) - last_X;
            Y_diff = Tws(1,3) - last_Y;
            Z_diff = Tws(2,3) - last_Z;
            last_X = Tws(0,3);
            last_Y = Tws(1,3);
            last_Z = Tws(2,3);

            tracking_X_diff = (pKF->tracking_x) - tracking_last_X;
            tracking_Y_diff = (pKF->tracking_y) - tracking_last_Y;
            tracking_Z_diff = (pKF->tracking_z) - tracking_last_Z;
            tracking_last_X = pKF->tracking_x;
            tracking_last_Y = pKF->tracking_y;
            tracking_last_Z = pKF->tracking_z;

            pKF->violate_flag = false;

            pKF_last_2 = pKF_last;
            pKF_last = pKF;

            goodKF_in_a_row++;
            //badKF_in_a_row = 0;
            
            if (goodKF_in_a_row >= 2) {
                pKF->update_current = true;
            }
        }
    }
    else {
        X_diff = Tws(0,3) - last_X;
        Y_diff = Tws(1,3) - last_Y;
        Z_diff = Tws(2,3) - last_Z;
        last_X = Tws(0,3);
        last_Y = Tws(1,3);
        last_Z = Tws(2,3);

        tracking_X_diff = (pKF->tracking_x) - tracking_last_X;
        tracking_Y_diff = (pKF->tracking_y) - tracking_last_Y;
        tracking_Z_diff = (pKF->tracking_z) - tracking_last_Z;
        tracking_last_X = pKF->tracking_x;
        tracking_last_Y = pKF->tracking_y;
        tracking_last_Z = pKF->tracking_z;

        pKF_last_2 = pKF_last;
        pKF_last = pKF;
    }
    
    if (degree_margin > 3.0) {
        degree_margin = 3.0;
    }
    if (movement_margin > 3.0) {
        movement_margin = 3.0;
    }

    KF_check_count ++;
    //pKF->Tcw_new = pKF->GetPose();
  }
  cout << "motion check is completed." << endl;

  if (badKF_counter >= (foundKFs.size()/10) && isGBA == false) {
    cout << COUTNOTICE << "All KFs are checked by loose conditions because too many KFs are bad." << endl;
    /*
    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
        kfptr pKF = (*itr);
        pKF->SetPose(pKF->Tcw_original, true);

        // update tracking_x,y,z
        if (!pKF->isBad()) {
            cv::Mat T_wc_current = cv::Mat::eye(4,4, (pKF->GetPose()).type());
            cv::Mat mRwc = ((pKF->GetPose()).rowRange(0,3).colRange(0,3).t());
            mRwc.copyTo(T_wc_current.rowRange(0,3).colRange(0,3));
            (pKF -> GetCameraCenter()).copyTo(T_wc_current.rowRange(0,3).col(3)); 

            const Eigen::Matrix4d eT_wc_current = Converter::toMatrix4d(T_wc_current);

            Eigen::Matrix4d T_SC_current;
            T_SC_current = pKF -> mT_SC;

            const Eigen::Matrix4d Tws_current = eT_wc_current * T_SC_current.inverse();

            pKF -> tracking_x = Tws_current(0,3);
            pKF -> tracking_y = Tws_current(1,3);
            pKF -> tracking_z = Tws_current(2,3);
        }
    }
    */
    deleted_counter = this->TrimMVKF_loose(clientId, foundKFs);
  }
  /*
  else if ((badKF_in_a_row_max >= 5 || badKF_counter >= (foundKFs.size()/10)) && isGBA == true) {
    cout << COUTNOTICE << "All KFs are set to previous state because GBA didn't work well." << endl;
    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
        kfptr pKF = (*itr);
        pKF->SetPose(pKF->Tcw_current, true);
        
        if (reverse == true) {
            // update tracking_x,y,z
            if (!pKF->isBad()) {
                cv::Mat T_wc_current = cv::Mat::eye(4,4, (pKF->GetPose()).type());
                cv::Mat mRwc = ((pKF->GetPose()).rowRange(0,3).colRange(0,3).t());
                mRwc.copyTo(T_wc_current.rowRange(0,3).colRange(0,3));
                (pKF -> GetCameraCenter()).copyTo(T_wc_current.rowRange(0,3).col(3)); 

                const Eigen::Matrix4d eT_wc_current = Converter::toMatrix4d(T_wc_current);

                Eigen::Matrix4d T_SC_current;
                T_SC_current = pKF -> mT_SC;

                const Eigen::Matrix4d Tws_current = eT_wc_current * T_SC_current.inverse();

                pKF -> tracking_x = Tws_current(0,3);
                pKF -> tracking_y = Tws_current(1,3);
                pKF -> tracking_z = Tws_current(2,3);
            }
        }
    }
  }
  */
  else if (isGBA == true && reverse == true){
    cout << "CheckMVKF is called." << endl;
    badKF_in_a_row = CheckMVKF(foundKFs);
    if ((badKF_in_a_row >= 5 || badKF_counter >= (foundKFs.size()/10)) && isGBA == true) {
        cout << COUTNOTICE << "All KFs are set to previous state because GBA didn't work well." << endl;
        for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
            kfptr pKF = (*itr);
            pKF->SetPose(pKF->Tcw_current, true);
        }
    }
    else {
        cout << "DeleteMVKF is called." << endl;
        deleted_counter = DeleteMVKF(foundKFs);
    }
  }

  // for debug
  if (isGBA == true) {
    cout << "number of corrected KFs : " << corrected_counter << endl;
    cout << "number of deleted KFs : " << deleted_counter << endl;
  }
  else {
    cout << "After Loop Found : Check Completed - No Problem." << endl;
  }
  
  
  //initialize for next call
  corrected_counter = 0;
  deleted_counter = 0;
  KF_check_count = 0;
  degree_margin = 1;
  movement_margin = 1;
  update_flag = 1;
  violate_flag = 0;
  goodKF_in_a_row = 0;
  badKF_counter = 0;
  badKF_in_a_row = 0;
  badKF_in_a_row_max = 0;
}

Eigen::Matrix4d Map::ComputeTws(kfptr pKF) {
    const cv::Mat T_wc = pKF->GetPoseInverse();
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();

    return Tws;
}

void Map::CorrectKF_V3(const size_t clientId, std::map<idpair,kfptr> mmpKeyFrames, bool isGBA, bool reverse) {
    
  std::vector<kfptr> foundKFs;
  foundKFs.reserve(mmpKeyFrames.size());
  // Get all frames from the required client
  for (std::map<idpair,kfptr>::const_iterator itr = mmpKeyFrames.begin();
       itr != mmpKeyFrames.end(); ++itr) {
    kfptr pKF = itr->second;
    idpair currID = itr->first;
    if (currID.second == clientId) {
      foundKFs.push_back(pKF);
    }
  }
  if(foundKFs.empty()) //would overwrite files from other maps with empty files
    return;
  
  // Sort the keyframes by timestamp
  std::sort(foundKFs.begin(), foundKFs.end(), KeyFrame::compKFstamp);
  

  if (reverse == true) {
    std::reverse(foundKFs.begin(), foundKFs.end());
  }

  // kfptr for motion check
  kfptr pKF_last;
  kfptr pKF_last_2;
  kfptr pKF_last_3;

  // Check if KF violate motion
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
    kfptr pKF = (*itr);
    KF_check_count++;

    /*
    const cv::Mat T_wc = pKF->GetPoseInverse();
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
    */

   const Eigen::Matrix4d Tws = this->ComputeTws(pKF);
    
    if ((KF_check_count > 10) && (KF_check_count < (foundKFs.size() - 10))) {
        if ((pKF->doubt_for_MV >= 2) && ((itr+1) != foundKFs.end())) {
            itr ++;
            pKF = (*itr);
        }
        else if ((pKF->doubt_for_MV >= 2) && ((itr+1) == foundKFs.end())) {
            continue;
        }

        bool motion_check = this->MotionCheck(clientId, pKF, Tws, pKF_last, pKF_last_2, last_X, last_Y, last_Z, tracking_last_X, tracking_last_Y, tracking_last_Z, X_diff, Y_diff, Z_diff, tracking_X_diff, tracking_Y_diff, tracking_Z_diff, degree_margin, movement_margin);

        if (motion_check == false) {
            //cout << "KF count : " << KF_check_count << endl;
            badKF_counter++;
            if (isGBA == true) {
                pKF->doubt_for_MV++;
                pKF_last->doubt_for_MV ++;
            }
        }

        if ((pKF_last->doubt_for_MV >= 2 || pKF_last_2->doubt_for_MV >= 2) && reverse == true) {
            std::vector<kfptr>::const_iterator temp;
            temp = itr;
            while (!(pKF_last->doubt_for_MV >=2)) {
                pKF_last = (*temp);
                temp--;
            }
            degree_margin = degree_margin * 1.5;
            movement_margin = movement_margin * 1.5;
        }
        else {
            degree_margin = 1;
            movement_margin = 1;
        }

        const Eigen::Matrix4d Tws_last = this->ComputeTws(pKF_last);
        const Eigen::Matrix4d Tws_last_2 = this->ComputeTws(pKF_last_2);
        X_diff = Tws(0,3) - Tws_last(0,3);
        Y_diff = Tws(1,3) - Tws_last(1,3);
        Z_diff = Tws(2,3) - Tws_last(2,3);
        last_X = Tws(0,3);
        last_Y = Tws(1,3);
        last_Z = Tws(2,3);

        tracking_X_diff = (pKF->tracking_x) - (pKF_last->tracking_x);
        tracking_Y_diff = (pKF->tracking_y) - (pKF_last->tracking_y);
        tracking_Z_diff = (pKF->tracking_z) - (pKF_last->tracking_z);
        tracking_last_X = pKF->tracking_x;
        tracking_last_Y = pKF->tracking_y;
        tracking_last_Z = pKF->tracking_z;
        pKF_last_2 = pKF_last;
        pKF_last = pKF;
    }
    else {
        
        X_diff = Tws(0,3) - last_X;
        Y_diff = Tws(1,3) - last_Y;
        Z_diff = Tws(2,3) - last_Z;
        last_X = Tws(0,3);
        last_Y = Tws(1,3);
        last_Z = Tws(2,3);

        tracking_X_diff = (pKF->tracking_x) - tracking_last_X;
        tracking_Y_diff = (pKF->tracking_y) - tracking_last_Y;
        tracking_Z_diff = (pKF->tracking_z) - tracking_last_Z;
        tracking_last_X = pKF->tracking_x;
        tracking_last_Y = pKF->tracking_y;
        tracking_last_Z = pKF->tracking_z;
        
        pKF_last_2 = pKF_last;
        pKF_last = pKF;
    }
    
    if (degree_margin > 3.0) {
        degree_margin = 3.0;
    }
    if (movement_margin > 3.0) {
        movement_margin = 3.0;
    }
  }
  cout << "motion check is completed." << endl;
  /*
  if (badKF_counter >= (foundKFs.size()/10) && isGBA == false) {
    cout << COUTNOTICE << "All KFs are checked by loose conditions because too many KFs are bad." << endl;
    deleted_counter = this->TrimMVKF_loose(clientId, foundKFs);
  }
  */

  if (isGBA == false && reverse == true) {
    SetViolateFlag(foundKFs);
    cout << "CheckMVKF is called." << endl;
    badKF_in_a_row = CheckMVKF(foundKFs);
    
    if ((badKF_in_a_row >= 5 || badKF_counter >= (foundKFs.size()/10))) {
        cout << COUTNOTICE << "CorrectKF will not work." << endl;
        CorrectKF_switch = false;
    }

    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
        kfptr pKF = (*itr);
        pKF->violate_flag = false;
        pKF->doubt_for_MV = 0;
    }
  }

  if (isGBA == true && reverse == true && CorrectKF_switch == true){
    SetViolateFlag(foundKFs);
    cout << "CheckMVKF is called." << endl;
    badKF_in_a_row = CheckMVKF(foundKFs);
    if ((badKF_in_a_row >= 5 || badKF_counter >= (foundKFs.size()/10)) && isGBA == true) {
        cout << COUTNOTICE << "All KFs are set to previous state because GBA didn't work well." << endl;
        for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
            kfptr pKF = (*itr);
            pKF->SetPose(pKF->Tcw_current, true);
        }
    }
    else {
        cout << "DeleteMotionViolateKF is called." << endl;
        deleted_counter = DeleteMotionViolateKF(foundKFs);
    }
  }

  // for debug
  if (isGBA == true) {
    cout << "number of corrected KFs : " << corrected_counter << endl;
    cout << "number of deleted KFs : " << deleted_counter << endl;
  }
  else {
    cout << "After Loop Found : Check Completed - No Problem." << endl;
  }
  
  
  //initialize for next call
  corrected_counter = 0;
  deleted_counter = 0;
  KF_check_count = 0;
  degree_margin = 1;
  movement_margin = 1;
  update_flag = 1;
  violate_flag = 0;
  goodKF_in_a_row = 0;
  badKF_counter = 0;
  badKF_in_a_row = 0;
  badKF_in_a_row_max = 0;
}

int Map::DeleteMVKF(std::vector<kfptr> foundKFs) {
    kfptr pKF_last;
    kfptr pKF_last_2;
    kfptr pKF_last_3;
    int delete_counter = 0;
    int check_counter = 0;
    //cout << "check A" << endl;

    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
        //cout << "check B-1" << endl;
        kfptr pKF = (*itr);
        //cout << "check B-2" << endl;
        if (pKF->violate_flag == true && check_counter >= 4) {
            if (pKF_last_2->violate_flag == true) {
                pKF_last->violate_flag = true;
            }
            if (pKF_last_3->violate_flag == true) {
                pKF_last_2->violate_flag = true;
                pKF_last->violate_flag = true;
            }
        }
        check_counter++;
        pKF_last_3 = pKF_last_2;
        pKF_last_2 = pKF_last;
        pKF_last = pKF;
    }
    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
        kfptr pKF = (*itr);
        if (pKF->violate_flag == true) {
            delete_counter++;
            pKF->SetBadFlag();
        }
    }
    //cout << "check C" << endl;
    return delete_counter;
}

void Map::SetViolateFlag(std::vector<kfptr> foundKFs) {
    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
        kfptr pKF = (*itr);
        if (pKF->doubt_for_MV >= 2) {
            pKF->violate_flag = true;
        }
    }

    /*
    kfptr pKF_last;
    kfptr pKF_last_2;
    kfptr pKF_last_3;
    kfptr pKF_last_4;
    int delete_counter = 0;
    int check_counter = 0;

    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
        kfptr pKF = (*itr);
        if (pKF->violate_flag == true && check_counter >= 5) {
            if (pKF_last_2->violate_flag == true) {
                pKF_last->violate_flag = true;
            }
            if (pKF_last_3->violate_flag == true) {
                pKF_last_2->violate_flag = true;
                pKF_last->violate_flag = true;
            }
            if (pKF_last_4->violate_flag == true) {
                pKF_last_3->violate_flag = true;
                pKF_last_2->violate_flag = true;
                pKF_last->violate_flag = true;
            }
        }
        check_counter++;
        pKF_last_4 = pKF_last_3;
        pKF_last_3 = pKF_last_2;
        pKF_last_2 = pKF_last;
        pKF_last = pKF;
    }
    */
}

int Map::DeleteMotionViolateKF(std::vector<kfptr> foundKFs) {
    int delete_counter = 0;
    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
        kfptr pKF = (*itr);
        if (pKF->violate_flag == true) {
            pKF->SetBadFlag();
            delete_counter++;
        }
    }
    return delete_counter;
}

int Map::CheckMVKF(std::vector<kfptr> foundKFs) {
    kfptr pKF;
    int bad_counter = 0;
    int bad_counter_max = 0;
    int counter = 0;

    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
        pKF = (*itr);
        counter++;
        if (pKF->violate_flag == true) {
            bad_counter++;
            //cout << "CheckMVKF counter : " << counter << endl;
            if (bad_counter > bad_counter_max) {
                bad_counter_max = bad_counter;
            }
        }
        else {
            bad_counter = 0;
        }
    }
    return bad_counter_max;
}

int Map::TrimMVKF_loose(const size_t clientId, std::vector<kfptr> foundKFs) {
  // kfptr for motion check
  kfptr pKF_last;
  kfptr pKF_last_2;
  int check_counter = 0;
  int delete_counter = 0;

  // Check if KF violate motion
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
    kfptr pKF = (*itr);

    const cv::Mat T_wc = pKF->GetPoseInverse();
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
    
    if (check_counter > 3) {
        bool motion_check = this->MotionCheck(clientId, pKF, Tws, pKF_last, pKF_last_2, last_X, last_Y, last_Z, tracking_last_X, tracking_last_Y, tracking_last_Z, X_diff, Y_diff, Z_diff, tracking_X_diff, tracking_Y_diff, tracking_Z_diff, 3, 3);

        if (motion_check == false) {
            pKF->SetBadFlag();
            delete_counter++;
        }
        else {
            X_diff = Tws(0,3) - last_X;
            Y_diff = Tws(1,3) - last_Y;
            Z_diff = Tws(2,3) - last_Z;
            last_X = Tws(0,3);
            last_Y = Tws(1,3);
            last_Z = Tws(2,3);

            tracking_X_diff = (pKF->tracking_x) - tracking_last_X;
            tracking_Y_diff = (pKF->tracking_y) - tracking_last_Y;
            tracking_Z_diff = (pKF->tracking_z) - tracking_last_Z;
            tracking_last_X = pKF->tracking_x;
            tracking_last_Y = pKF->tracking_y;
            tracking_last_Z = pKF->tracking_z;

            pKF_last_2 = pKF_last;
            pKF_last = pKF;
        }
    }
    else {
        X_diff = Tws(0,3) - last_X;
        Y_diff = Tws(1,3) - last_Y;
        Z_diff = Tws(2,3) - last_Z;
        last_X = Tws(0,3);
        last_Y = Tws(1,3);
        last_Z = Tws(2,3);

        tracking_X_diff = (pKF->tracking_x) - tracking_last_X;
        tracking_Y_diff = (pKF->tracking_y) - tracking_last_Y;
        tracking_Z_diff = (pKF->tracking_z) - tracking_last_Z;
        tracking_last_X = pKF->tracking_x;
        tracking_last_Y = pKF->tracking_y;
        tracking_last_Z = pKF->tracking_z;

        pKF_last_2 = pKF_last;
        pKF_last = pKF;
    }

    check_counter ++;
  }
  return delete_counter;
}

void Map::SetTcwCurrent(const size_t clientId, std::map<idpair,kfptr> mmpKeyFrames) {
    std::vector<kfptr> foundKFs;
    cv::Mat Ow = cv::Mat(3,1,5);
    foundKFs.reserve(mmpKeyFrames.size());
    // Get all frames from the required client
    for (std::map<idpair,kfptr>::const_iterator itr = mmpKeyFrames.begin();
        itr != mmpKeyFrames.end(); ++itr) {
        kfptr pKF = itr->second;
        idpair currID = itr->first;
        if (currID.second == clientId) {
        foundKFs.push_back(pKF);
        }
    }
    if(foundKFs.empty())
        return;
  
    // Sort the keyframes by timestamp
    std::sort(foundKFs.begin(), foundKFs.end(), KeyFrame::compKFstamp);

    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
        kfptr pKF = (*itr);
        pKF->Tcw_current = pKF->GetPose();

        const cv::Mat T_wc_current = cv::Mat::eye(4,4,(pKF->Tcw_current).type());
        cv::Mat Rcw = (pKF->Tcw_current).rowRange(0,3).colRange(0,3);
        cv::Mat tcw = (pKF->Tcw_current).rowRange(0,3).col(3);
        cv::Mat Rwc = Rcw.t();
        Ow = -Rwc*tcw;
        Rwc.copyTo(T_wc_current.rowRange(0,3).colRange(0,3));
        Ow.copyTo(T_wc_current.rowRange(0,3).col(3));

        const Eigen::Matrix4d eT_wc_current = Converter::toMatrix4d(T_wc_current);

        const Eigen::Matrix4d Tws_current = eT_wc_current * (pKF->mT_SC).inverse();

        pKF->tracking_x = Tws_current(0,3);
        pKF->tracking_y = Tws_current(1,3);
        pKF->tracking_z = Tws_current(2,3);
    }
}

void Map::SetTcwNew(const size_t clientId, std::map<idpair,kfptr> mmpKeyFrames) {
    std::vector<kfptr> foundKFs;
    foundKFs.reserve(mmpKeyFrames.size());
    // Get all frames from the required client
    for (std::map<idpair,kfptr>::const_iterator itr = mmpKeyFrames.begin();
        itr != mmpKeyFrames.end(); ++itr) {
        kfptr pKF = itr->second;
        idpair currID = itr->first;
        if (currID.second == clientId) {
        foundKFs.push_back(pKF);
        }
    }
    if(foundKFs.empty())
        return;
  
    // Sort the keyframes by timestamp
    std::sort(foundKFs.begin(), foundKFs.end(), KeyFrame::compKFstamp);

    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); itr++) {
        kfptr pKF = (*itr);
        pKF->Tcw_new = pKF->GetPose();
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////

void Map::WriteStateToCsv(const std::string& filename,
                          const size_t clientId) {  //  modify this code (2023/11/7)  //////////////////
  std::vector<kfptr> foundKFs;
  foundKFs.reserve(mmpKeyFrames.size());
  // Get all frames from the required client
  for (std::map<idpair,kfptr>::const_iterator itr = mmpKeyFrames.begin();
       itr != mmpKeyFrames.end(); ++itr) {
    kfptr pKF = itr->second;
    idpair currID = itr->first;
    if (currID.second == clientId) {
      foundKFs.push_back(pKF);
    }
  }

  if(foundKFs.empty()) //would overwrite files from other maps with empty files
      return;

  // Sort the keyframes by timestamp
  std::sort(foundKFs.begin(), foundKFs.end(), KeyFrame::compKFstamp);

  // for debug
  //cout << "foundKFs' size (before) : " << foundKFs.size() << endl;

  // Write out the keyframe data
  std::ofstream keyframesFile;
  keyframesFile.open(filename);

  violate_check_by_three_switch = 0; // 1 : ON   /   0 : OFF

  if (violate_check_by_three_switch == 1) {
    // Check if KF violate motion
    for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ) {
        kfptr pKF = (*itr);
        const cv::Mat T_wc = pKF->GetPoseInverse();
        const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
        ccptr pCC = *(mspCC.begin());
        Eigen::Matrix4d T_SC;
        if(mSysState == SERVER)
            T_SC = pKF->mT_SC;
        else
            T_SC = pCC->mT_SC;

        const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
        const Eigen::Quaterniond q(Tws.block<3,3>(0,0));

        // add this code (2023/11/5) ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        if (KF_check_count != 0) {
            abs_X_total += abs(Tws(0,3) - last_X);
            abs_X_average = abs_X_total / KF_check_count;

            abs_Y_total += abs(Tws(1,3) - last_Y);
            abs_Y_average = abs_Y_total / KF_check_count;

            abs_Z_total += abs(Tws(2,3) - last_Z);
            abs_Z_average = abs_Z_total / KF_check_count;

            if (abs(Tws(0,3) - last_X) > max_abs_X) {
                max_abs_X = abs(Tws(0,3) - last_X);
                remember_X = Tws(0,3);
                remember_last_X = last_X;
                remember_past_three_X_diff = past_three_X_diff;
                }
            if (abs(Tws(1,3) - last_Y) > max_abs_Y) {
                max_abs_Y = abs(Tws(1,3) - last_Y);
                remember_Y = Tws(1,3);
                remember_last_Y = last_Y;
            }
            if (abs(Tws(2,3) - last_Z) > max_abs_Z) {
                max_abs_Z = abs(Tws(2,3) - last_Z);
                remember_Z = Tws(2,3);
                remember_last_Z = last_Z;
            }
        }

        if (KF_check_count >= 3) {
            if ((Tws(0,3) - last_X) <= (X_diff_1 * (-1) * 2) && (Tws(0,3) - last_X) >= (X_diff_1 * (-1) * 0.5) && violate_flag_X == 1) {
                //TrimMotionViolateKF(violate_KF_candidate);
                //foundKFs = DeleteMotionViolateKF(foundKFs, violate_KF_candidate);
                for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
                    kfptr pKF = (*itr);
                    if (pKF == violate_KF_candidate) {
                        foundKFs.erase(itr);
                        erase_flag = 1;
                        cout << "DEBUG : KF is Deleted from foundKFs (X)." << endl;
                    }
                }
                violate_flag_X = 0;
                violate_flag_Y = 0;
                violate_flag_Z = 0;
            }
            if ((Tws(1,3) - last_Y) <= (Y_diff_1 * (-1) * 2) && (Tws(1,3) - last_Y) >= (Y_diff_1 * (-1) * 0.5) && violate_flag_Y == 1) {
                //TrimMotionViolateKF(violate_KF_candidate);
                //foundKFs = DeleteMotionViolateKF(foundKFs, violate_KF_candidate);
                for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
                    kfptr pKF = (*itr);
                    if (pKF == violate_KF_candidate) {
                        foundKFs.erase(itr);
                        erase_flag = 1;
                        cout << "DEBUG : KF is Deleted from foundKFs (Y)." << endl;
                    }
                }
                violate_flag_X = 0;
                violate_flag_Y = 0;
                violate_flag_Z = 0;
            }
            if ((Tws(2,3) - last_Z) <= (Z_diff_1 * (-1) * 2) && (Tws(2,3) - last_Z) >= (Z_diff_1 * (-1) * 0.5) && violate_flag_Z == 1) {
                //TrimMotionViolateKF(violate_KF_candidate);
                //foundKFs = DeleteMotionViolateKF(foundKFs, violate_KF_candidate);
                for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
                    kfptr pKF = (*itr);
                    if (pKF == violate_KF_candidate) {
                        foundKFs.erase(itr);
                        erase_flag = 1;
                        cout << "DEBUG : KF is Deleted from foundKFs (Z)." << endl;
                    }
                }
                violate_flag_X = 0;
                violate_flag_Y = 0;
                violate_flag_Z = 0;
            }

            //if ((((Tws(0,3) - last_X) >= 0) != (X_diff_1 >=0 )) && violate_flag_X == 1) {
                //TrimMotionViolateKF(violate_KF_candidate);
                //foundKFs = DeleteMotionViolateKF(foundKFs, violate_KF_candidate);
                //for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
                    //kfptr pKF = (*itr);
                    //if (pKF == violate_KF_candidate) {
                        //foundKFs.erase(itr);
                        //cout << "DEBUG : KF is Deleted from foundKFs (X)." << endl;
                    //}
                //}
                //violate_flag_X = 0;
                //violate_flag_Y = 0;
                //violate_flag_Z = 0;
            //}
            //if ((((Tws(1,3) - last_Y) >= 0) != (Y_diff_1 >=0 )) && violate_flag_Y == 1) {
                //TrimMotionViolateKF(violate_KF_candidate);
                //foundKFs = DeleteMotionViolateKF(foundKFs, violate_KF_candidate);
                //for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
                    //kfptr pKF = (*itr);
                    //if (pKF == violate_KF_candidate) {
                        //foundKFs.erase(itr);
                        //cout << "DEBUG : KF is Deleted from foundKFs (Y)." << endl;
                    //}
                //}
                //violate_flag_X = 0;
                //violate_flag_Y = 0;
                //violate_flag_Z = 0;
            //}
            //if ((((Tws(2,3) - last_Z) >= 0) != (Z_diff_1 >=0 )) && violate_flag_Z == 1) {
                //TrimMotionViolateKF(violate_KF_candidate);
                //foundKFs = DeleteMotionViolateKF(foundKFs, violate_KF_candidate);
                //for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
                    //kfptr pKF = (*itr);
                    //if (pKF == violate_KF_candidate) {
                        //foundKFs.erase(itr);
                        //cout << "DEBUG : KF is Deleted from foundKFs (Z)." << endl;
                    //}
                //}
                //violate_flag_X = 0;
                //violate_flag_Y = 0;
                //violate_flag_Z = 0;
            //}

            if (abs(Tws(0,3) - last_X) >= (past_three_X_diff * 10)) {
                violate_KF_candidate = pKF;
                violate_flag_X = 1;
            }
            if (abs(Tws(1,3) - last_Y) >= (past_three_Y_diff * 10)) {
                violate_KF_candidate = pKF;
                violate_flag_Y = 1;
            }
            if (abs(Tws(2,3) - last_Z) >= (past_three_Z_diff * 10)) {
                violate_KF_candidate = pKF;
                violate_flag_Z = 1;
            }
        }

        X_diff_3 = X_diff_2;
        X_diff_2 = X_diff_1;
        X_diff_1 = Tws(0,3) - last_X;
        past_three_X_diff = (abs(X_diff_1) + abs(X_diff_2) + abs(X_diff_3)) / 3;

        Y_diff_3 = Y_diff_2;
        Y_diff_2 = Y_diff_1;
        Y_diff_1 = Tws(1,3) - last_Y;
        past_three_Y_diff = (abs(Y_diff_1) + abs(Y_diff_2) + abs(Y_diff_3)) / 3;

        Z_diff_3 = Z_diff_2;
        Z_diff_2 = Z_diff_1;
        Z_diff_1 = Tws(2,3) - last_Z;
        past_three_Z_diff = (abs(Z_diff_1) + abs(Z_diff_2) + abs(Z_diff_3)) / 3;

        KF_check_count ++;
        last_X = Tws(0,3);
        last_Y = Tws(1,3);
        last_Z = Tws(2,3);

        if (erase_flag == 0) {
            itr++;
        }
        else {
            erase_flag = 0;
        }
    }
  }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // for debug
  //cout << "foundKFs' size (after) : " << foundKFs.size() << endl;
  
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
    kfptr pKF = (*itr);
    const double stamp = pKF->mTimeStamp;
    Eigen::Vector3d bA = Eigen::Vector3d::Zero();
    Eigen::Vector3d bG = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    const cv::Mat T_wc = pKF->GetPoseInverse();
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
    const Eigen::Quaterniond q(Tws.block<3,3>(0,0));
    if(params::stats::miTrajectoryFormat == 0) {
        keyframesFile << std::setprecision(25) << stamp * 1e9f << ",";
        keyframesFile << Tws(0,3) << "," << Tws(1,3) << "," << Tws(2,3) << ",";
        keyframesFile << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ",";
        keyframesFile << vel[0] << "," << vel[1] << "," << vel[2] << ",";
        keyframesFile << bG[0] << "," << bG[1] << "," << bG[2] << ",";
        keyframesFile << bA[0] << "," << bA[1] << "," << bA[2] << std::endl;
    } else if(params::stats::miTrajectoryFormat == 1) {
        keyframesFile << std::setprecision(25) << stamp << " ";
        keyframesFile << Tws(0,3) << " " << Tws(1,3) << " " << Tws(2,3) << " ";
        keyframesFile << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        // add this code (2023/11/5) /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (KF_check_count != 0) {
            abs_X_total += abs(Tws(0,3) - last_X);
            abs_X_average = abs_X_total / KF_check_count;

            abs_Y_total += abs(Tws(1,3) - last_Y);
            abs_Y_average = abs_Y_total / KF_check_count;

            abs_Z_total += abs(Tws(2,3) - last_Z);
            abs_Z_average = abs_Z_total / KF_check_count;

            if (abs(Tws(0,3) - last_X) > max_abs_X) {
                max_abs_X = abs(Tws(0,3) - last_X);
                remember_X = Tws(0,3);
                remember_last_X = last_X;
                remember_past_three_X_diff = past_three_X_diff;
                }
            if (abs(Tws(1,3) - last_Y) > max_abs_Y) {
                max_abs_Y = abs(Tws(1,3) - last_Y);
                remember_Y = Tws(1,3);
                remember_last_Y = last_Y;
            }
            if (abs(Tws(2,3) - last_Z) > max_abs_Z) {
                max_abs_Z = abs(Tws(2,3) - last_Z);
                remember_Z = Tws(2,3);
                remember_last_Z = last_Z;
            }
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    } else {
        std::cout << COUTFATAL << "miTrajectoryFormat '" << params::stats::miTrajectoryFormat << "' not in { 0=EUROC | 1=TUM }" << std::endl;
        exit(-1);
    }
  }
  keyframesFile.close();

  // add this code (2023/11/3) /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (violate_check_by_three_switch == 1) {
    cout << "Motion Violate Check (by past three) : active" << endl;
  }
  else {
    // do nothing    
  }
  
  //cout << "max_abs_X (KFwrite) : " << max_abs_X << endl;
  //cout << "average_X (past_three) : " << remember_past_three_X_diff << endl;
  //cout << "average_X : " << abs_X_average << endl;
  //cout << "remember_X : " << remember_X << endl;
  //cout << "remember_last_X : " << remember_last_X << endl;
  //cout << endl;

  //cout << "max_abs_Y (KFwrite) : " << max_abs_Y << endl;
  //cout << "average_Y : " << abs_Y_average << endl;
  //cout << "remember_Y : " << remember_Y << endl;
  //cout << "remember_last_Y : " << remember_last_Y << endl;
  //cout << endl;

  //cout << "max_abs_Z (KFwrite) : " << max_abs_Z << endl;
  //cout << "average_Z : " << abs_Z_average << endl;
  //cout << "remember_Z : " << remember_Z << endl;
  //cout << "remember_last_Z : " << remember_last_Z << endl;
  //cout << endl;

  KF_check_count = 0;

  abs_X_total = 0;
  abs_Y_total = 0;
  abs_Z_total = 0;

  abs_X_average = 0;
  abs_Y_average = 0;
  abs_Z_average = 0;

  max_abs_X = 0;
  max_abs_Y = 0;
  max_abs_Z = 0;

  remember_X = 0;
  remember_Y = 0;
  remember_Z = 0;

  remember_last_X = 0;
  remember_last_Y = 0;
  remember_last_Z = 0;

  X_diff_1 = 0;
  X_diff_2 = 0;
  X_diff_3 = 0;

  Y_diff_1 = 0;
  Y_diff_2 = 0;
  Y_diff_3 = 0;

  Z_diff_1 = 0;
  Z_diff_2 = 0;
  Z_diff_3 = 0;
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::cout << "KFs written to file : " << filename << std::endl;
  std::cout << "KFs in Map : " << mmpKeyFrames.size() << std::endl;
}

// add this code (2023/11/10) ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Map::WriteStateToCsvTrimmedV2(const std::string& filename,
                          const size_t clientId) {
  std::vector<kfptr> foundKFs;
  foundKFs.reserve(mmpKeyFrames.size());
  // Get all frames from the required client
  for (std::map<idpair,kfptr>::const_iterator itr = mmpKeyFrames.begin();
       itr != mmpKeyFrames.end(); ++itr) {
    kfptr pKF = itr->second;
    idpair currID = itr->first;
    if (currID.second == clientId) {
      foundKFs.push_back(pKF);
    }
  }

  if(foundKFs.empty()) //would overwrite files from other maps with empty files
      return;

  // Sort the keyframes by timestamp
  std::sort(foundKFs.begin(), foundKFs.end(), KeyFrame::compKFstamp);

  // for debug
  cout << "foundKFs' size (before) : " << foundKFs.size() << endl;

  // Write out the keyframe data
  std::ofstream keyframesFile;
  keyframesFile.open(filename);

  // Check if KF violate motion
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ) {
    kfptr pKF = (*itr);
    // add this code (2023/11/16) /////////////////////////////////////////////////////////////////////////////
    //(pKF -> Twc_copy) = (pKF -> GetPoseInverse());
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    const cv::Mat T_wc = pKF->GetPoseInverse();
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
    const Eigen::Quaterniond q(Tws.block<3,3>(0,0));

    v1_x = Tws(0,3) - last_X;
    v1_y = Tws(1,3) - last_Y;
    v1_z = Tws(2,3) - last_Z;
    v2_x = X_diff;
    v2_y = Y_diff;
    v2_z = Z_diff;

    tracking_v1_x = (pKF->tracking_x) - tracking_last_X;
    tracking_v1_y = (pKF->tracking_y) - tracking_last_Y;
    tracking_v1_z = (pKF->tracking_z) - tracking_last_Z;
    tracking_v2_x = tracking_X_diff;
    tracking_v2_y = tracking_Y_diff;
    tracking_v2_z = tracking_Z_diff;

    if (KF_check_count > 10) {
        //if (violate_flag == 1) {
            //kfptr previous_KF = (*(itr - 2));
            //kfptr target_KF = (*(itr - 1));
            //(target_KF -> Twc_copy) = ((previous_KF -> Twc_copy) + (pKF -> Twc_copy)) / 2;
            //violate_flag = 0;
            //erase_counter ++;
            // for debug
            //cout << "Twc is overwritten." << endl;
        //}
        //xy_radian = acos(((v1_x * v2_x) + (v1_y * v2_y)) / ((sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0)))*(sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0)))));
        //yz_radian = acos(((v1_y * v2_y) + (v1_z * v2_z)) / ((sqrt(pow(v1_y, 2.0) + pow(v1_z, 2.0)))*(sqrt(pow(v2_y, 2.0) + pow(v2_z, 2.0)))));
        //zx_radian = acos(((v1_z * v2_z) + (v1_x * v2_x)) / ((sqrt(pow(v1_z, 2.0) + pow(v1_x, 2.0)))*(sqrt(pow(v2_z, 2.0) + pow(v2_x, 2.0)))));
        //xy_degree = xy_radian * (180/3.14);
        //yz_degree = yz_radian * (180/3.14);
        //zx_degree = zx_radian * (180/3.14);

        radian = acos(((v1_x * v2_x) + (v1_y * v2_y) + (v1_z * v2_z)) / ((sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0) + pow(v1_z, 2.0)))*(sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0) + pow(v2_z, 2.0)))));
        degree = radian * (180/3.14);

        tracking_radian = acos(((tracking_v1_x * tracking_v2_x) + (tracking_v1_y * tracking_v2_y) + (tracking_v1_z * tracking_v2_z)) / ((sqrt(pow(tracking_v1_x, 2.0) + pow(tracking_v1_y, 2.0) + pow(tracking_v1_z, 2.0)))*(sqrt(pow(tracking_v2_x, 2.0) + pow(tracking_v2_y, 2.0) + pow(tracking_v2_z, 2.0)))));
        tracking_degree = tracking_radian * (180/3.14);

        // for debug
        //cout << "degree x, y, z (KF write) : " << xy_degree << ", " << yz_degree << ", " << zx_degree << endl;
        //cout << "degree x, y, z (KF) : " << pKF->xy_deg << ", " << pKF->yz_deg << ", " << pKF->zx_deg << endl;
        //cout << "abs degree_diff x, y, z : " << abs(xy_degree - (pKF->xy_deg)) << ", " << abs(yz_degree - (pKF->yz_deg)) << ", " << abs(zx_degree - (pKF->zx_deg)) << endl;
        //cout << endl;

        //cout << "degree (KF write) : " << degree << endl;
        //cout << "degree (KF) : " << pKF->deg << endl;
        //cout << "degree (KF) : " << tracking_degree << endl;
        //cout << "abs degree_diff : " << abs(degree - (pKF->deg)) << endl;

        //if ((abs(xy_degree - (pKF->xy_deg))) > 10 || (abs(yz_degree - (pKF->yz_deg))) > 10 || (abs(zx_degree - (pKF->zx_deg))) > 10) {
        //if ((abs(degree - (pKF->deg))) > 30) {
        if (abs(degree - tracking_degree) > 20 * margin) {
            //violate_flag = 1;
            //cout << "DEBUG : motion violate KF is found." << endl;
            //cout << "DEBUG : KF_check_count = " << KF_check_count << endl;
            /*
            cout << "DEBUG : v1_x = " << v1_x << endl;
            cout << "DEBUG : v1_y = " << v1_y << endl;
            cout << "DEBUG : v1_z = " << v1_z << endl;
            cout << "DEBUG : v2_x = " << v2_x << endl;
            cout << "DEBUG : v2_y = " << v2_y << endl;
            cout << "DEBUG : v2_z = " << v2_z << endl;
            cout << "DEBUG : tracking_v1_x = " << tracking_v1_x << endl;
            cout << "DEBUG : tracking_v1_y = " << tracking_v1_y << endl;
            cout << "DEBUG : tracking_v1_z = " << tracking_v1_z << endl;
            cout << "DEBUG : tracking_v2_x = " << tracking_v2_x << endl;
            cout << "DEBUG : tracking_v2_y = " << tracking_v2_y << endl;
            cout << "DEBUG : tracking_v2_z = " << tracking_v2_z << endl;
            cout << "DEBUG : degree = " << degree << endl;
            cout << "DEBUG : tracking_degree = " << tracking_degree << endl;
            cout << endl;
            */
            violate_KF = (*itr);
            for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
                kfptr pKF = (*itr);
                if (pKF == violate_KF) {
                    foundKFs.erase(itr);
                    //pKF -> violate_flag = 1;
                    violate_flag = 1;
                    //cout << "DEBUG : KF is Deleted from foundKFs." << endl;
                    erase_counter ++;
                    update_flag = 0;
                    margin = margin + 0.2;
                    break;
                }
            }
        }
        else {
            update_flag = 1;
            if (margin > 1) {
                margin = 1;
            }
        }
    }

    if (update_flag == 1) {
        X_diff = Tws(0,3) - last_X;
        Y_diff = Tws(1,3) - last_Y;
        Z_diff = Tws(2,3) - last_Z;
        last_X = Tws(0,3);
        last_Y = Tws(1,3);
        last_Z = Tws(2,3);

        tracking_X_diff = (pKF->tracking_x) - tracking_last_X;
        tracking_Y_diff = (pKF->tracking_y) - tracking_last_Y;
        tracking_Z_diff = (pKF->tracking_z) - tracking_last_Z;
        tracking_last_X = pKF->tracking_x;
        tracking_last_Y = pKF->tracking_y;
        tracking_last_Z = pKF->tracking_z; 
    }

    //cout << "DEBUG : margin = " << margin << endl;

    KF_check_count ++;
    
    if (violate_flag == 0) {
        itr++;
    }
    else {
        violate_flag = 0;
    }
    //itr++;
  }

  // for debug
  cout << "foundKFs' size (after) : " << foundKFs.size() << endl;
  cout << "number of trimmed KFs (V2) : " << erase_counter << endl;
  erase_counter = 0;

  // write state to csv
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
    kfptr pKF = (*itr);
    const double stamp = pKF->mTimeStamp;
    Eigen::Vector3d bA = Eigen::Vector3d::Zero();
    Eigen::Vector3d bG = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    const cv::Mat T_wc = pKF->GetPoseInverse();
    // add this code (2023/11/16) ////////////////////////////////////////////////////////////////////////////
    //const cv::Mat T_wc = pKF->Twc_copy;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
    const Eigen::Quaterniond q(Tws.block<3,3>(0,0));

    // add this code (2023/11/16) ////////////////////////////////////////////////////////////////////////////
    //if (pKF -> violate_flag == 1 && KF_check_count > 5) {
        //Tws(0,3) = last_X + (pKF->v1_x);
        //Tws(1,3) = last_Y + (pKF->v1_y);
        //Tws(2,3) = last_Z + (pKF->v1_z);

        //for debug
        //cout << "Tws is overwritten by using tracking motion." << endl;
    //}
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    if(params::stats::miTrajectoryFormat == 0) {
        keyframesFile << std::setprecision(25) << stamp * 1e9f << ",";
        keyframesFile << Tws(0,3) << "," << Tws(1,3) << "," << Tws(2,3) << ",";
        keyframesFile << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ",";
        keyframesFile << vel[0] << "," << vel[1] << "," << vel[2] << ",";
        keyframesFile << bG[0] << "," << bG[1] << "," << bG[2] << ",";
        keyframesFile << bA[0] << "," << bA[1] << "," << bA[2] << std::endl;
    } else if(params::stats::miTrajectoryFormat == 1) {
        keyframesFile << std::setprecision(25) << stamp << " ";
        keyframesFile << Tws(0,3) << " " << Tws(1,3) << " " << Tws(2,3) << " ";
        keyframesFile << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    } else {
        std::cout << COUTFATAL << "miTrajectoryFormat '" << params::stats::miTrajectoryFormat << "' not in { 0=EUROC | 1=TUM }" << std::endl;
        exit(-1);
    }
    // add this code (2023/11/16) ///////////////////////////////////////////////////////////////////////////////////////
    //last_X = Tws(0,3);
    //last_Y = Tws(1,3);
    //last_Z = Tws(2,3);
    //KF_check_count++;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  }
  keyframesFile.close();

  KF_check_count = 0;
  violate_flag = 0;
  update_flag = 1;

  X_diff = 0;
  Y_diff = 0;
  Z_diff = 0;

  last_X = 0;
  last_Y = 0;
  last_Z = 0;

  std::cout << "KFs written to file" << std::endl;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Map::WriteStateToCsvTrimmedV3(const std::string& filename,
                          const size_t clientId) {
  std::vector<kfptr> foundKFs;
  foundKFs.reserve(mmpKeyFrames.size());
  // Get all frames from the required client
  for (std::map<idpair,kfptr>::const_iterator itr = mmpKeyFrames.begin();
       itr != mmpKeyFrames.end(); ++itr) {
    kfptr pKF = itr->second;
    idpair currID = itr->first;
    if (currID.second == clientId) {
      foundKFs.push_back(pKF);
    }
  }

  if(foundKFs.empty()) //would overwrite files from other maps with empty files
      return;

  // Sort the keyframes by timestamp
  std::sort(foundKFs.begin(), foundKFs.end(), KeyFrame::compKFstamp);

  // for debug
  cout << "foundKFs' size (before) : " << foundKFs.size() << endl;

  // Write out the keyframe data
  std::ofstream keyframesFile;
  keyframesFile.open(filename);

  // Check if KF violate motion
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ) {
    kfptr pKF = (*itr);
    // add this code (2023/11/16) /////////////////////////////////////////////////////////////////////////////
    //(pKF -> Twc_copy) = (pKF -> GetPoseInverse());
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    const cv::Mat T_wc = pKF->GetPoseInverse();
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
    const Eigen::Quaterniond q(Tws.block<3,3>(0,0));

    v1_x = Tws(0,3) - last_X;
    v1_y = Tws(1,3) - last_Y;
    v1_z = Tws(2,3) - last_Z;
    v2_x = X_diff;
    v2_y = Y_diff;
    v2_z = Z_diff;

    tracking_v1_x = (pKF->tracking_x) - tracking_last_X;
    tracking_v1_y = (pKF->tracking_y) - tracking_last_Y;
    tracking_v1_z = (pKF->tracking_z) - tracking_last_Z;
    tracking_v2_x = tracking_X_diff;
    tracking_v2_y = tracking_Y_diff;
    tracking_v2_z = tracking_Z_diff;

    if (KF_check_count > 10) {
        radian = acos(((v1_x * v2_x) + (v1_y * v2_y) + (v1_z * v2_z)) / ((sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0) + pow(v1_z, 2.0)))*(sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0) + pow(v2_z, 2.0)))));
        degree = radian * (180/3.14);
        tracking_radian = acos(((tracking_v1_x * tracking_v2_x) + (tracking_v1_y * tracking_v2_y) + (tracking_v1_z * tracking_v2_z)) / ((sqrt(pow(tracking_v1_x, 2.0) + pow(tracking_v1_y, 2.0) + pow(tracking_v1_z, 2.0)))*(sqrt(pow(tracking_v2_x, 2.0) + pow(tracking_v2_y, 2.0) + pow(tracking_v2_z, 2.0)))));
        tracking_degree = tracking_radian * (180/3.14);

        movement_ratio = sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0) + pow(v1_z, 2.0)) / sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0) + pow(v2_z, 2.0));
        tracking_movement_ratio = sqrt(pow(tracking_v1_x, 2.0) + pow(tracking_v1_y, 2.0) + pow(tracking_v1_z, 2.0)) / sqrt(pow(tracking_v2_x, 2.0) + pow(tracking_v2_y, 2.0) + pow(tracking_v2_z, 2.0));

        if (abs(degree - tracking_degree) > 20 * degree_margin) {
            violate_KF = (*itr);
            //cout << "DEBUG : KF is deleted from foundKF by DEGREE" << endl;
            //cout << "DEBUG : KF_check_count = " << KF_check_count << endl;
            
            for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
                kfptr pKF = (*itr);
                if (pKF == violate_KF) {
                    foundKFs.erase(itr);
                   //pKF -> violate_flag = 1;
                    violate_flag = 1;
                    //cout << "DEBUG : KF is Deleted from foundKFs." << endl;
                    erase_counter_degree ++;
                    update_flag = 0;
                    degree_margin = degree_margin + 0.1;
                    movement_margin = movement_margin + 0.2;
                    //cout << "DEBUG : decided as Violate Motion KF by DEGREE" << endl;
                    break;
                }
            }
        }
        else if ((movement_ratio / tracking_movement_ratio) > (2.0 * movement_margin) || (tracking_movement_ratio / movement_ratio) > (2.0 * movement_margin)) {
            violate_KF = (*itr);
            //cout << "DEBUG : KF is deleted from foundKF by MOVEMENT" << endl;
            //cout << "DEBUG : KF_check_count = " << KF_check_count << endl;

            for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
                kfptr pKF = (*itr);
                if (pKF == violate_KF) {
                    foundKFs.erase(itr);
                    //pKF -> violate_flag = 1;
                    violate_flag = 1;
                    //cout << "DEBUG : KF is Deleted from foundKFs." << endl;
                    erase_counter_movement ++;
                    update_flag = 0;
                    degree_margin = degree_margin + 0.1;
                    movement_margin = movement_margin + 0.2;
                    //cout << "DEBUG : decided as Violate Motion KF by MOTION RATIO" << endl;
                    break;
                }
            }
        }
        else {
            update_flag = 1;
            degree_margin = 1;
            movement_margin = 1;
        }
        
    }

    if (update_flag == 1) {
        X_diff = Tws(0,3) - last_X;
        Y_diff = Tws(1,3) - last_Y;
        Z_diff = Tws(2,3) - last_Z;
        last_X = Tws(0,3);
        last_Y = Tws(1,3);
        last_Z = Tws(2,3);

        tracking_X_diff = (pKF->tracking_x) - tracking_last_X;
        tracking_Y_diff = (pKF->tracking_y) - tracking_last_Y;
        tracking_Z_diff = (pKF->tracking_z) - tracking_last_Z;
        tracking_last_X = pKF->tracking_x;
        tracking_last_Y = pKF->tracking_y;
        tracking_last_Z = pKF->tracking_z; 
    }
    /*
    else {
            cout << "DEBUG (V3) : v1_x = " << v1_x << endl;
            cout << "DEBUG (V3) : v1_y = " << v1_y << endl;
            cout << "DEBUG (V3) : v1_z = " << v1_z << endl;
            cout << "DEBUG (V3) : v2_x = " << v2_x << endl;
            cout << "DEBUG (V3) : v2_y = " << v2_y << endl;
            cout << "DEBUG (V3) : v2_z = " << v2_z << endl;
            cout << "DEBUG (V3) : tracking_v1_x = " << tracking_v1_x << endl;
            cout << "DEBUG (V3) : tracking_v1_y = " << tracking_v1_y << endl;
            cout << "DEBUG (V3) : tracking_v1_z = " << tracking_v1_z << endl;
            cout << "DEBUG (V3) : tracking_v2_x = " << tracking_v2_x << endl;
            cout << "DEBUG (V3) : tracking_v2_y = " << tracking_v2_y << endl;
            cout << "DEBUG (V3) : tracking_v2_z = " << tracking_v2_z << endl;
            cout << "DEBUG (V3) : degree = " << degree << endl;
            cout << "DEBUG (V3) : tracking_degree = " << tracking_degree << endl;
            cout << "DEBUG (V3) : movement_ratio = " << movement_ratio << endl;
            cout << "DEBUG (V3) : tracking_movement_ratio = " << tracking_movement_ratio << endl;
            cout << "DEBUG (V3) : movement_ratio / tracking_movement_ratio = " << movement_ratio / tracking_movement_ratio << endl;
            cout << "DEBUG (V3) : tracking_movement_ratio / movement_ratio = " << tracking_movement_ratio / movement_ratio << endl;
            cout << endl;
    }
    */

    //cout << "DEBUG : margin = " << margin << endl;

    KF_check_count ++;
    
    if (violate_flag == 0) {
        itr++;
    }
    else {
        violate_flag = 0;
    }
    //itr++;
  }

  // for debug
  cout << "foundKFs' size (after) : " << foundKFs.size() << endl;
  cout << "number of trimmed KFs (V3, by degree) : " << erase_counter_degree << endl;
  cout << "number of trimmed KFs (V3, by movement) : " << erase_counter_movement << endl;
  erase_counter_degree = 0;
  erase_counter_movement = 0;

  // write state to csv
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
    kfptr pKF = (*itr);
    const double stamp = pKF->mTimeStamp;
    Eigen::Vector3d bA = Eigen::Vector3d::Zero();
    Eigen::Vector3d bG = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    const cv::Mat T_wc = pKF->GetPoseInverse();
    // add this code (2023/11/16) ////////////////////////////////////////////////////////////////////////////
    //const cv::Mat T_wc = pKF->Twc_copy;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
    const Eigen::Quaterniond q(Tws.block<3,3>(0,0));

    // add this code (2023/11/16) ////////////////////////////////////////////////////////////////////////////
    //if (pKF -> violate_flag == 1 && KF_check_count > 5) {
        //Tws(0,3) = last_X + (pKF->v1_x);
        //Tws(1,3) = last_Y + (pKF->v1_y);
        //Tws(2,3) = last_Z + (pKF->v1_z);

        //for debug
        //cout << "Tws is overwritten by using tracking motion." << endl;
    //}
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    if(params::stats::miTrajectoryFormat == 0) {
        keyframesFile << std::setprecision(25) << stamp * 1e9f << ",";
        keyframesFile << Tws(0,3) << "," << Tws(1,3) << "," << Tws(2,3) << ",";
        keyframesFile << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ",";
        keyframesFile << vel[0] << "," << vel[1] << "," << vel[2] << ",";
        keyframesFile << bG[0] << "," << bG[1] << "," << bG[2] << ",";
        keyframesFile << bA[0] << "," << bA[1] << "," << bA[2] << std::endl;
    } else if(params::stats::miTrajectoryFormat == 1) {
        keyframesFile << std::setprecision(25) << stamp << " ";
        keyframesFile << Tws(0,3) << " " << Tws(1,3) << " " << Tws(2,3) << " ";
        keyframesFile << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    } else {
        std::cout << COUTFATAL << "miTrajectoryFormat '" << params::stats::miTrajectoryFormat << "' not in { 0=EUROC | 1=TUM }" << std::endl;
        exit(-1);
    }
    // add this code (2023/11/16) ///////////////////////////////////////////////////////////////////////////////////////
    //last_X = Tws(0,3);
    //last_Y = Tws(1,3);
    //last_Z = Tws(2,3);
    //KF_check_count++;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  }
  keyframesFile.close();

  KF_check_count = 0;

  X_diff = 0;
  Y_diff = 0;
  Z_diff = 0;

  degree_margin = 1;
  movement_margin = 1;
  update_flag = 1;
  violate_flag = 0;

  std::cout << "KFs written to file" << std::endl;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Map::WriteStateToCsvTrimmedV4(const std::string& filename,
                          const size_t clientId) {
  std::vector<kfptr> foundKFs;
  foundKFs.reserve(mmpKeyFrames.size());
  // Get all frames from the required client
  for (std::map<idpair,kfptr>::const_iterator itr = mmpKeyFrames.begin();
       itr != mmpKeyFrames.end(); ++itr) {
    kfptr pKF = itr->second;
    idpair currID = itr->first;
    if (currID.second == clientId) {
      foundKFs.push_back(pKF);
    }
  }

  if(foundKFs.empty()) //would overwrite files from other maps with empty files
      return;

  // Sort the keyframes by timestamp
  std::sort(foundKFs.begin(), foundKFs.end(), KeyFrame::compKFstamp);

  // for debug
  cout << "foundKFs' size (before) : " << foundKFs.size() << endl;

  // Write out the keyframe data
  std::ofstream keyframesFile;
  keyframesFile.open(filename);

  // Check if KF violate motion
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ) {
    kfptr pKF = (*itr);
    // add this code (2023/11/16) /////////////////////////////////////////////////////////////////////////////
    //(pKF -> Twc_copy) = (pKF -> GetPoseInverse());
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    const cv::Mat T_wc = pKF->GetPoseInverse();
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
    const Eigen::Quaterniond q(Tws.block<3,3>(0,0));

    v1_x = Tws(0,3) - last_X;
    v1_y = Tws(1,3) - last_Y;
    v1_z = Tws(2,3) - last_Z;
    v2_x = X_diff;
    v2_y = Y_diff;
    v2_z = Z_diff;

    tracking_v1_x = (pKF->tracking_x) - tracking_last_X;
    tracking_v1_y = (pKF->tracking_y) - tracking_last_Y;
    tracking_v1_z = (pKF->tracking_z) - tracking_last_Z;
    tracking_v2_x = tracking_X_diff;
    tracking_v2_y = tracking_Y_diff;
    tracking_v2_z = tracking_Z_diff;

    if (KF_check_count > 10) {
        xy_radian = acos(((v1_x * v2_x) + (v1_y * v2_y)) / ((sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0)))*(sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0)))));
        yz_radian = acos(((v1_y * v2_y) + (v1_z * v2_z)) / ((sqrt(pow(v1_y, 2.0) + pow(v1_z, 2.0)))*(sqrt(pow(v2_y, 2.0) + pow(v2_z, 2.0)))));
        zx_radian = acos(((v1_z * v2_z) + (v1_x * v2_x)) / ((sqrt(pow(v1_z, 2.0) + pow(v1_x, 2.0)))*(sqrt(pow(v2_z, 2.0) + pow(v2_x, 2.0)))));
        xy_degree = xy_radian * (180/3.14);
        yz_degree = yz_radian * (180/3.14);
        zx_degree = zx_radian * (180/3.14);
        tracking_xy_radian = acos(((tracking_v1_x * tracking_v2_x) + (tracking_v1_y * tracking_v2_y)) / ((sqrt(pow(tracking_v1_x, 2.0) + pow(tracking_v1_y, 2.0)))*(sqrt(pow(tracking_v2_x, 2.0) + pow(tracking_v2_y, 2.0)))));
        tracking_yz_radian = acos(((tracking_v1_y * tracking_v2_y) + (tracking_v1_z * tracking_v2_z)) / ((sqrt(pow(tracking_v1_y, 2.0) + pow(tracking_v1_z, 2.0)))*(sqrt(pow(tracking_v2_y, 2.0) + pow(tracking_v2_z, 2.0)))));
        tracking_zx_radian = acos(((tracking_v1_z * tracking_v2_z) + (tracking_v1_x * tracking_v2_x)) / ((sqrt(pow(tracking_v1_z, 2.0) + pow(tracking_v1_x, 2.0)))*(sqrt(pow(tracking_v2_z, 2.0) + pow(tracking_v2_x, 2.0)))));
        tracking_xy_degree = tracking_xy_radian * (180/3.14);
        tracking_yz_degree = tracking_yz_radian * (180/3.14);
        tracking_zx_degree = tracking_zx_radian * (180/3.14);

        radian = acos(((v1_x * v2_x) + (v1_y * v2_y) + (v1_z * v2_z)) / ((sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0) + pow(v1_z, 2.0)))*(sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0) + pow(v2_z, 2.0)))));
        degree = radian * (180/3.14);
        tracking_radian = acos(((tracking_v1_x * tracking_v2_x) + (tracking_v1_y * tracking_v2_y) + (tracking_v1_z * tracking_v2_z)) / ((sqrt(pow(tracking_v1_x, 2.0) + pow(tracking_v1_y, 2.0) + pow(tracking_v1_z, 2.0)))*(sqrt(pow(tracking_v2_x, 2.0) + pow(tracking_v2_y, 2.0) + pow(tracking_v2_z, 2.0)))));
        tracking_degree = tracking_radian * (180/3.14);

        movement_ratio = sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0) + pow(v1_z, 2.0)) / sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0) + pow(v2_z, 2.0));
        tracking_movement_ratio = sqrt(pow(tracking_v1_x, 2.0) + pow(tracking_v1_y, 2.0) + pow(tracking_v1_z, 2.0)) / sqrt(pow(tracking_v2_x, 2.0) + pow(tracking_v2_y, 2.0) + pow(tracking_v2_z, 2.0));

        if (abs(degree - tracking_degree) > 20 * degree_margin) {
            violate_KF = (*itr);
            //cout << "DEBUG : KF is deleted from foundKF by DEGREE" << endl;
            //cout << "DEBUG : KF_check_count = " << KF_check_count << endl;
           
            for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
                kfptr pKF = (*itr);
                if (pKF == violate_KF) {
                    foundKFs.erase(itr);
                   //pKF -> violate_flag = 1;
                    violate_flag = 1;
                    //cout << "DEBUG : KF is Deleted from foundKFs." << endl;
                    erase_counter_degree ++;
                    update_flag = 0;
                    degree_margin = degree_margin + 0.1;
                    movement_margin = movement_margin + 0.4;
                    //cout << "DEBUG : decided as Violate Motion KF by DEGREE" << endl;
                    break;
                }
            }
        }
        else if ((movement_ratio / tracking_movement_ratio) > (2.0 * movement_margin) || (tracking_movement_ratio / movement_ratio) > (2.0 * movement_margin)) {
            violate_KF = (*itr);
            //cout << "DEBUG : KF is deleted from foundKF by MOVEMENT" << endl;
            //cout << "DEBUG : KF_check_count = " << KF_check_count << endl;
            
            for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
                kfptr pKF = (*itr);
                if (pKF == violate_KF) {
                    foundKFs.erase(itr);
                    //pKF -> violate_flag = 1;
                    violate_flag = 1;
                    //cout << "DEBUG : KF is Deleted from foundKFs." << endl;
                    erase_counter_movement ++;
                    update_flag = 0;
                    degree_margin = degree_margin + 0.1;
                    movement_margin = movement_margin + 0.4;
                    //cout << "DEBUG : decided as Violate Motion KF by MOTION RATIO" << endl;
                    break;
                }
            }
        }
        else if (abs(xy_degree - tracking_xy_degree) > 50 * degree_margin || abs(yz_degree - tracking_yz_degree) > 50 * degree_margin || abs(zx_degree - tracking_zx_degree) > 50 * degree_margin) {
            violate_KF = (*itr);
            //cout << "DEBUG : KF is deleted from foundKF by xy,yz,zx DEGREE" << endl;
            //cout << "DEBUG : KF_check_count = " << KF_check_count << endl;
           
            for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
                kfptr pKF = (*itr);
                if (pKF == violate_KF) {
                    foundKFs.erase(itr);
                   //pKF -> violate_flag = 1;
                    violate_flag = 1;
                    //cout << "DEBUG : KF is Deleted from foundKFs." << endl;
                    erase_counter_plain_degree ++;
                    update_flag = 0;
                    degree_margin = degree_margin + 0.1;
                    movement_margin = movement_margin + 0.4;
                    //cout << "DEBUG : decided as Violate Motion KF by DEGREE" << endl;
                    break;
                }
            }
        }
        else {
            update_flag = 1;
            degree_margin = 1;
            movement_margin = 1;
        }
        
    }

    if (update_flag == 1) {
        X_diff = Tws(0,3) - last_X;
        Y_diff = Tws(1,3) - last_Y;
        Z_diff = Tws(2,3) - last_Z;
        last_X = Tws(0,3);
        last_Y = Tws(1,3);
        last_Z = Tws(2,3);

        tracking_X_diff = (pKF->tracking_x) - tracking_last_X;
        tracking_Y_diff = (pKF->tracking_y) - tracking_last_Y;
        tracking_Z_diff = (pKF->tracking_z) - tracking_last_Z;
        tracking_last_X = pKF->tracking_x;
        tracking_last_Y = pKF->tracking_y;
        tracking_last_Z = pKF->tracking_z; 
    }
    /*
    else {
            cout << "DEBUG (V4) : v1_x = " << v1_x << endl;
            cout << "DEBUG (V4) : v1_y = " << v1_y << endl;
            cout << "DEBUG (V4) : v1_z = " << v1_z << endl;
            cout << "DEBUG (V4) : v2_x = " << v2_x << endl;
            cout << "DEBUG (V4) : v2_y = " << v2_y << endl;
            cout << "DEBUG (V4) : v2_z = " << v2_z << endl;
            cout << "DEBUG (V4) : tracking_v1_x = " << tracking_v1_x << endl;
            cout << "DEBUG (V4) : tracking_v1_y = " << tracking_v1_y << endl;
            cout << "DEBUG (V4) : tracking_v1_z = " << tracking_v1_z << endl;
            cout << "DEBUG (V4) : tracking_v2_x = " << tracking_v2_x << endl;
            cout << "DEBUG (V4) : tracking_v2_y = " << tracking_v2_y << endl;
            cout << "DEBUG (V4) : tracking_v2_z = " << tracking_v2_z << endl;
            cout << "DEBUG (V4) : degree = " << degree << endl;
            cout << "DEBUG (V4) : tracking_degree = " << tracking_degree << endl;
            cout << "DEBUG (V4) : movement_ratio = " << movement_ratio << endl;
            cout << "DEBUG (V4) : tracking_movement_ratio = " << tracking_movement_ratio << endl;
            cout << "DEBUG (V4) : movement_ratio / tracking_movement_ratio = " << movement_ratio / tracking_movement_ratio << endl;
            cout << "DEBUG (V4) : tracking_movement_ratio / movement_ratio = " << tracking_movement_ratio / movement_ratio << endl;
            cout << endl;
    }
    */

    //cout << "DEBUG : margin = " << margin << endl;

    if (degree_margin > 3.0) {
        degree_margin = 3.0;
    }

    if (movement_margin > 2.5) {
        movement_margin = 2.5;
    }

    KF_check_count ++;
    
    if (violate_flag == 0) {
        itr++;
    }
    else {
        violate_flag = 0;
    }
    //itr++;
  }

  // for debug
  cout << "foundKFs' size (after) : " << foundKFs.size() << endl;
  cout << "number of trimmed KFs (V4, by degree) : " << erase_counter_degree << endl;
  cout << "number of trimmed KFs (V4, by xy,yz,zx degree) : " << erase_counter_plain_degree << endl;
  cout << "number of trimmed KFs (V4, by movement) : " << erase_counter_movement << endl;
  erase_counter_degree = 0;
  erase_counter_movement = 0;

  // write state to csv
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
    kfptr pKF = (*itr);
    const double stamp = pKF->mTimeStamp;
    Eigen::Vector3d bA = Eigen::Vector3d::Zero();
    Eigen::Vector3d bG = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    const cv::Mat T_wc = pKF->GetPoseInverse();
    // add this code (2023/11/16) ////////////////////////////////////////////////////////////////////////////
    //const cv::Mat T_wc = pKF->Twc_copy;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
    const Eigen::Quaterniond q(Tws.block<3,3>(0,0));

    // add this code (2023/11/16) ////////////////////////////////////////////////////////////////////////////
    //if (pKF -> violate_flag == 1 && KF_check_count > 5) {
        //Tws(0,3) = last_X + (pKF->v1_x);
        //Tws(1,3) = last_Y + (pKF->v1_y);
        //Tws(2,3) = last_Z + (pKF->v1_z);

        //for debug
        //cout << "Tws is overwritten by using tracking motion." << endl;
    //}
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    if(params::stats::miTrajectoryFormat == 0) {
        keyframesFile << std::setprecision(25) << stamp * 1e9f << ",";
        keyframesFile << Tws(0,3) << "," << Tws(1,3) << "," << Tws(2,3) << ",";
        keyframesFile << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ",";
        keyframesFile << vel[0] << "," << vel[1] << "," << vel[2] << ",";
        keyframesFile << bG[0] << "," << bG[1] << "," << bG[2] << ",";
        keyframesFile << bA[0] << "," << bA[1] << "," << bA[2] << std::endl;
    } else if(params::stats::miTrajectoryFormat == 1) {
        keyframesFile << std::setprecision(25) << stamp << " ";
        keyframesFile << Tws(0,3) << " " << Tws(1,3) << " " << Tws(2,3) << " ";
        keyframesFile << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    } else {
        std::cout << COUTFATAL << "miTrajectoryFormat '" << params::stats::miTrajectoryFormat << "' not in { 0=EUROC | 1=TUM }" << std::endl;
        exit(-1);
    }
    // add this code (2023/11/16) ///////////////////////////////////////////////////////////////////////////////////////
    //last_X = Tws(0,3);
    //last_Y = Tws(1,3);
    //last_Z = Tws(2,3);
    //KF_check_count++;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  }
  keyframesFile.close();

  KF_check_count = 0;

  X_diff = 0;
  Y_diff = 0;
  Z_diff = 0;

  std::cout << "KFs written to file" << std::endl;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// add this code (2023/11/21) ////////////////////////////////////////////////////////////////////////////////////////
void Map::WriteStateToCsvTrimmed(const std::string& filename,
                          const size_t clientId) {
  std::vector<kfptr> foundKFs;
  foundKFs.reserve(mmpKeyFrames.size());
  // Get all frames from the required client
  for (std::map<idpair,kfptr>::const_iterator itr = mmpKeyFrames.begin();
       itr != mmpKeyFrames.end(); ++itr) {
    kfptr pKF = itr->second;
    idpair currID = itr->first;
    if (currID.second == clientId) {
      foundKFs.push_back(pKF);
    }
  }

  if(foundKFs.empty()) //would overwrite files from other maps with empty files
      return;

  // Sort the keyframes by timestamp
  std::sort(foundKFs.begin(), foundKFs.end(), KeyFrame::compKFstamp);

  // for debug
  cout << "foundKFs' size (before) : " << foundKFs.size() << endl;

  // Write out the keyframe data
  std::ofstream keyframesFile;
  keyframesFile.open(filename);

  // Check if KF violate motion
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ) {
    kfptr pKF = (*itr);
    // add this code (2023/11/16) /////////////////////////////////////////////////////////////////////////////
    //(pKF -> Twc_copy) = (pKF -> GetPoseInverse());
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    const cv::Mat T_wc = pKF->GetPoseInverse();
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
    const Eigen::Quaterniond q(Tws.block<3,3>(0,0));

    v1_x = Tws(0,3) - last_X;
    v1_y = Tws(1,3) - last_Y;
    v1_z = Tws(2,3) - last_Z;
    v2_x = X_diff;
    v2_y = Y_diff;
    v2_z = Z_diff;

    if (KF_check_count > 10) {
        //if (violate_flag == 1) {
            //kfptr previous_KF = (*(itr - 2));
            //kfptr target_KF = (*(itr - 1));
            //(target_KF -> Twc_copy) = ((previous_KF -> Twc_copy) + (pKF -> Twc_copy)) / 2;
            //violate_flag = 0;
            //erase_counter ++;
            // for debug
            //cout << "Twc is overwritten." << endl;
        //}
        //xy_radian = acos(((v1_x * v2_x) + (v1_y * v2_y)) / ((sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0)))*(sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0)))));
        //yz_radian = acos(((v1_y * v2_y) + (v1_z * v2_z)) / ((sqrt(pow(v1_y, 2.0) + pow(v1_z, 2.0)))*(sqrt(pow(v2_y, 2.0) + pow(v2_z, 2.0)))));
        //zx_radian = acos(((v1_z * v2_z) + (v1_x * v2_x)) / ((sqrt(pow(v1_z, 2.0) + pow(v1_x, 2.0)))*(sqrt(pow(v2_z, 2.0) + pow(v2_x, 2.0)))));
        //xy_degree = xy_radian * (180/3.14);
        //yz_degree = yz_radian * (180/3.14);
        //zx_degree = zx_radian * (180/3.14);

        radian = acos(((v1_x * v2_x) + (v1_y * v2_y) + (v1_z * v2_z)) / ((sqrt(pow(v1_x, 2.0) + pow(v1_y, 2.0) + pow(v1_z, 2.0)))*(sqrt(pow(v2_x, 2.0) + pow(v2_y, 2.0) + pow(v2_z, 2.0)))));
        degree = radian * (180/3.14);

        // for debug
        //cout << "degree x, y, z (KF write) : " << xy_degree << ", " << yz_degree << ", " << zx_degree << endl;
        //cout << "degree x, y, z (KF) : " << pKF->xy_deg << ", " << pKF->yz_deg << ", " << pKF->zx_deg << endl;
        //cout << "abs degree_diff x, y, z : " << abs(xy_degree - (pKF->xy_deg)) << ", " << abs(yz_degree - (pKF->yz_deg)) << ", " << abs(zx_degree - (pKF->zx_deg)) << endl;
        //cout << endl;

        //cout << "degree (KF write) : " << degree << endl;
        //cout << "degree (KF) : " << pKF->deg << endl;
        //cout << "degree (KF) : " << tracking_degree << endl;
        //cout << "abs degree_diff : " << abs(degree - (pKF->deg)) << endl;

        //if ((abs(xy_degree - (pKF->xy_deg))) > 10 || (abs(yz_degree - (pKF->yz_deg))) > 10 || (abs(zx_degree - (pKF->zx_deg))) > 10) {
        if ((abs(degree - (pKF->deg))) > 30) {
            //violate_flag = 1;
            //cout << "DEBUG : motion violate KF is found." << endl;
            violate_KF = (*itr);
            for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
                kfptr pKF = (*itr);
                if (pKF == violate_KF) {
                    foundKFs.erase(itr);
                    //pKF -> violate_flag = 1;
                    violate_flag = 1;
                    //cout << "DEBUG : KF is Deleted from foundKFs." << endl;
                    erase_counter ++;
                    break;
                }
            }
        }
    }

    X_diff = Tws(0,3) - last_X;
    Y_diff = Tws(1,3) - last_Y;
    Z_diff = Tws(2,3) - last_Z;
    last_X = Tws(0,3);
    last_Y = Tws(1,3);
    last_Z = Tws(2,3);

    KF_check_count ++;
    
    if (violate_flag == 0) {
        itr++;
    }
    else {
        violate_flag = 0;
    }
    //itr++;
  }

  // for debug
  cout << "foundKFs' size (after) : " << foundKFs.size() << endl;
  cout << "number of trimmed KFs : " << erase_counter << endl;
  erase_counter = 0;

  // write state to csv
  for (std::vector<kfptr>::const_iterator itr = foundKFs.begin(); itr != foundKFs.end(); ++itr) {
    kfptr pKF = (*itr);
    const double stamp = pKF->mTimeStamp;
    Eigen::Vector3d bA = Eigen::Vector3d::Zero();
    Eigen::Vector3d bG = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    const cv::Mat T_wc = pKF->GetPoseInverse();
    // add this code (2023/11/16) ////////////////////////////////////////////////////////////////////////////
    //const cv::Mat T_wc = pKF->Twc_copy;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    const Eigen::Matrix4d eT_wc = Converter::toMatrix4d(T_wc);
    ccptr pCC = *(mspCC.begin());
    Eigen::Matrix4d T_SC;
    if(mSysState == SERVER)
        T_SC = pKF->mT_SC;
    else
        T_SC = pCC->mT_SC;

    const Eigen::Matrix4d Tws = eT_wc * T_SC.inverse();
    const Eigen::Quaterniond q(Tws.block<3,3>(0,0));

    // add this code (2023/11/16) ////////////////////////////////////////////////////////////////////////////
    //if (pKF -> violate_flag == 1 && KF_check_count > 5) {
        //Tws(0,3) = last_X + (pKF->v1_x);
        //Tws(1,3) = last_Y + (pKF->v1_y);
        //Tws(2,3) = last_Z + (pKF->v1_z);

        //for debug
        //cout << "Tws is overwritten by using tracking motion." << endl;
    //}
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    if(params::stats::miTrajectoryFormat == 0) {
        keyframesFile << std::setprecision(25) << stamp * 1e9f << ",";
        keyframesFile << Tws(0,3) << "," << Tws(1,3) << "," << Tws(2,3) << ",";
        keyframesFile << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << ",";
        keyframesFile << vel[0] << "," << vel[1] << "," << vel[2] << ",";
        keyframesFile << bG[0] << "," << bG[1] << "," << bG[2] << ",";
        keyframesFile << bA[0] << "," << bA[1] << "," << bA[2] << std::endl;
    } else if(params::stats::miTrajectoryFormat == 1) {
        keyframesFile << std::setprecision(25) << stamp << " ";
        keyframesFile << Tws(0,3) << " " << Tws(1,3) << " " << Tws(2,3) << " ";
        keyframesFile << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    } else {
        std::cout << COUTFATAL << "miTrajectoryFormat '" << params::stats::miTrajectoryFormat << "' not in { 0=EUROC | 1=TUM }" << std::endl;
        exit(-1);
    }
    // add this code (2023/11/16) ///////////////////////////////////////////////////////////////////////////////////////
    //last_X = Tws(0,3);
    //last_Y = Tws(1,3);
    //last_Z = Tws(2,3);
    //KF_check_count++;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  }
  keyframesFile.close();

  KF_check_count = 0;

  X_diff = 0;
  Y_diff = 0;
  Z_diff = 0;

  std::cout << "KFs written to file" << std::endl;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//add this code (2023/11/5) ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void Map::TrimMotionViolateKF(kfptr targetKF)
//{
//    unique_lock<mutex> lock(mMutexMap);
//    unique_lock<mutex> lock2(mMutexErased);

//    cout << "DEBUG : TrimMotionViolateKF is called." << endl;
//    for (std::map<idpair,kfptr>::const_iterator itr = mmpKeyFrames.begin();
//       itr != mmpKeyFrames.end(); ++itr) {
//        kfptr pKF = itr->second;
//        if (pKF == targetKF) {
//            mmpKeyFrames.erase(itr);
//            cout << "DEBUG : KF is trimmed by TrimMotionViolateKF." << endl;
//        }
//  }
//}

//vector<Map::kfptr> Map::DeleteMotionViolateKF(vector<Map::kfptr> foundKFs, Map::kfptr targetKF)
//{
//    cout << "DEBUG : DeleteMotionViolateKF is called." << endl;
//    for (kfptr itr = foundKFs.begin();
//       itr != foundKFs.end(); ++itr) {
//        kfptr pKF = itr->second;
//        if (pKF == targetKF) {
//            foundKFs.erase(itr);
//            cout << "DEBUG : KF is Deleted from foundKFs by DeleteMotionViolateKF." << endl;
//        }
//    }

//    return foundKFs;
//}

#ifdef DEBUGGING2
void Map::CheckStructure()
{
    std::cout << "+++++ Check Map Structure +++++" << std::endl;

    std::cout << "--- KFs ---" << std::endl;

    for(set<kfptr>::iterator sit = mspKeyFrames.begin();sit!=mspKeyFrames.end();++sit)
    {
        kfptr pKF = *sit;

        if(pKF->isBad())
            std::cout << COUTWARN << "KF " << pKF->GetId() << ": bad but in map" << std::endl;

        kfptr pKFp = pKF->GetParent();

        if(!pKFp)
        {
            if(pKF->mId.first == 0)
                continue; //ok
            else
            {
                std::cout << "KF " << pKF->GetId() << ": no parent" << std::endl;

                kfptr pNewParent;
                size_t newid = pKF->mId.first-1;

                pNewParent = this->GetKfPtr(newid,pKF->mId.second);

                while(!pNewParent)
                {
                    newid += -1;
                    pNewParent = this->GetKfPtr(newid,pKF->mId.second);
                }

                if(!pNewParent)
                {
                    std::cout << COUTFATAL << "KF " << pKF->GetId() << ": Cannot find new parent - quitting" << std::endl;
                    throw estd::infrastructure_ex();
                }

                pKF->ChangeParent(pNewParent);
            }
        }
        else
        {
            if(pKFp->mId == pKF->mId)
            {
                std::cout << COUTERROR << "KF " << pKF->GetId() << ": is its own parent" << std::endl;
                std::cout << "Same ptr? " << (int)(pKF == pKFp) << std::endl;

                kfptr pNewParent;
                size_t newid = pKF->mId.first-1;

                pNewParent = this->GetKfPtr(newid,pKF->mId.second);

                while(!pNewParent)
                {
                    newid += -1;
                    pNewParent = this->GetKfPtr(newid,pKF->mId.second);
                }

                if(!pNewParent)
                {
                    std::cout << COUTFATAL << "KF " << pKF->GetId() << ": Cannot find new parent - quitting" << std::endl;
                    throw estd::infrastructure_ex();
                }

                pKF->ChangeParent(pNewParent);
            }
            else
                continue; //ok
        }

    }

    std::cout << "--- MPs ---" << std::endl;

    for(set<mpptr>::iterator sit = mspMapPoints.begin();sit!=mspMapPoints.end();++sit)
    {
        mpptr pMP = *sit;

        if(pMP->isBad())
        {
            std::cout << COUTWARN << "MP " << pMP->GetId() << ": bad but in map" << std::endl;

            this->EraseMapPoint(pMP);
        }

        kfptr pKFp = pMP->GetReferenceKeyFrame();

        if(!pKFp)
        {
                std::cout << "MP " << pMP->GetId() << ": no parent" << std::endl;

                map<kfptr,size_t> observations = pMP->GetObservations();

                if(observations.size() < 2)
                {
                    std::cout << "MP " << pMP->GetId() << ": only " << observations.size() << " observation" << std::endl;

                    pMP->SetBadFlag();

                    continue;
                }

                kfptr pNewParent;

                for(map<kfptr,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
                {
                    pNewParent = mit->first;

                    if(pNewParent)
                        break;
                }

                if(!pNewParent)
                {
                    std::cout << COUTFATAL << "MP " << pMP->GetId() << ": Cannot find new parent - quitting" << std::endl;
                    throw estd::infrastructure_ex();
                }

                pMP->SetReferenceKeyFrame(pNewParent);
        }
    }

    std::cout << "--- DONE ---" << std::endl;
}
#endif

} //end ns
