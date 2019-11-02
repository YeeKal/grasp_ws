#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "utils.hpp"

namespace obj6D
{
class Pose6D;
typedef cv::Ptr<Pose6D> Pose6DPtr;

class PoseCluster6D;
typedef cv::Ptr<PoseCluster6D> PoseCluster6DPtr;

class Pose6D
{
public:
    Pose6D()
    {
        pose = cv::Matx44d::all(0);
    }
    Pose6D(size_t NumVotes=0)
    {
        numVotes = NumVotes;
        pose = cv::Matx44d::all(0);
    }
    virtual ~Pose6D() {}

    void updatePose(cv::Matx44d& NewPose)
    {
        pose = NewPose;
        poseToRT(pose, pose_R, pose_t);
        dcmToQuat(pose_R, q);
    }
    void updatePoseQuat(cv::Vec4d& Q, cv::Vec3d& NewT)
    {
        cv::Matx33d NewR;
        quatToDCM(Q, NewR);
        q = Q;
        rtToPose(NewR, NewT, pose);
		pose_R = NewR;
		pose_t = NewT;
    }

    Pose6DPtr clone()
    {
        cv::Ptr<Pose6D> new_pose(new Pose6D(numVotes));
        new_pose->pose = this->pose;
        new_pose->pose_R = pose_R;
        new_pose->q = q;
        new_pose->pose_t = pose_t;
        return new_pose;
    }

    cv::Matx44d pose;
    cv::Matx33d pose_R;
    cv::Vec3d pose_t;
    cv::Vec4d q;
    size_t numVotes;
};

class PoseCluster6D
{
public:
    PoseCluster6D()
    {
        numVotes=0;
        id=0;
        poseList.clear();
    }
    PoseCluster6D(Pose6DPtr newPose)
    {
        poseList.clear();
        poseList.push_back(newPose);
        numVotes=newPose->numVotes;
        id=0;
    }

    virtual ~PoseCluster6D(){}

    void addPose(Pose6DPtr newPose)
    {
        poseList.push_back(newPose);
        numVotes += newPose->numVotes;
    }

    std::vector<Pose6DPtr> poseList;
    size_t numVotes;
    int id;
};
}
