#pragma once

//#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
//#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/flann/flann.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <iostream>
#include <fstream>
#include <vector>
#include "object6D.hpp"
#include "hash_int.hpp"
#include "utils.hpp"
#include "hash_murmur.hpp"

namespace ppf
{
typedef pcl::PointXYZ PointT1;
typedef pcl::PointNormal PointT2;
typedef pcl::PointCloud<PointT1> CloudPointT1;
typedef pcl::PointCloud<PointT2> CloudPointT2;
typedef pcl::PointCloud<PointT1>::ConstPtr CloudPointT1Ptr;
typedef pcl::PointCloud<PointT2>::ConstPtr CloudPointT2Ptr;

typedef struct THash
{
    int id;
    int i, ppfInd;
} THash;

class PPF6DDetector
{
public:
    PPF6DDetector();
    PPF6DDetector(const double distSampling, const double angleStep,
                  const double rotationThred, const double positionThred);

    void clearTrainingModels();
    void preProcessing(CloudPointT1Ptr pc, CloudPointT2 &pc_with_normals, bool training);
    void trainModel(CloudPointT2Ptr pc);
    void computePPFFeatures(const cv::Vec3d& p1, const cv::Vec3d& n1,
                            const cv::Vec3d& p2, const cv::Vec3d& n2,
                            cv::Vec4d& f);
    void match(CloudPointT2Ptr pc, std::vector<obj6D::Pose6DPtr>& results, const double relativeSceneSampleStep);
    bool matchPose(const obj6D::Pose6D& sourcePose, const obj6D::Pose6D& targetPose);
    void clusterPoses(std::vector<obj6D::Pose6DPtr>& poseList, std::vector<obj6D::Pose6DPtr> &finalPoses);
    virtual ~PPF6DDetector();
private:
    double dist_sampling, angle_step, distance_step;
    double rotation_threshold, position_threshold, dist_threshold;
    cv::Mat angle_table;
    hashtable_int* hash_table;
    THash* hash_nodes;
    cv::Mat pc_model;
};

}
