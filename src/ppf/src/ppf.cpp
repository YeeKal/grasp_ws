#include "ppf.hpp"

namespace ppf
{
static bool pose6DPtrCompare(const obj6D::Pose6DPtr& a, const obj6D::Pose6DPtr& b)
{
    CV_Assert(!a.empty() && !b.empty());
    return ( a->numVotes > b->numVotes );
}
static int sortPoseClusters(const obj6D::PoseCluster6DPtr& a, const obj6D::PoseCluster6DPtr& b)
{
    CV_Assert(!a.empty() && !b.empty());
    return ( a->numVotes > b->numVotes );
}
static bool numberCompare(const float& a, const float& b)
{
    return ( a > b );
}
static KeyType hashPPF(const cv::Vec4d& f, const double AngleStep, const double DistanceStep, int &idx)
{
    cv::Vec4i key(
                (int)(f[0] / AngleStep),
            (int)(f[1] / AngleStep),
            (int)(f[2] / AngleStep),
            (int)(f[3] / DistanceStep));
    KeyType hashKey = 0;
    int angle_dim = (int)(360.0f/(AngleStep*180.0f/CV_PI));
    idx = key[0] + key[1] * angle_dim
            + key[2] * angle_dim * angle_dim
            + key[3] * angle_dim * angle_dim * angle_dim;

    murmurHash(key.val, 4*sizeof(int), 42, &hashKey);
    return hashKey;
}
static double computeAlpha(const cv::Vec3d& p1, const cv::Vec3d& n1, const cv::Vec3d& p2)
{
    cv::Vec3d Tmg, mpt;
    cv::Matx33d R;
    double alpha;

    computeTransformRT(p1, n1, R, Tmg);
    mpt = Tmg + R * p2;
    alpha=atan2(-mpt[2], mpt[1]);


    if ( alpha != alpha)
    {
        return 0;
    }

    if (sin(alpha)*mpt[2]<0.0)
        alpha=-alpha;

    return (-alpha);
}
bool PPF6DDetector::matchPose(const obj6D::Pose6D& sourcePose, const obj6D::Pose6D& targetPose)
{
    // translational difference
    cv::Vec3d dv = targetPose.pose_t - sourcePose.pose_t;
    double dNorm = cv::norm(dv);
    double phi = 0;
    cv::Matx33d R = sourcePose.pose_R*targetPose.pose_R.t();
    cv::Vec3d v1(sourcePose.pose_R(0,2),sourcePose.pose_R(1,2),sourcePose.pose_R(2,2));
    cv::Vec3d v2(targetPose.pose_R(0,2),targetPose.pose_R(1,2),targetPose.pose_R(2,2));
    double angle_z = TAngle3Normalized(v1,v2);
    const double trace = cv::trace(R);
    if (fabs(trace - 3) > EPS)
    {
        if (fabs(trace + 1) <= EPS)
        {
            phi = CV_PI;
        }
        else
        {
            phi = ( acos((trace - 1)/2) );
        }
    }
    if(phi<0)
    {
        phi = -phi;
    }
    //std::cout<<phi<<std::endl;
    return ((phi < rotation_threshold && dNorm < position_threshold)||(angle_z<rotation_threshold && dNorm < position_threshold));
}
PPF6DDetector::PPF6DDetector()
{
    dist_sampling = 0.05;
    angle_step = 30.0f;
    distance_step = 0.015;
    rotation_threshold = 360.0f/30.0f*(CV_PI/180.0f);
    position_threshold = 0.01;
    dist_threshold = 0.1;
}
PPF6DDetector::PPF6DDetector(const double distSampling, const double angleStep,
                             const double rotationThred, const double positionThred)
{
	std::cout << "hehhhh	" << std::endl;
    dist_sampling = distSampling;
    angle_step = angleStep;
    distance_step = 0.015;
    rotation_threshold = rotationThred*(CV_PI/180.0f);
    position_threshold = positionThred;
    dist_threshold = 0.1;
}
void PPF6DDetector::preProcessing(CloudPointT1Ptr pc, CloudPointT2 &pc_with_normals, bool training)
{
    if(training)
    {
        cv::Vec2f xRange, yRange, zRange;
        computeBboxStd(pc, xRange, yRange, zRange);
        std::vector<float> d(3);
        d[0] = xRange[1] - xRange[0];
        d[1] = yRange[1] - yRange[0];
        d[2] = zRange[1] - zRange[0];
        float diameter = sqrt ( d[0] * d[0] + d[1] * d[1] + d[2] * d[2] );
        std::sort(d.begin(),d.end(),numberCompare);
        float sampleDiameter = sqrt(d[1]*d[1] + d[2]*d[2]);
        distance_step = (float)(diameter * dist_sampling);
        dist_threshold = sampleDiameter;
        std::cout<<"distance_step: "<<distance_step<<std::endl;
    }

    pcl::PointCloud<PointT1>::Ptr pc_downsample (new pcl::PointCloud<PointT1> ());
    pcl::VoxelGrid<PointT1> sor;
    sor.setInputCloud(pc);
    sor.setLeafSize(distance_step, distance_step, distance_step);
    sor.filter(*pc_downsample);

    pcl::NormalEstimationOMP<PointT1, pcl::Normal> ne;
    ne.setInputCloud(pc_downsample);
    pcl::search::KdTree<PointT1>::Ptr tree (new pcl::search::KdTree<PointT1>);
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr pc_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.03);
    ne.compute(*pc_normals);
    pcl::concatenateFields(*pc_downsample, *pc_normals, pc_with_normals);
    if(training)
    {
#pragma omp parallel for
        for (int i=0; i<pc_with_normals.points.size(); i++)
        {
            pc_with_normals.points[i].normal_x = -pc_with_normals.points[i].normal_x;
            pc_with_normals.points[i].normal_y = -pc_with_normals.points[i].normal_y;
            pc_with_normals.points[i].normal_z = -pc_with_normals.points[i].normal_z;
        }
        std::cout<<"normal size: "<<pc_with_normals.points.size()<<std::endl;
    }
}
void PPF6DDetector::trainModel(CloudPointT2Ptr pc)
{
    int numRefPoints = pc->points.size();
    hashtable_int* hashTable = hashtableCreate(numRefPoints*numRefPoints, NULL);
    hash_nodes = (THash*)calloc(numRefPoints*numRefPoints, sizeof(THash));
    angle_table = cv::Mat(numRefPoints*numRefPoints, 1, CV_32FC1);
#pragma omp parallel for
    for (int i=0; i<numRefPoints; i++)
    {
        const cv::Vec3f p1(pc->points[i].x, pc->points[i].y, pc->points[i].z);
        const cv::Vec3f n1(pc->points[i].normal_x, pc->points[i].normal_y, pc->points[i].normal_z);
        for (int j=0; j<numRefPoints; j++)
        {
            if (i!=j)
            {
                const cv::Vec3f p2(pc->points[j].x, pc->points[j].y, pc->points[j].z);
                const cv::Vec3f n2(pc->points[j].normal_x, pc->points[j].normal_y, pc->points[j].normal_z);

                cv::Vec4d f = cv::Vec4d::all(0);
                computePPFFeatures(p1, n1, p2, n2, f);
                int idx;
                KeyType hashValue = hashPPF(f, angle_step*CV_PI/180.0f, distance_step, idx);
                double alpha = computeAlpha(p1, n1, p2);
                uint ppfInd = i*numRefPoints+j;
                THash* hashNode = &hash_nodes[i*numRefPoints+j];
                hashNode->id = hashValue;
                hashNode->i = i;
                hashNode->ppfInd = ppfInd;
                //#pragma omp critical
                hashtableInsertHashed(hashTable, hashValue, (void*)hashNode);
                angle_table.ptr<float>(ppfInd)[0] = (float)alpha;
            }
        }
    }
    pcl2cvMat(pc_model,*pc);
    hash_table = hashTable;
}
void PPF6DDetector::computePPFFeatures(const cv::Vec3d& p1, const cv::Vec3d& n1,
                                       const cv::Vec3d& p2, const cv::Vec3d& n2,
                                       cv::Vec4d& f)
{
    cv::Vec3d d(p2 - p1);
    f[3] = cv::norm(d);
    if (f[3] <= EPS)
        return;
    d *= 1.0 / f[3];

    f[0] = TAngle3Normalized(n1, d);
    f[1] = TAngle3Normalized(n2, d);
    f[2] = TAngle3Normalized(n1, n2);
}
void PPF6DDetector::match(CloudPointT2Ptr pc, std::vector<obj6D::Pose6DPtr>& results, const double relativeSceneSampleStep)
{
    int angle_dim = (int)(360.0f/angle_step);
    int dist_dim = (int)(1.0/dist_sampling);
    int numRefPoints = pc->points.size();
    int numModlePoints = pc_model.rows;
    int sceneSamplingStep = (int)(1.0/relativeSceneSampleStep);
    int numAngles = (int) (floor (360.0f / angle_step));
    int acc_space = numModlePoints*numAngles;
    int flag_space = angle_dim*angle_dim*angle_dim*dist_dim;
    //std::cout<<"acc_space: "<<acc_space<<std::endl;
    std::vector<obj6D::Pose6DPtr> poseList(0);

    pcl::KdTreeFLANN<PointT2> kdtree;
    kdtree.setInputCloud(pc);
    float radius = dist_threshold;

#pragma omp parallel for
    for (size_t i = 0; i < numRefPoints; i += sceneSamplingStep)
    {
        pcl::PointNormal referencePoint = pc->points[i];
        uint refIndMax = 0, alphaIndMax = 0;
        uint maxVotes = 0;
        const cv::Vec3f p1(referencePoint.x, referencePoint.y, referencePoint.z);
        const cv::Vec3f n1(referencePoint.normal_x, referencePoint.normal_y, referencePoint.normal_z);
        cv::Vec3d tsg = cv::Vec3d::all(0);
        cv::Matx33d Rsg = cv::Matx33d::all(0), RInv = cv::Matx33d::all(0);
        computeTransformRT(p1, n1, Rsg, tsg);
        uint* accumulator = (uint*)calloc(acc_space, sizeof(uint));
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        ulong* flag = (ulong*)calloc(flag_space, sizeof(ulong));
        //uint* flag = (uint*)calloc(angle_dim*angle_dim*angle_dim*dist_dim*numAngles, sizeof(uint));
        if(kdtree.radiusSearch(referencePoint,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance)>0)
        {
            //std::cout<<pointIdxRadiusSearch.size()<<std::endl;
            for(size_t j = 1; j < pointIdxRadiusSearch.size(); ++j)
            {
                pcl::PointNormal point = pc->points[pointIdxRadiusSearch[j]];
                const cv::Vec3f p2(point.x, point.y, point.z);
                const cv::Vec3f n2(point.normal_x, point.normal_y, point.normal_z);
                cv::Vec3d p2t;
                double alpha_scene;

                cv::Vec4d f = cv::Vec4d::all(0);
                computePPFFeatures(p1, n1, p2, n2, f);
                //std::cout<<f<<std::endl;
                int idx;
                KeyType hashValue = hashPPF(f, angle_step*CV_PI/180.0f, distance_step, idx);

                p2t = tsg + Rsg * cv::Vec3d(p2);
                alpha_scene=atan2(-p2t[2], p2t[1]);

                if (alpha_scene != alpha_scene)
                {
                    continue;
                }

                if (sin(alpha_scene)*p2t[2]<0.0)
                    alpha_scene=-alpha_scene;

                alpha_scene=-alpha_scene;
                int alpha_scene_index = (int)(numAngles*(alpha_scene + 2*M_PI) / (4*M_PI));
                uint alphaIndex = idx;
                ulong alpha_scene_bit = pow(2, alpha_scene_index);
                ulong t = flag[alphaIndex] | alpha_scene_bit;
                if(t==flag[alphaIndex]){continue;}
                else{flag[alphaIndex] = t;}
                //                uint alphaIndex = idx * numAngles + alpha_scene_index;
                //                if(flag[alphaIndex] == 1){continue;}
                //                else{flag[alphaIndex]++;}
                hashnode_i* node = hashtableGetBucketHashed(hash_table, (hashValue));
                while (node)
                {
                    THash* tData = (THash*) node->data;
                    int corrI = (int)tData->i;
                    int ppfInd = (int)tData->ppfInd;
                    double alpha_model = angle_table.ptr<float>(ppfInd)[0];
                    double alpha = alpha_model - alpha_scene;

                    int alpha_index = (int)(numAngles*(alpha + 2*M_PI) / (4*M_PI));
                    uint accIndex = corrI * numAngles + alpha_index;
                    accumulator[accIndex]++;
                    node = node->next;
                }
            }
            // Maximize the accumulator
            for (uint k = 0; k < numModlePoints; k++)
            {
                for (int j = 0; j < numAngles; j++)
                {
                    const uint accInd = k*numAngles + j;
                    const uint accVal = accumulator[ accInd ];
                    if (accVal > maxVotes)
                    {
                        maxVotes = accVal;
                        refIndMax = k;
                        alphaIndMax = j;
                    }
                }
            }
            cv::Vec3d tInv, tmg;
            cv::Matx33d Rmg;
            RInv = Rsg.t();
            tInv = -RInv * tsg;

            cv::Matx44d TsgInv;
            rtToPose(RInv, tInv, TsgInv);

            const cv::Vec3f pMax(pc_model.ptr<float>(refIndMax));
            const cv::Vec3f nMax(pc_model.ptr<float>(refIndMax)+3);

            computeTransformRT(pMax, nMax, Rmg, tmg);

            cv::Matx44d Tmg;
            rtToPose(Rmg, tmg, Tmg);

            // convert alpha_index to alpha
            int alpha_index = alphaIndMax;

            double alpha = (alpha_index*(4*M_PI))/numAngles-2*M_PI;

            cv::Matx44d Talpha;
            cv::Matx33d R;
            cv::Vec3d t = cv::Vec3d::all(0);
            getUnitXRotation(alpha, R);

            rtToPose(R, t, Talpha);

            cv::Matx44d rawPose = TsgInv * (Talpha * Tmg);

            obj6D::Pose6DPtr pose(new obj6D::Pose6D(maxVotes));
            pose->updatePose(rawPose);
#pragma omp critical
            {
                poseList.push_back(pose);
            }
        }
        free(accumulator);
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
        free(flag);
    }
    clusterPoses(poseList, results);
}
void PPF6DDetector::clusterPoses(std::vector<obj6D::Pose6DPtr>& poseList, std::vector<obj6D::Pose6DPtr> &finalPoses)
{
    std::vector<obj6D::PoseCluster6DPtr> poseClusters(0);
    finalPoses.clear();

    std::sort(poseList.begin(), poseList.end(), pose6DPtrCompare);
    for (int i=0; i<0.5*poseList.size(); i++)
    {
        obj6D::Pose6DPtr pose = poseList[i];
        bool assigned = false;
        for (size_t j=0; j<poseClusters.size() && !assigned; j++)
        {
            const obj6D::Pose6DPtr poseCenter = poseClusters[j]->poseList[0];
            if (matchPose(*pose, *poseCenter))
            {
                poseClusters[j]->addPose(pose);
                assigned = true;
            }
        }
        if (!assigned)
        {
            poseClusters.push_back(obj6D::PoseCluster6DPtr(new obj6D::PoseCluster6D(pose)));
        }
    }
    // sort the clusters so that we could output multiple hypothesis
    std::sort(poseClusters.begin(), poseClusters.end(), sortPoseClusters);

    finalPoses.resize(poseClusters.size());

#pragma omp parallel for
    for (int i=0; i<static_cast<int>(poseClusters.size()); i++)
    {
        cv::Vec4d qAvg = cv::Vec4d::all(0);
        cv::Matx44d qAvg_mat = cv::Matx44d::all(0);
        cv::Vec3d tAvg = cv::Vec3d::all(0);

        // Perform the final averaging
        obj6D::PoseCluster6DPtr curCluster = poseClusters[i];
        std::vector<obj6D::Pose6DPtr> curPoses = curCluster->poseList;
        const int curSize = (int)curPoses.size();

        for (int j=0; j<curSize; j++)
        {
            qAvg_mat += curPoses[j]->q*curPoses[j]->q.t();
            tAvg += curPoses[j]->pose_t;
        }

        cv::Mat eigVect, eigVal;
        eigen(qAvg_mat, eigVal, eigVect);
        qAvg = eigVect.row(0);

        tAvg *= 1.0 / double(curSize);
        curPoses[0]->updatePoseQuat(qAvg, tAvg);
        curPoses[0]->numVotes=curCluster->numVotes;
        finalPoses[i]=curPoses[0]->clone();
    }
    poseClusters.clear();
}
void PPF6DDetector::clearTrainingModels()
{
    if (this->hash_nodes)
    {
        free(this->hash_nodes);
        this->hash_nodes=0;
    }

    if (this->hash_table)
    {
        hashtableDestroy(this->hash_table);
        this->hash_table=0;
    }
}
PPF6DDetector::~PPF6DDetector()
{
    clearTrainingModels();
}
}
