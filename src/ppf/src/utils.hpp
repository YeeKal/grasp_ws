#pragma once
#include <opencv/cv.h>
#include <pcl/common/common_headers.h>
#include <cmath>
#include <cstdio>


static const float EPS = 1.192092896e-07F; /* smallest such that 1.0+FLT_EPSILON != 1.0 */

static void TNormalize3(cv::Vec3d& v)
{
    double norm = cv::norm(v);
    if (norm > EPS)
    {
        v *= 1.0 / norm;
    }
}
static double TAngle3Normalized(const cv::Vec3d& a, const cv::Vec3d& b)
{
    /*
    angle = atan2(a dot b, |a x b|) # Bertram (accidental mistake)
    angle = atan2(|a x b|, a dot b) # Tolga Birdal (correction)
    angle = acos(a dot b)           # Hamdi Sahloul (simplification, a & b are normalized)
    */
    return acos(a.dot(b));
}
static void rtToPose(const cv::Matx33d& R, const cv::Vec3d& t, cv::Matx44d& Pose)
{
    cv::Matx34d P;
    cv::hconcat(R, t, P);
    cv::vconcat(P, cv::Matx14d(0, 0, 0, 1), Pose);
}
static void getUnitXRotation(double angle, cv::Matx33d& Rx)
{
    const double sx = sin(angle);
    const double cx = cos(angle);

    cv::Mat(Rx.eye()).copyTo(Rx);
    Rx(1, 1) = cx;
    Rx(1, 2) = -sx;
    Rx(2, 1) = sx;
    Rx(2, 2) = cx;
}
static void aaToR(const cv::Vec3d& axis, double angle, cv::Matx33d& R)
{
    const double sinA = sin(angle);
    const double cosA = cos(angle);
    const double cos1A = (1 - cosA);
    uint i, j;

    cv::Mat(cosA * cv::Matx33d::eye()).copyTo(R);
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
        {
            if (i != j)
            {
                // Symmetry skew matrix
                R(i, j) += (((i + 1) % 3 == j) ? -1 : 1) * sinA * axis[3 - i - j];
            }
            R(i, j) += cos1A * axis[i] * axis[j];
        }
}
static void poseToR(const cv::Matx44d& Pose, cv::Matx33d& R)
{
    cv::Mat(Pose).rowRange(0, 3).colRange(0, 3).copyTo(R);
}

static void poseToRT(const cv::Matx44d& Pose, cv::Matx33d& R, cv::Vec3d& t)
{
    poseToR(Pose, R);
    cv::Mat(Pose).rowRange(0, 3).colRange(3, 4).copyTo(t);
}
static void dcmToQuat(cv::Matx33d& R, cv::Vec4d& q)
{
    float tr = cv::trace(R);
    if ( tr > 0.0f )
    {
        float s = 0.5f / sqrtf(tr + 1.0f);
        q[0] = 0.25f / s;
        q[1] = ( R(2,1) - R(1,2) ) * s;
        q[2] = ( R(0,2) - R(2,0) ) * s;
        q[3] = ( R(1,0) - R(0,1) ) * s;
    }
    else
    {
        if ( R(0,0) > R(1,1) && R(0,0) > R(2,2) )
        {
            float s = 0.5f / sqrtf( 1.0f + R(0,0) - R(1,1) - R(2,2));
            q[0] = (R(2,1) - R(1,2)) * s;
            q[1] =  0.25f / s;
            q[2] = (R(0,1) + R(1,0)) * s;
            q[3] = (R(2,0) + R(0,2)) * s;
        }else if(R(1,1)>R(2,2))
        {
            float s = 0.5f / sqrtf( 1.0f + R(1,1) - R(0,0) - R(2,2));
            q[0] = (R(0,2) - R(2,0)) * s;
            q[1] = (R(0,1) + R(1,0)) * s;
            q[2] = 0.25f / s;
            q[3] = (R(1,2) + R(2,1)) * s;
        }else
        {
            float s = 0.5f / sqrtf( 1.0f + R(2,2) - R(0,0) - R(1,1));
            q[0] = (R(1,0) - R(0,1)) * s;
            q[1] = (R(0,2) + R(2,0)) * s;
            q[2] = (R(1,2) + R(2,1)) * s;
            q[3] = 0.25f / s;
        }
    }
}
static void quatToDCM(cv::Vec4d& q, cv::Matx33d& R)
{
    cv::Vec4d sq = q.mul(q);

    double tmp1, tmp2;

    R(0, 0) = sq[0] + sq[1] - sq[2] - sq[3]; // since norm(q) = 1
    R(1, 1) = sq[0] - sq[1] + sq[2] - sq[3];
    R(2, 2) = sq[0] - sq[1] - sq[2] + sq[3];

    tmp1 = q[1] * q[2];
    tmp2 = q[3] * q[0];

    R(0, 1) = 2.0 * (tmp1 - tmp2);
    R(1, 0) = 2.0 * (tmp1 + tmp2);

    tmp1 = q[1] * q[3];
    tmp2 = q[2] * q[0];

    R(0, 2) = 2.0 * (tmp1 + tmp2);
    R(2, 0) = 2.0 * (tmp1 - tmp2);

    tmp1 = q[2] * q[3];
    tmp2 = q[1] * q[0];

    R(1, 2) = 2.0 * (tmp1 - tmp2);
    R(2, 1) = 2.0 * (tmp1 + tmp2);
}
static void computeTransformRT(const cv::Vec3d& p1, const cv::Vec3d& n1, cv::Matx33d& R, cv::Vec3d& t)
{
    // dot product with x axis
    double angle = acos(n1[0]);

    // cross product with x axis
    cv::Vec3d axis(0, n1[2], -n1[1]);
    // we try to project on the ground plane but it's already parallel
    if (n1[1] == 0 && n1[2] == 0)
    {
        axis[1] = 1;
        axis[2] = 0;
    }
    else
    {
        TNormalize3(axis);
    }
    aaToR(axis, angle, R);

    t = -R * p1;
}
static void opencvPC2PCL(const cv::Mat PC, pcl::PointCloud<pcl::PointNormal> &PC_pcl)
{
    PC_pcl.resize(0);
#pragma omp parallel for
    for(int i=0;i<PC.rows;i++)
    {
        pcl::PointNormal point;
        point.x = PC.at<float>(i,0);
        point.y = PC.at<float>(i,1);
        point.z = PC.at<float>(i,2);
        point.normal_x = PC.at<float>(i,3);
        point.normal_y = PC.at<float>(i,4);
        point.normal_z = PC.at<float>(i,5);
#pragma omp critical
        {
            PC_pcl.push_back(point);
        }
    }
}
static void pcl2cvMat(cv::Mat &pc_mat, pcl::PointCloud<pcl::PointNormal> pc_pcl)
{
    pc_mat = cv::Mat(pc_pcl.points.size(), 6, CV_32F);
    #pragma omp parallel for
    for(int i=0;i<pc_pcl.points.size();i++)
    {
        float *pcData = pc_mat.ptr<float>(i);
        pcData[0]=(float)pc_pcl.points[i].x;
        pcData[1]=(float)pc_pcl.points[i].y;
        pcData[2]=(float)pc_pcl.points[i].z;
        pcData[3]=(float)pc_pcl.points[i].normal_x;
        pcData[4]=(float)pc_pcl.points[i].normal_y;
        pcData[5]=(float)pc_pcl.points[i].normal_z;
    }
}

template <typename T>
static void computeBboxStd(T pc, cv::Vec2f& xRange, cv::Vec2f& yRange, cv::Vec2f& zRange)
{
    xRange[0] = pc->points[0].x;
    xRange[1] = pc->points[0].x;
    yRange[0] = pc->points[0].y;
    yRange[1] = pc->points[0].y;
    zRange[0] = pc->points[0].z;
    zRange[1] = pc->points[0].z;
#pragma omp parallel for
    for(int i=0;i<pc->points.size();i++)
    {
        float x = pc->points[i].x;
        float y = pc->points[i].y;
        float z = pc->points[i].z;

        if (x<xRange[0])
            xRange[0]=x;
        if (x>xRange[1])
            xRange[1]=x;

        if (y<yRange[0])
            yRange[0]=y;
        if (y>yRange[1])
            yRange[1]=y;

        if (z<zRange[0])
            zRange[0]=z;
        if (z>zRange[1])
            zRange[1]=z;
    }
}
static Eigen::Matrix4d cvMat2Eigen(cv::Matx44d mat)
{
    Eigen::Matrix4d mat_eigen;
    for(int r=0;r<4;r++)
        for(int c=0;c<4;c++)
            mat_eigen(r,c) = mat(r,c);
    return mat_eigen;
}
