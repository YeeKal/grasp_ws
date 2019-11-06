
#ifndef SUPER4PCS_H
#define SUPER4PCS_H

#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <super4pcs/shared4pcs.h>
#include <super4pcs/algorithms/super4pcs.h>
#include <pcl/console/time.h>

#include "demo-utils.h"

namespace pcl
{
/** Pose estimation firstly by Super4PCS and then refined by ICP
*
**/
struct TransformVisitor {
	inline void operator()(
			float fraction,
			float best_LCP,
			Eigen::Ref<GlobalRegistration::Match4PCSBase::MatrixType> /*transformation*/) const {
	if(fraction >= 0)
		{
		printf("done: %d%c best: %f                  \r",
				static_cast<int>(fraction * 100), '%', best_LCP);
		fflush(stdout);
		}
	}
	constexpr bool needsGlobalTransformation() const { return false; }
};

template <typename PointSource, typename PointTarget>
class Super4pcsICP 
{
	public:
	
	typedef typename pcl::PointCloud<PointSource> PointCloudSource;
	typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
	typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;
	typedef typename pcl::PointCloud<PointTarget> PointCloudTarget;
	typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;

	typedef PointIndices::Ptr PointIndicesPtr;
	typedef PointIndices::ConstPtr PointIndicesConstPtr;


	pcl::IterativeClosestPoint<PointSource, PointTarget> icp_;
	//GlobalRegistration::MatchSuper4PCS matcher_;
	//pcl::NormalEstimationOMP<PointSource,PointTarget> nest_;

	PointCloudSourcePtr src_pcl_;
	PointCloudTargetPtr tar_pcl_;
	PointCloudTargetPtr final_pcl_;
	PointCloudTargetPtr temp_pcl_;

	std::vector<GlobalRegistration::Point3D> src_sp4_, tar_sp4_;
	Eigen::Matrix4f transformation_;

	GlobalRegistration::Match4PCSOptions options_;

	double score_;


	/** \brief Constructor */
	Super4pcsICP (int argc, char **argv)
	: src_pcl_(new PointCloudSource),
	  tar_pcl_(new PointCloudTarget),
	  final_pcl_(new PointCloudTarget),
	  temp_pcl_(new PointCloudTarget)
	{
		initMatcher(argc, argv);
	}
	Super4pcsICP()
	: src_pcl_(new PointCloudSource),
	  tar_pcl_(new PointCloudTarget),
	  final_pcl_(new PointCloudTarget),
	  temp_pcl_(new PointCloudTarget)
	{}

	virtual ~Super4pcsICP (){}

	void initMatcher(int argc, char **argv){
		GlobalRegistration::Demo::getArgs(argc, argv);
		GlobalRegistration::Demo::setOptionsFromArgs(options_);
		icp_.setMaximumIterations (GlobalRegistration::Demo::n_ite);
	}

	void setInputSource (const PointCloudSourcePtr &cloud){
		pcl::copyPointCloud(*cloud,*src_pcl_);
		fillPointSet(*cloud,src_sp4_);

		//icp_.setInputSource(cloud);
	}

	void setInputTarget (const PointCloudTargetPtr &cloud){
		pcl::copyPointCloud(*cloud,*tar_pcl_);
		fillPointSet(*cloud,tar_sp4_);
		icp_.setInputTarget(cloud);

	}

	void matchDebug(){
		//Ini Super4PCS matcher
		using namespace GlobalRegistration;
		constexpr Utils::LogLevel loglvl = Utils::Verbose;
		using SamplerType   = GlobalRegistration::Sampling::UniformDistSampler;
		using TrVisitorType = typename std::conditional <loglvl==Utils::NoLog,
									Match4PCSBase::DummyTransformVisitor,
									TransformVisitor>::type;
		SamplerType sampler;
		TrVisitorType visitor;
		Utils::Logger logger(loglvl);
		GlobalRegistration::MatchSuper4PCS matcher=MatchSuper4PCS(options_, logger);

		//first use Super4PCS natch function
		pcl::console::TicToc time;
		time.tic();
		score_ = matcher.ComputeTransformation(tar_sp4_, &src_sp4_, transformation_,sampler,visitor);
		std::cout<<"Time:"<<time.toc()<<"ms"<<std::endl;

		std::cout<<"score(Super4PCS):"<<score_<<std::endl;
		print4x4Matrix(transformation_);

		//PointCloudTargetPtr temp_pcl(new PointCloudTarget());
		transformPointCloud (*src_pcl_, *temp_pcl_, transformation_);
		icp_.setInputSource(temp_pcl_);

		//second use icp to refine the transformation
		time.tic();
		icp_.align(*final_pcl_);
		std::cout<<"Time:"<<time.toc()<<"ms"<<std::endl;

		if(icp_.hasConverged()){
			score_=icp_.getFitnessScore ();
			std::cout << "\nICP has converged, score is " <<score_ << std::endl;
			transformation_ *=icp_.getFinalTransformation ();
			print4x4Matrix(transformation_);
		}

	}

	void match(){
		//Ini Super4PCS matcher
		using namespace GlobalRegistration;
		constexpr Utils::LogLevel loglvl = Utils::Verbose;
		using SamplerType   = GlobalRegistration::Sampling::UniformDistSampler;
		using TrVisitorType = typename std::conditional <loglvl==Utils::NoLog,
									Match4PCSBase::DummyTransformVisitor,
									TransformVisitor>::type;
		SamplerType sampler;
		TrVisitorType visitor;
		Utils::Logger logger(loglvl);
		GlobalRegistration::MatchSuper4PCS matcher=MatchSuper4PCS(options_, logger);

		//first use Super4PCS natch function
		score_ = matcher.ComputeTransformation(tar_sp4_, &src_sp4_, transformation_,sampler,visitor);

		//PointCloudTargetPtr temp_pcl(new PointCloudTarget());
		transformPointCloud (*src_pcl_, *temp_pcl_, transformation_);
		icp_.setInputSource(temp_pcl_);

		//second use icp to refine the transformation
		icp_.align(*final_pcl_);

		if(icp_.hasConverged()){
			score_=icp_.getFitnessScore ();
			//std::cout << "\nICP has converged, score is " <<score_ << std::endl;
			transformation_ *=icp_.getFinalTransformation ();
			//print4x4Matrix(transformation_);
		}

	}

	

	// conveert from pcl:PointCloud to point in Super4PCS
	void fillPointSet(const PointCloudSource& m, std::vector<GlobalRegistration::Point3D>& out) {
		out.clear();
		out.reserve(m.size());

		// TODO: copy other point-wise information, if any
		for(size_t i = 0; i< m.size(); i++){
			const auto& point = m[i];
			out.emplace_back(point.x, point.y, point.z);
		}
	}

	void print4x4Matrix (const Eigen::Matrix4f & matrix)
	{
		printf ("Rotation matrix :\n");
		printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
		printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
		printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
		printf ("Translation vector :\n");
		printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
	}
};	// end class Super4pcsICP
}//end namespace pcl
#endif
