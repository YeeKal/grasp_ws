#ifndef HSV_SEGMENTATION_H
#define HSV_SEGMENTATION_H

#include<iostream>
#include<string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class HSVSegmentation{
public:
	cv::Mat src_,bgr_,hsv_;
	cv::Mat mask_,final_mask_;
	int hmin_,hmax_,smin_,smax_,vmin_,vmax_;
	HSVSegmentation(int hmin,int hmax,int smin,int smax,int vmin,int vmax)
	:hmin_(hmin),hmax_(hmax),smin_(smin),smax_(smax),vmin_(vmin),vmax_(vmax)
	{}
	~HSVSegmentation(){}
	//Mat对象做为函数参数，不论是传引用，传指针，对于传值调用，在函数内对其进行的操作会作用到原对象
	void setSrc(cv::Mat src){
		src_=src;//not need deep copy
		mask_ = Mat::zeros(src_.rows, src_.cols, CV_8UC1);
		final_mask_ = Mat::zeros(src_.rows, src_.cols, CV_8UC1);
		segProcess();
	}
	void segProcess(){
		cv::GaussianBlur(src_, bgr_, cv::Size(5, 5), 3, 3);
		cv::cvtColor(bgr_, hsv_, CV_BGR2HSV);
		cv::inRange(hsv_, Scalar(hmin_, smin_, vmin_), Scalar(hmax_, smax_, vmax_), mask_);

		//open close
		cv::Mat element=getStructuringElement(MORPH_RECT, Size(15,15));
		cv::morphologyEx(mask_, mask_, MORPH_OPEN, element);

		//contour
		std::vector<std::vector<cv::Point> > contours;
		cv::findContours(mask_,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE); 
		// 寻找最大连通域  
		double maxArea = 0;  
		int max_contour=0;
		std::vector<cv::Point> maxContour;  
		for(size_t i = 0; i < contours.size(); i++)  
		{  
			double area = cv::contourArea(contours[i]);  
			if (area > maxArea)  
			{  
				maxArea = area;  
				max_contour=i; 
			}  
		}  

		maxContour=contours[max_contour];
		drawContours(final_mask_, contours, max_contour, Scalar(255), CV_FILLED);
	}

	void getMask(Mat &mask){
		mask=final_mask_.clone();
	}

	void getColorDst(Mat &img){
		Mat dst;
		dst = Mat::zeros(src_.size(), src_.type());
		for (int r = 0; r < src_.rows; r++)
		{
			for (int c = 0; c < src_.cols; c++)
			{
				if (final_mask_.at<uchar>(r, c) == 255)
				{
					dst.at<Vec3b>(r, c) = src_.at<Vec3b>(r, c);
				}
			}
		}
		img= dst.clone();
	}

	// filter the depth img by the mask
	void getDepthDst(Mat &img){
		Mat dst;
		dst = Mat::zeros(img.size(),img.type());
		for (int r = 0; r < img.rows; r++)
		{
			for (int c = 0; c < img.cols; c++)
			{
				if (final_mask_.at<uchar>(r, c) == 255)
				{
					dst.at<float>(r, c) = img.at<float>(r, c);
				}
			}
		}
		img= dst.clone();
	}

};  // End class HSVSegmentation

#endif