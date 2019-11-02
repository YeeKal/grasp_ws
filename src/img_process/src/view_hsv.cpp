
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
#include<iostream>
#include <fstream>
#include<string>
#include <algorithm>
using namespace std;
//输入图像
Mat img;
//灰度值归一化
Mat bgr;
//HSV图像
Mat hsv;
//色相
int hmin = 85;
int hmin_Max = 180;
int hmax = 125;
int hmax_Max = 180;
//饱和度
int smin = 20;
int smin_Max = 255;
int smax = 255;
int smax_Max = 255;
//亮度
int vmin = 20;
int vmin_Max = 255;
int vmax = 255;
int vmax_Max = 235;
//显示原图的窗口
string windowName = "src";
//输出图像的显示窗口
string dstName = "dst";
//输出图像
Mat dst;
//回调函数



void printHelp(){
	std::cout<<"view  HSV of the pictures in some. "<<std::endl;
}
int main(int argc, char** argv)
{
	string folder_name="imgs";
	string file_type="jpg";

	if(argc>1){
		folder_name=argv[1];
	}
	if(argc>2){
		file_type=argv[2];
	}

	string file_template=folder_name+"/*."+file_type;
	vector<cv::String> image_files;
	cv::glob(file_template,image_files);

	Mat mask,bgr,hsv,dst;
	int image_num=0;

	vector<int> hv(181),sv(256,0),vv(256,0);
	for(int i=0;i<image_files.size();i++){
		std::cout<<"view image:"<<image_num<<std::endl;
		Mat src=imread(image_files[i]);
		bgr=src.clone();
		GaussianBlur(src, bgr, cv::Size(5, 5), 3, 3);
		// Mat element=getStructuringElement(MORPH_RECT, Size(15,15));
		// morphologyEx(bgr, bgr, MORPH_OPEN, element);

		cvtColor(bgr, hsv, CV_BGR2HSV);

		dst = Mat::zeros(src.size(), src.type());
		inRange(hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), mask);

		//open close
		Mat element=getStructuringElement(MORPH_RECT, Size(15,15));
		morphologyEx(mask, mask, MORPH_OPEN, element);

		//contour
		vector<vector<cv::Point>> contours;
		cv::findContours(mask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE); 
		// 寻找最大连通域  
		double maxArea = 0;  
		int max_contour=0;
		vector<cv::Point> maxContour;  
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

		// final mask
		Mat final_mask = Mat::zeros(src.rows, src.cols, CV_8UC1);
		// for(int i=0;i<maxContour.size();i++){
		// 	final_mask.at<uchar>(maxContour[i].y, maxContour[i].x)=255;
		// 	circle(src, maxContour[i], 2, Scalar(0,0,255),FILLED);

		// }
		drawContours(final_mask, contours, max_contour, Scalar(255), CV_FILLED);

		for (int r = 0; r < bgr.rows; r++)
		{
			for (int c = 0; c < bgr.cols; c++)
			{
				if (final_mask.at<uchar>(r, c) == 255)
				{
					dst.at<Vec3b>(r, c) = src.at<Vec3b>(r, c);
				}
			}
		}
		imshow("mask",final_mask);
		imshow("seg", dst);
		waitKey(0);
	}
	return 0;
}
