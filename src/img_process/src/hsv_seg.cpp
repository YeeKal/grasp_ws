
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
#include<iostream>
#include<string>
using namespace std;
//输入图像
Mat img;
//灰度值归一化
Mat bgr;
//HSV图像
Mat hsv;
//色相
int hmin = 100;
int hmin_Max = 180;
int hmax = 130;
int hmax_Max = 180;
//饱和度
int smin = 130;
int smin_Max = 255;
int smax = 255;
int smax_Max = 255;
//亮度
int vmin = 80;
int vmin_Max = 255;
int vmax = 220;
int vmax_Max = 255;
//显示原图的窗口
string windowName = "src";
//输出图像的显示窗口
string dstName = "dst";
//输出图像
Mat dst;
//回调函数
void callBack(int, void*)
{
	//输出图像分配内存
	dst = Mat::zeros(img.size(), img.type());
	//掩码
	Mat mask;
	inRange(hsv, Scalar(hmin, smin, vmin), Scalar(hmax, smax, vmax), mask);
	//掩模到原图的转换
	for (int r = 0; r < bgr.rows; r++)
	{
		for (int c = 0; c < bgr.cols; c++)
		{
			if (mask.at<uchar>(r, c) == 255)
			{
				dst.at<Vec3b>(r, c) = bgr.at<Vec3b>(r, c);
			}
		}
	}
	//输出图像
	imshow(dstName, dst);
	//保存图像
	//dst.convertTo(dst, CV_8UC3, 255.0, 0);
	//imwrite("HSV_inRange.jpg", dst);
}
int main(int argc, char** argv)
{
	//输入图像
	img = imread(argv[1]);
	if (!img.data || img.channels() != 3)
		return -1;
	imshow(windowName, img);
	bgr = img.clone();
	//颜色空间转换
	cvtColor(bgr, hsv, CV_BGR2HSV);
	//定义输出图像的显示窗口
	namedWindow(dstName, WINDOW_GUI_EXPANDED);
	//调节色相 H
	createTrackbar("hmin", dstName, &hmin, hmin_Max, callBack);
	createTrackbar("hmax", dstName, &hmax, hmax_Max, callBack);
	//调节饱和度 S
	createTrackbar("smin", dstName, &smin, smin_Max, callBack);
	createTrackbar("smax", dstName, &smax, smax_Max, callBack);
	//调节亮度 V
	createTrackbar("vmin", dstName, &vmin, vmin_Max, callBack);
	createTrackbar("vmax", dstName, &vmax, vmax_Max, callBack);
	callBack(0, 0);
	waitKey(0);
	return 0;
}