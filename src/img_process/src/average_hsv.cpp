
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
int hmin = 0;
int hmin_Max = 180;
int hmax = 180;
int hmax_Max = 180;
//饱和度
int smin = 0;
int smin_Max = 255;
int smax = 255;
int smax_Max = 255;
//亮度
int vmin = 106;
int vmin_Max = 255;
int vmax = 255;
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
class MaskGenerate{
    public:
        Mat fore_img;
        Mat mask_img;
        Mat mask_final;
        string fg_window="mask generate";
        Point2i last_point;
        bool mouse_enable=true;
        MaskGenerate(Mat &fg);
        ~MaskGenerate(){}
        void getMask();
        static void mouseGetMask(int event, int x, int y, int flags, void* ptr);
        void drawMask(int event,int x,int y,int flags);   
};

void printHelp(){
	std::cout<<"Calculate the average HSV of the pictures in some. "<<std::endl;
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

	ofstream file(folder_name+"/hsv.csv");

	string file_template=folder_name+"/*."+file_type;
	vector<cv::String> image_files;
	cv::glob(file_template,image_files);

	Mat mask,bgr,hsv;
	int image_num=0;

	vector<int> hv(181),sv(256,0),vv(256,0);
	for(int i=0;i<image_files.size();i++){
		std::cout<<"process image:"<<image_num<<std::endl;
		Mat src=imread(image_files[i]);
		MaskGenerate maskGenerate(src);
		mask=maskGenerate.mask_final;

		bgr=src.clone();
		cvtColor(bgr, hsv, CV_BGR2HSV);


	//accumulate h-s-v
		for (int r = 0; r < bgr.rows; r++)
		{
			for (int c = 0; c < bgr.cols; c++)
			{
				if (mask.at<uchar>(r, c) == 255)
				{
					hv[hsv.at<Vec3b>(r, c)[0]]++;
					sv[hsv.at<Vec3b>(r, c)[1]]++;
					vv[hsv.at<Vec3b>(r, c)[2]]++;
				}
			}
		}
		image_num++;
	}

	file<<"value,h,s,v\n";
	for(int i=0;i<256;i++){
		file<<i<<",";
		if(hv.size()>i) file<<hv[i]<<",";
		else file<<0<<",";
		file<<sv[i]<<","<<vv[i]<<"\n";
	}

	std::vector<int>::iterator m_index=std::max_element(std::begin(hv),std::end(hv));
	std::cout<<"H:"<<std::distance(std::begin(hv),m_index);
	m_index=std::max_element(std::begin(sv),std::end(sv));
	std::cout<<" S:"<<std::distance(std::begin(sv),m_index);
	m_index=std::max_element(std::begin(vv),std::end(vv));
	std::cout<<" V:"<<std::distance(std::begin(vv),m_index);

	// std::sort(std::begin(hv),std::end(hv));
	// std::sort(std::begin(sv),std::end(sv));
	// std::sort(std::begin(vv),std::end(vv));

	waitKey(0);
	return 0;
}

MaskGenerate::MaskGenerate(Mat& fg){
    fore_img=fg.clone();
    mask_img=Mat::zeros(fg.size(),CV_8UC1);
    getMask();
}
void MaskGenerate::getMask(){
    imshow(fg_window, fore_img);
    //imshow("mask", mask_img);
    cout<<"HINT: drag your mouse to draw a close curve, and right-click inside the curve!"<<endl;
    setMouseCallback(fg_window,MaskGenerate::mouseGetMask,this);
    waitKey(0);
}
void MaskGenerate::mouseGetMask(int event, int x, int y, int flags, void* ptr){
    MaskGenerate * p=(MaskGenerate*) ptr;
    p->drawMask(event,x,y,flags);
    
}
void MaskGenerate::drawMask(int event,int x,int y,int flags){
    if(flags==CV_EVENT_FLAG_LBUTTON && mouse_enable){
        if(!last_point.x){
            last_point=Point2i(x,y);
            circle(fore_img, Point2i(x,y), 2, Scalar(0,0,255),FILLED);
            circle(mask_img,  Point2i(x,y), 2,Scalar(100),FILLED );
            imshow(fg_window, fore_img);
            //imshow("mask", mask_img);
        }
        else{
            line(fore_img,last_point,Point2i(x,y),Scalar(0,0,255),2,FILLED);
            line(mask_img,last_point,Point2i(x,y),Scalar(100),2,FILLED);
            imshow(fg_window, fore_img);
            //imshow("mask", mask_img);
            last_point=Point2i(x,y);
        }
    }
    if(event==CV_EVENT_RBUTTONDOWN && mouse_enable){
	    std::cout<<"click right buton\n";
        floodFill(mask_img, Point2i(x,y), Scalar(255));
        Mat mask_temp;
        Mat element = getStructuringElement(MORPH_RECT, Size(15, 15));
        dilate(mask_img, mask_temp, element);
        threshold(mask_temp, mask_final, 125, 255, THRESH_BINARY);
        //destroyWindow(fg_window);
        //destroyAllWindows();
        mouse_enable=false;
    }
    //cout<<x<<endl;
}