// kyle 2015-07-15

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
// // 为了使用图割算法而添加的头文件
// #include "cvaux.hpp"
// #include "cv.h"
// #include "highgui.h"
// #include "cxcore.h"

#include <iostream>
#include <algorithm>
#include <stdio.h>

using namespace cv;
using namespace std;


void get_3d_model(Mat& left_image, Mat& right_image, int algorithm_no = 1, const string intrinsic_filename = "intrinsics.yml",
	const string extrinsic_filename = "extrinsics.yml") {

	const char* disparity_filename = "disparity.jpg";

	enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_GC = 4 };
	// 由函数传入的参数来决定使用的匹配算法
	int SADWindowSize = 0, numberOfDisparities = 0;
	bool no_display = false;
	float scale = 1.f;

	StereoBM bm;
	StereoSGBM sgbm;
	StereoVar var;
	// BM算法只可以用于灰度图
	int color_mode = algorithm_no == STEREO_BM ? 0 : -1;
	//Mat img1 = imread(left_image, color_mode);
	//Mat img2 = imread(right_image, color_mode);
	Mat img1, img2;
	left_image.copyTo(img1);
	right_image.copyTo(img2);
	// 图片是否做缩放操作;scale是缩放因子
	if (scale != 1.f) {
		Mat temp1, temp2;
		int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
		resize(img1, temp1, Size(), scale, scale, method);
		img1 = temp1;
		resize(img2, temp2, Size(), scale, scale, method);
		img2 = temp2;
	}

	Size img_size = img1.size();

	Rect roi1, roi2;
	Mat Q;
	//读入内参矩阵文件
	FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
	if (!fs.isOpened()) {
		cout << intrinsic_filename << "文件打开失败" << endl;
	}
	//双相机内参
	Mat M1, D1, M2, D2;
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;

	M1 *= scale;
	M2 *= scale;

	fs.open(extrinsic_filename, CV_STORAGE_READ);
	if (!fs.isOpened()) {
		cout << extrinsic_filename << "文件打开失败" << endl;
	}
	// 双相机外参
	Mat R, T, R1, P1, R2, P2;
	fs["R"] >> R;
	fs["T"] >> T;
	// 矫正相机使其图像平面平行
	stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);
	// 初始化两个相机矫正模型
	Mat map11, map12, map21, map22;
	initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
	initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
	// 矫正两幅图像使其对极线平行;矫正完后得到的就是边缘弯曲的图像;矫正完之后就可以使用匹配算法
	Mat img1r, img2r;
	remap(img1, img1r, map11, map12, INTER_LINEAR);
	remap(img2, img2r, map21, map22, INTER_LINEAR);

	img1 = img1r;
	img2 = img2r;
	imwrite("leftr.jpg", img1r);
	imwrite("rightr.jpg", img2r);

	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;
	// bm算法的参数设置
	bm.state->roi1 = roi1;
	bm.state->roi2 = roi2;
	bm.state->preFilterCap = 31;
	bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
	bm.state->minDisparity = 0;
	bm.state->numberOfDisparities = numberOfDisparities;
	bm.state->textureThreshold = 10;
	bm.state->uniquenessRatio = 15;
	bm.state->speckleWindowSize = 100;
	bm.state->speckleRange = 32;
	bm.state->disp12MaxDiff = 1;

	// sgbm算法的参数设置
	// SADWindowSize：SAD窗口大小，容许范围是[1,11]，一般应该在 3x3 至 11x11 之间，参数必须是奇数，int 型
	sgbm.preFilterCap = 63;
	sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
	int cn = img1.channels();
	sgbm.P1 = 8 * cn * sgbm.SADWindowSize * sgbm.SADWindowSize;
	sgbm.P2 = 32 * cn * sgbm.SADWindowSize * sgbm.SADWindowSize;
	sgbm.minDisparity = 0;// 设置回搜参数
	sgbm.numberOfDisparities = numberOfDisparities;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleWindowSize = bm.state->speckleWindowSize;
	sgbm.speckleRange = bm.state->speckleRange;
	sgbm.disp12MaxDiff = 1;
	sgbm.fullDP = algorithm_no == STEREO_HH;

	// var算法的参数设置
	var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
	var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
	var.nIt = 25;
	var.minDisp = -numberOfDisparities;
	var.maxDisp = 0;
	var.poly_n = 3;
	var.poly_sigma = 0.0;
	var.fi = 15.0f;
	var.lambda = 0.03f;
	var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
	var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
	var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING;

	Mat disp, disp8;// 保存视差图
	//Mat img1p, img2p, dispp;
	//copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
	//copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

	// 根据输入的参数选择不同的算法来进行立体匹配
	int64 t = getTickCount();
	if (algorithm_no == STEREO_BM) {
		bm(img1, img2, disp);
	}
	else if (algorithm_no == STEREO_VAR) {
		var(img1, img2, disp);
	}
	else if (algorithm_no == STEREO_SGBM || algorithm_no == STEREO_HH) {
		sgbm(img1, img2, disp);
	}
	// 图割算法，时间很慢，效果不知为何没有比较好
	// else if (algorithm_no == STEREO_GC) {
	//     CvStereoGCState* state = cvCreateStereoGCState( 16, 2 );
	//     //  将Mat转化const* CvArr,来使用ＧＣ算法
	//     IplImage* img1_ = cvLoadImage("leftr.jpg", 0);
	//     IplImage* img2_ = cvLoadImage("rightr.jpg", 0);
	//     CvMat* left_disp_  = cvCreateMat( img1.size().height, img1.size().width, CV_32F );
	//     CvMat* right_disp_ = cvCreateMat( img2.size().height, img2.size().width, CV_32F );

	//     cvFindStereoCorrespondenceGC( img1_, img2_, left_disp_, right_disp_, state, 0 );
	//     cvReleaseStereoGCState( &state );
	//     // post-progressing the result
	//     CvMat* disparity_left_visual = cvCreateMat( img1.size().height, img1.size().width, CV_8U );
	//     cvConvertScale( left_disp_, disparity_left_visual, -16 );
	//     cvSave( "disparity.jpg", disparity_left_visual );
	//     cvNamedWindow( "disparity", 1);
	//     cvShowImage("disparity", disparity_left_visual );
	//     cvWaitKey( 0 );
	//     cvDestroyWindow( "disparity" );
	// }
	t = getTickCount() - t;
	printf("消耗时间: %fms\n", t * 1000 / getTickFrequency());

	//disp = dispp.colRange(numberOfDisparities, img1p.cols);
	if (algorithm_no != STEREO_VAR) {
		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities * 16.));
	}
	else {
		disp.convertTo(disp8, CV_8U);
	}
	if (!no_display) {
		namedWindow("left", 1);
		imshow("left", img1);
		namedWindow("right", 1);
		imshow("right", img2);
		namedWindow("视差图", 1);
		imshow("视差图", disp8);
		fflush(stdout);
	}

	if (disparity_filename) {
		imwrite(disparity_filename, disp8);
	}

	Mat xyz;
	// 三维重投影获得深度信息;坐标位置信息
	reprojectImageTo3D(disp, xyz, Q, true);

	if (true) {
		vector<vector<Point> > contours;//定义存储边界所需的点
		vector<Vec4i> hierarchy;//定义存储层次的向量
		// cv::Point

		Mat disp_fore;;
		// disp8.copyTo(disp_fore);
		img1.copyTo(disp_fore);
		// cvtColor(disp_fore, disp_fore, CV_BGR2GRAY);
		/// 使用 3x3内核降噪
		// blur(disp_fore, disp_fore, Size(3, 3));
		Canny(disp_fore, disp_fore, 50, 150, 3);//参数：（输入图像,输出图像,低阈值,高阈值）；opencv建议是低阈值的3倍

		//将图形二值化，像素值大于第三个参数的像素点，像素值被设置为第四个参数，否则为0
		threshold(disp_fore, disp_fore, 5, 255, THRESH_BINARY);

		//检测所有轮廓并重构层次
		findContours(disp_fore, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
		//画出所有轮廓
		// drawContours(frame, contours, -1, Scalar(255, 0, 0));
		vector<Rect> rects;//存储符合条件的外接矩形，由boundingRect函数返回
		int idx = 0;//轮廓个数循环
		if (contours.size())//必须加上此判断，否则当视频中只有背景时会出错
		{
			for (; idx >= 0; idx = hierarchy[idx][0])//找到面积最大的轮廓（hierarchy[idx][0]会指向下一个轮廓，若没有下一个轮廓则hierarchy[idx][0]为负数）
			{
				drawContours(disp_fore, contours, idx, Scalar(255, 0, 0));//画出该轮廓线

				if (fabs(contourArea(Mat(contours[idx]))) > 5000)//如果当前轮廓的面积改阈值，则保存其外接矩形；调节可以改变外接矩形框的个数
				{
					rects.push_back(boundingRect(contours[idx]));//压栈保存符合条件的外接矩形
				}
			}
		}

		for (vector<Rect>::iterator it = rects.begin(); it != rects.end(); it++)//遍历所有符合条件的外接矩形
		{
			rectangle(disp_fore, *it, Scalar(255, 0, 255), 3, 8, 0);//在前景画出符合条件的外接矩形框
			// rectangle(outline, *it, Scalar(255, 255, 0), 3, 8, 0);//在视频中画出符合条件的外接矩形框
			// 待夹取目标深度
			float object_depth = 0;
			// 设置候选点作为深度参考标准
			vector<float> candidate_point;
			int candidate_x = 200;// x方向划直线的条数
			int candidate_y = 100;// y方向划直线的条数
			int candidate_save_num = 1;// 保留候选点个数的一半
			for (int i = 1; i <= candidate_y; ++i) {
				for (int j = 1; j <= candidate_x; ++j) {
					object_depth = xyz.at<Vec3f>((int)(it->x + (1.0 / (candidate_x + 1)) * j * it->width), (int)(it->y + (1.0 / (candidate_y + 1)) * i * it->height))[2];

					// 输出候选点的x,y,z坐标
					// cout << xyz.at<Vec3f>((int)(it->x + (1.0 / (candidate_x + 1)) * j * it->width), (int)(it->y + (1.0 / (candidate_y + 1)) * i * it->height))[0] << " "  \
					                    //      << xyz.at<Vec3f>((int)(it->x + (1.0 / (candidate_x + 1)) * j * it->width), (int)(it->y + (1.0 / (candidate_y + 1)) * i * it->height))[1] << " "  \
					                    //      << xyz.at<Vec3f>((int)(it->x + (1.0 / (candidate_x + 1)) * j * it->width), (int)(it->y + (1.0 / (candidate_y + 1)) * i * it->height))[2] << endl;

					candidate_point.push_back(object_depth);
				}
			}
			// 将候选点排序
			std::sort(candidate_point.begin(), candidate_point.end());
			// 只保留指定的中间的几个候选点，来排除干扰
			int num = 0;
			object_depth = 0;
			for (vector<float>::iterator it_point = candidate_point.begin() + (candidate_x * candidate_y / 2 - candidate_save_num); it_point != candidate_point.end() - (candidate_x * candidate_y / 2 - candidate_save_num); ++it_point) {
				object_depth += *it_point;
				num++;
			}
			object_depth /= num;
			cout << "目标深度" << object_depth * 8 / 10 << "cm" << endl;

			// 待夹取目标的横纵坐标
			// 横坐标使用图像列横坐标排序后的中间几个位置的平均值；纵坐标使用图片中间一行的纵坐标排序后的中间几个值的平均值
			int object_left_x = 0;
			int object_left_y = 0;
			int object_right_x = 0;
			int object_right_y = 0;

			// 左抓取点坐标点的x
			candidate_point.clear();
			for (int i = 1; i <= candidate_y; ++i) {
				object_left_x = xyz.at<Vec3f>((int)(it->x), (int)(it->y + (1.0 / (candidate_y + 1)) * i * it->height))[0];
				candidate_point.push_back(object_left_x);
			}
			sort(candidate_point.begin(), candidate_point.end());
			num = 0;
			object_left_x = 0;
			for (vector<float>::iterator it_point = candidate_point.begin() + (candidate_y / 2 - candidate_save_num); it_point != candidate_point.end() - (candidate_y / 2 - candidate_save_num); ++it_point) {
				object_left_x += *it_point;
				num++;
			}
			object_left_x /= num;

			// 左抓取点坐标点的y
			candidate_point.clear();
			for (int i = 1; i <= candidate_x; ++i) {
				object_left_y = xyz.at<Vec3f>((int)(it->x + (1.0 / (candidate_x + 1)) * i * it->width), (int)(it->y + 0.5 * it->height))[1];
				candidate_point.push_back(object_left_y);
			}
			sort(candidate_point.begin(), candidate_point.end());
			num = 0;
			object_left_y = 0;
			for (vector<float>::iterator it_point = candidate_point.begin() + (candidate_x / 2 - candidate_save_num); it_point != candidate_point.end() - (candidate_x / 2 - candidate_save_num); ++it_point) {
				object_left_y += *it_point;
				num++;
			}
			object_left_y /= num;

			// 右抓取点坐标点的x
			candidate_point.clear();
			for (int i = 1; i <= candidate_y; ++i) {
				object_right_x = xyz.at<Vec3f>((int)(it->x + it->width), (int)(it->y + (1.0 / (candidate_y + 1)) * i * it->height))[0];
				candidate_point.push_back(object_right_x);
			}
			sort(candidate_point.begin(), candidate_point.end());
			num = 0;
			object_right_x = 0;
			for (vector<float>::iterator it_point = candidate_point.begin() + (candidate_y / 2 - candidate_save_num); it_point != candidate_point.end() - (candidate_y / 2 - candidate_save_num); ++it_point) {
				object_right_x += *it_point;
				num++;
			}
			object_right_x /= num;

			// 右抓取点坐标点的y
			candidate_point.clear();
			for (int i = 1; i <= candidate_x; ++i) {
				object_right_y = xyz.at<Vec3f>((int)(it->x + (1.0 / (candidate_x + 1)) * i * it->width), (int)(it->y + 0.5 * it->height))[1];
				candidate_point.push_back(object_right_y);
			}
			sort(candidate_point.begin(), candidate_point.end());
			num = 0;
			object_right_y = 0;
			for (vector<float>::iterator it_point = candidate_point.begin() + (candidate_x / 2 - candidate_save_num); it_point != candidate_point.end() - (candidate_x / 2 - candidate_save_num); ++it_point) {
				object_right_y += *it_point;
				num++;
			}
			object_right_y /= num;

			cout << "夹取坐标点X1:(" << object_left_x * 8 / 10 << "," << object_left_y * 8 / 10 << ") " << "X2:(" << object_right_x * 8 / 10 << "," << object_right_y * 8 / 10 << ")" << endl;
		}
		imshow("rect", disp_fore);
	}
	waitKey();
}

int main() {
	VideoCapture left_capture;
	VideoCapture right_capture;
	left_capture.open(0);
	right_capture.open(1);
	Mat left, right;
	while (1){
		left_capture >> left;
		right_capture >> right;
		imshow("left", left);
		imshow("right", right);
		get_3d_model(left, right, 1, "intrinsics.yml", "extrinsics.yml");
		char c = waitKey(33);
		if (c == 27){
			break;
		}
	}
	return 0;
}