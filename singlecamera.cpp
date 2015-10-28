

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

vector<Rect> getEdge(Mat image, int threshold);
Rect getCarPosition(Mat &image, BackgroundSubtractorMOG2 &mog);
Rect trackCar(Mat &image, Rect original_rect, int method);


int main() {
	BackgroundSubtractorMOG2 mog;//定义一个提取背景类类的对象
	Rect car_rect = Rect(0, 0, 0, 0); // 小车初始的位置
	vector<Rect> obstacle_position;
	double pixel2x = 1; //将像素的位置转换成实际的坐标值的系数
	double pixel2y = 1;

	VideoCapture capture;//定义一个VideoCapture类的对象
	VideoWriter capture_save;
	capture.open("1.flv");// 打开摄像头，也可以改成视频路径读取视频文件如将0改为" D:/1.avi"
	//capture.open("test.avi");
	if (!capture.isOpened()) {
		cout << "打开摄像头失败！" << endl;
		return -1;
	}
	size_t frameNo = 0;//保存视频的帧数
	Mat frame;//保存视频每一帧的数据

	capture >> frame;
	capture_save.open("result.avi", CV_FOURCC('M', 'J', 'P', 'G'), 33, frame.size(), true);
	capture_save << frame;
	while (capture.read(frame)) {
		++frameNo;
		cout << frameNo << endl;//输出当前的帧数

		//设置兴趣区域;参数为左上角的x,y，宽，高
		Rect image_roi_rect(70, 0, frame.cols - 150, frame.rows);
		Mat image_roi = frame(image_roi_rect); //要处理的图像
		// frame(image_roi_rect).copyTo(image_roi);
		// frame.copyTo(image_roi);

		// 检测小车位置结束
		if (frameNo >= 15) {
			if(frameNo%5==0){
				car_rect = getCarPosition(image_roi, mog);
				obstacle_position = getEdge(image_roi, 130);
			}
			// rectangle(image_roi, car_rect, Scalar(255, 0, 0), 3, 8, 0);//外接矩形框
			if(car_rect.area()>20){
				trackCar(image_roi, car_rect, 1);
			}

			// 获取障碍物的位置
			for (vector<Rect>::iterator it = obstacle_position.begin(); it != obstacle_position.end(); it++)//遍历所有符合条件的外接矩形
			{
				if (it->x >= car_rect.x - 5 && it->y > car_rect.y - 5 && it->x + it->width <= car_rect.x + car_rect.width && it->y + it->height <= car_rect.y + car_rect.height) {
					continue;
				}
				rectangle(image_roi, *it, Scalar(0, 0, 255), 3, 8, 0);//外接矩形框
			}
		}

		imshow("image", frame);
		capture_save << frame;
		char c = waitKey(33); //控制帧率
	}

	return 0;
}

vector<Rect> getEdge(Mat image, int threshold) {
	Mat src_gray;
	GaussianBlur( image, src_gray, Size(3, 3), 0, 0, BORDER_DEFAULT );
	// 如果是彩色图就转换成灰度图
	if (src_gray.channels() == 3) {
		cvtColor( src_gray, src_gray, CV_RGB2GRAY );
	}
	else if (src_gray.channels() == 1) {
		src_gray.copyTo(src_gray);
	}
	else {
		cout << "image error" << endl;
	}

	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;

	//改为Scharr滤波器计算x轴导数
	Scharr(src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT);
	// Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
	convertScaleAbs(grad_x, abs_grad_x);
	//改为Scharr滤波器计算y轴导数
	Scharr(src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT);
	// Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
	convertScaleAbs( grad_y, abs_grad_y );
	Mat grad;
	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

	// imshow("grad", grad);
	Mat bin_grad;
	cv::threshold(grad, bin_grad, threshold, 255, cv::THRESH_BINARY);
	// imshow("bin_grad", bin_grad);


	vector<vector<Point> > contours;//定义存储边界所需的点
	vector<Vec4i> hierarchy;//定义存储层次的向量
	//检测所有轮廓并重构层次
	findContours(bin_grad, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
	//画出所有轮廓
	// drawContours(frame, contours, -1, Scalar(255, 0, 0));
	vector<Rect> rects;//存储符合条件的外接矩形，由boundingRect函数返回
	int idx = 0;//轮廓个数循环
	if (contours.size())//必须加上此判断，否则当视频中只有背景时会出错
	{
		for (; idx >= 0; idx = hierarchy[idx][0])//找到面积最大的轮廓（hierarchy[idx][0]会指向下一个轮廓，若没有下一个轮廓则hierarchy[idx][0]为负数）
		{
			drawContours(grad, contours, idx, Scalar(255, 0, 0));//画出该轮廓线;;仅仅为了测试

			if (fabs(contourArea(Mat(contours[idx]))) > 500)//如果当前轮廓的面积改阈值，则保存其外接矩形；调节可以改变外接矩形框的个数
			{
				rects.push_back(boundingRect(contours[idx]));//压栈保存符合条件的外接矩形
			}
		}
	}

	// // 下面代码仅仅为了测试
	// for (vector<Rect>::iterator it = rects.begin(); it != rects.end(); it++)//遍历所有符合条件的外接矩形
	// {
	// 	rectangle(grad, *it, Scalar(255, 0, 255), 3, 8, 0);//在前景画出符合条件的外接矩形框
	// 	rectangle(image, *it, Scalar(255, 0, 255), 3, 8, 0);//在前景画出符合条件的外接矩形框
	// 	// rectangle(bin_grad, *it, Scalar(255, 255, 0), 3, 8, 0);//在视频中画出符合条件的外接矩形框
	// }
	// imshow("gray", grad);//显示视频帧
	// // imshow("bin_grad", bin_grad);
	// imshow("image", image);

	return rects;
}

Rect getCarPosition(Mat &image, BackgroundSubtractorMOG2 &mog) {
	Mat foreground;//保存前景帧数据
	Mat background;//保存背景帧数据
	// 运动前景检测，并更新背景
	mog(image, foreground, 0.05);//0.05为更新速率，可自己调整
	//去除噪声
	dilate(foreground, foreground, Mat(), Point(-1, -1), 1);//膨胀
	erode(foreground, foreground, Mat(), Point(-1, -1), 2);//腐蚀
	dilate(foreground, foreground, Mat(), Point(-1, -1), 1);

	mog.getBackgroundImage(background);   // 返回当前背景图像
	// 背景建模结束

	// 在前景上检测得到的矩形应该就是小车
	// Mat fgdrect;
	// foreground.copyTo(fgdrect);
	// fgdrect = fgdrect > 50; //转化成二值图
	// imshow("foreground", fgdrect);
	vector<Rect> car_rect = getEdge(foreground, 110);
	// if (!car_rect.empty()) {
	// 	return car_rect[0];// 只有一个小车，所以应该只返回一个矩形框
	// }
	double max_area = 0;
	if (car_rect.empty()) {
		return Rect(0, 0, 0, 0);
	}
	vector<Rect>::iterator max_area_it = car_rect.begin();
	for (vector<Rect>::iterator it = car_rect.begin(); it != car_rect.end(); ++it) {
		if (it->area() > max_area) {
			max_area = it->area();
			max_area_it = it;
		}
	}
	return *max_area_it;

	// vector<vector<Point> > contours;//定义存储边界所需的点
	// vector<Vec4i> hierarchy;//定义存储层次的向量
	// //检测所有轮廓并重构层次
	// findContours(fgdrect, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
	// //画出所有轮廓
	// // drawContours(frame, contours, -1, Scalar(255, 0, 0));
	// vector<Rect> rects;//存储符合条件的外接矩形，由boundingRect函数返回
	// int idx = 0;//轮廓个数循环
	// if (contours.size())//必须加上此判断，否则当视频中只有背景时会出错
	// {
	// 	for (; idx >= 0; idx = hierarchy[idx][0])//找到面积最大的轮廓（hierarchy[idx][0]会指向下一个轮廓，若没有下一个轮廓则hierarchy[idx][0]为负数）
	// 	{
	// 		drawContours(foreground, contours, idx, Scalar(255, 0, 0));//前景上画出该轮廓线;;仅仅为了测试

	// 		if (fabs(contourArea(Mat(contours[idx]))) > 500)//如果当前轮廓的面积改阈值，则保存其外接矩形；调节可以改变外接矩形框的个数
	// 		{
	// 			rects.push_back(boundingRect(contours[idx]));//压栈保存符合条件的外接矩形
	// 		}
	// 	}
	// }

	// for (vector<Rect>::iterator it = rects.begin(); it != rects.end(); it++)//遍历所有符合条件的外接矩形
	// {
	// 	rectangle(foreground, *it, Scalar(255, 0, 255), 3, 8, 0);//在前景画出符合条件的外接矩形框
	// 	rectangle(image, *it, Scalar(255, 0, 255), 3, 8, 0);//在兴趣区域画出符合条件的外接矩形框
	// }
	// if(!rects.empty()){
	// 	return rects[0];// 只有一个小车，所以应该只返回一个矩形框
	// }
}

// original_rect表示小车最初的位置;method表示跟踪的方法是meanshift还是camshift
Rect trackCar(Mat &image, Rect original_rect, int method) {
	// 使用camshift跟踪小车的位置
	cv::Rect trackWindow = original_rect; //定义跟踪的矩形;;可以固定小车的起始位置，其位置的坐标就放到该矩形内即可
	RotatedRect trackBox;//定义一个旋转的矩阵类对象，由CamShift返回
	int hsize = 16;//每一维直方图的大小
	float hranges[] = { 0, 180 };//hranges在后面的计算直方图函数中要用到
	int vmin = 10, vmax = 256, smin = 30;
	const float* phranges = hranges;//
	cv::Mat hsv, hue, mask, hist, backproj;

	cvtColor(image, hsv, CV_BGR2HSV);//将rgb摄像头帧转化成hsv空间的
	//inRange函数的功能是检查输入数组每个元素大小是否在2个给定数值之间，可以有多通道,mask保存0通道的最小值，也就是h分量
	//这里利用了hsv的3个通道，比较h,0~180,s,smin~256,v,min(vmin,vmax),max(vmin,vmax)。如果3个通道都在对应的范围内，则
	//mask对应的那个点的值全为1(0xff)，否则为0(0x00).
	inRange(hsv, Scalar(0, smin, 10), Scalar(180, 256, 256), mask);
	int ch[] = { 0, 0 };
	hue.create(hsv.size(), hsv.depth());//hue初始化为与hsv大小深度一样的矩阵，色调的度量是用角度表示的，红绿蓝之间相差120度，反色相差180度
	mixChannels(&hsv, 1, &hue, 1, ch, 1);//将hsv第一个通道(也就是色调)的数复制到hue中，0索引数组

	//此处的构造函数roi用的是Mat hue的矩阵头，且roi的数据指针指向hue，即共用相同的数据，select为其感兴趣的区域
	// trackWindow = *itRect;
	Mat roi(hue, trackWindow), maskroi(mask, trackWindow);//mask保存的hsv的最小值

	//calcHist()函数第一个参数为输入矩阵序列，第2个参数表示输入的矩阵数目，第3个参数表示将被计算直方图维数通道的列表，第4个参数表示可选的掩码函数
	//第5个参数表示输出直方图，第6个参数表示直方图的维数，第7个参数为每一维直方图数组的大小，第8个参数为每一维直方图bin的边界
	calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);//将roi的0通道计算直方图并通过mask放入hist中，hsize为每一维直方图的大小
	normalize(hist, hist, 0, 255, CV_MINMAX);//将hist矩阵进行数组范围归一化，都归一化到0~255

	calcBackProject(&hue, 1, 0, hist, backproj, &phranges);//计算直方图的反向投影，计算hue图像0通道直方图hist的反向投影，并让入backproj中
	backproj &= mask;

	// camshift
	if (1 == method) {
		trackBox = CamShift(backproj, trackWindow, TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1)); //trackWindow为开始的区域，TermCriteria为确定迭代终止的准则;CV_TERMCRIT_EPS是通过forest_accuracy,CV_TERMCRIT_ITER
		if (trackWindow.area() <= 1) {
			int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
			trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
			                   trackWindow.x + r, trackWindow.y + r) &
			              Rect(0, 0, cols, rows);//Rect函数为矩阵的偏移和大小，即第一二个参数为矩阵的左上角点坐标，第三四个参数为矩阵的宽和高
		}

		// ellipse(image, trackBox, Scalar(255, 255, 0), 3, CV_AA);//跟踪的时候以椭圆为代表目标
		Rect car_rect;//小车的外接矩形
		car_rect.x = trackBox.center.x - trackBox.size.width / 2;
		car_rect.y = trackBox.center.y - trackBox.size.height / 2;
		car_rect.width = trackBox.size.width;
		car_rect.height = trackBox.size.height;
		putText(image, "car", cv::Point(car_rect.x, car_rect.y), FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0));//在图片中输出字符
		cv::rectangle(image, car_rect, Scalar(255, 255, 0), 3, 0);
		return car_rect;
	}
	// meanshift
	else {
		meanShift(backproj, trackWindow, TermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));//trackWindow为开始的区域，TermCriteria为确定迭代终止的准则
		//ellipse( image, trackBox, Scalar(0,0,255), 3, CV_AA );//跟踪的时候以椭圆为代表目标
		// rectangle(image,Point(trackWindow.x,trackWindow.y),Point(trackWindow.x+trackWindow.width,trackWindow.y+trackWindow.height),Scalar(0,0,255),-1,CV_AA);// -1表示填充
		cv::rectangle(image, trackWindow, Scalar(255, 255, 0), 3, 0);
		return trackWindow;
	}
}


