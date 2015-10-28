////kyle 2015-07-15
//
//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//
//#include <vector>
//#include <string>
//#include <sstream>
//#include <iostream>
//
//using namespace cv;
//using namespace std;
//
//
//void StereoCalibate(bool capture_new_image = true, bool show_rectified = true) {
//	vector<string> imagelist;//存储采集到的图片的名字，用于标定
//	cv::Size board_size(6, 8);
//	if (capture_new_image) {
//		//采集用于标定的图片
//
//		//左右相机
//		cv::VideoCapture left_capture;
//		cv::Mat left_frame;
//		cv::VideoCapture right_capture;
//		cv::Mat right_frame;
//		cv::Mat left_image, right_image;//用于保存标定图片
//		//左右相机的序号
//		int left_cam_no = 0;
//		int right_cam_no = 1;
//		int picture_no = 0;//左右相机采集的图片对的个数
//		string left_prefix = "left"; //以left,right作为保存的图片名称的前缀
//		string right_prefix = "right";
//
//		// 打开的摄像头
//		left_capture.open(left_cam_no);
//		if (!left_capture.isOpened()) {
//			cout << "打开左摄像头失败" << endl;
//		}
//		else {
//			cout << "成功打开左摄像头" << endl;
//		}
//		right_capture.open(right_cam_no);
//		if (!right_capture.isOpened()) {
//			cout << "打开右摄像头失败" << endl;
//		}
//		else {
//			cout << "成功打开右摄像头" << endl;
//		}
//		cout << "按‘s’键拍照保存；按‘esc’键退出\n" << endl;
//
//		namedWindow("vedio");
//		while (1) {
//			Mat left_image, right_image;
//			left_capture >> left_frame;
//			if (left_frame.channels() == 3){
//				cvtColor(left_frame, left_image, CV_BGR2GRAY);
//			}
//			else{
//				left_frame.copyTo(left_image);
//			}
//			right_capture >> right_frame;
//			if (right_frame.channels() == 3){
//				cvtColor(right_frame, right_image, CV_BGR2GRAY);
//			}
//			else{
//				right_frame.copyTo(right_image);
//			}
//			Mat vedio_image;
//			vedio_image.push_back(left_frame);
//			vedio_image.push_back(right_frame);
//			imshow("vedio", vedio_image);
//
//			char c = waitKey(33);
//			// 按‘s’键保存当前帧为标定图片
//			if (c == 's') {
//				picture_no++;//记录采集图片的个数，同时用于区别保存图片的名字
//				string picture_no_temp;
//				stringstream ss;
//				ss << picture_no;
//				ss >> picture_no_temp;
//				string left_name = left_prefix;//每次采集时重新定义图片名字字符串
//				left_name += picture_no_temp;
//				left_name += ".jpg";
//				imwrite(left_name, left_image);
//				imagelist.push_back(left_name);// 保存采集到的图片的名称
//				cout << "保存图片" << left_name << endl;
//				string right_name = right_prefix;//每次采集时重新定义图片名字字符串
//				right_name += picture_no_temp;
//				right_name += ".jpg";
//				imwrite(right_name, right_image);
//				imagelist.push_back(right_name);// 保存采集到的图片的名称
//				cout << "保存图片" << right_name << endl;
//			}
//			// 按键esc退出标定图片采集程序
//			if (c == 27) {
//				cout << "共保存" << picture_no << "对图片" << endl;
//				destroyWindow("vedio");
//				break;
//			}
//		}
//		// 采集程序完成
//	}
//	else {
//		const string filename = "stereo_calib.xml";
//		imagelist.resize(0);
//		FileStorage fs(filename, FileStorage::READ);
//		if (!fs.isOpened()) {
//			cout << "打开文件失败" << endl;
//		}
//		FileNode n = fs.getFirstTopLevelNode();
//		if (n.type() != FileNode::SEQ) {
//			cout << "文件格式错误" << endl;
//		}
//		FileNodeIterator it = n.begin(), it_end = n.end();
//		for (; it != it_end; ++it) {
//			imagelist.push_back((string)*it);
//		}
//
//	}
//
//	if (imagelist.size() % 2 != 0) {
//		cout << "错误:图片的个数不是偶数个,无法标定\n";
//		return;
//	}
//
//	const int max_scale = 2;
//	const float square_size = 70.f;  // 棋盘格的宽度
//	// ARRAY AND VECTOR STORAGE:
//
//	vector<vector<Point2f> > corners_vect[2];
//	vector<vector<Point3f> > world_points_vect;
//	Size image_size;
//
//	int i, j, k, images_num = (int)imagelist.size() / 2;
//
//
//	vector<string> good_image_list;
//	namedWindow("corners");
//	// 遍历刚才采集到的用于标定的图片
//	for (i = j = 0; i < images_num; i++) {
//		// 表示左右相机的两张图片
//		for (k = 0; k < 2; k++) {
//			const string image_name = imagelist[i * 2 + k];
//			Mat img = imread(image_name, 0);
//			if (img.empty()) {
//				break;
//			}
//			//读取第一张图片之前，image_size为空;此时将其赋值为第一张图片的尺寸
//			if (image_size == Size()) {
//				image_size = img.size();
//			}
//			//以第一张图片为基准，判断图片的尺寸是否相同
//			else if (img.size() != image_size) {
//				cout << image_name << "与第一张图片大小不相同，跳过这一对图片\n";
//				break;
//			}
//			bool found = false;
//			vector<Point2f> corners;
//			for (int scale = 1; scale <= max_scale; scale++) {
//				Mat timg;
//				if (scale == 1)
//					timg = img;
//				else
//					// 如果没有检测到角点，将图片放大，然后再检测
//					resize(img, timg, Size(), scale, scale);
//				// 检测角点
//				found = findChessboardCorners(timg, board_size, corners,
//					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
//				if (found) {
//					// 如果是检测放大后的图片，需要将角点缩小放大倍数
//					if (scale > 1) {
//						Mat cornersMat(corners);
//						cornersMat *= 1. / scale;
//					}
//					break;
//				}
//			}
//			// 显示检测到的角点
//			Mat corner_img;
//			cvtColor(img, corner_img, COLOR_GRAY2BGR);
//			// 画出检测到的角点
//			drawChessboardCorners(corner_img, board_size, corners, found);
//			double sf = 640. / MAX(img.rows, img.cols);
//			resize(corner_img, corner_img, Size(), sf, sf);
//			imshow("corners", corner_img);
//			char c = (char)waitKey(200);
//			if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
//				exit(-1);
//
//			if (!found)
//				break;
//			// 精确查找角点，亚像素点
//			cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
//				TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS,
//				30, 0.01));
//			// 保存检测到的角点
//			corners_vect[k].push_back(corners);
//		}
//		if (k == 2)	{
//			good_image_list.push_back(imagelist[i * 2]);
//			good_image_list.push_back(imagelist[i * 2 + 1]);
//			j++;
//			// 如果检测到角点的左视图个数比检测角点的右视图个数大一个;说明上一次只检测了左视图的角点，
//			if (corners_vect[0].size() == corners_vect[1].size() + 1) {
//				vector<Point2f> corners_temp = *(corners_vect[0].end() - 1);
//				if (!corners_vect[0].empty()) {
//					corners_vect[0].pop_back();
//					corners_vect[0].push_back(corners_temp);
//				}
//			}
//		}
//	}
//	// 关闭显示角点的窗口
//	destroyWindow("corners");
//	cout << "成功检测到" << j << "对图片的角点\n";
//	images_num = j;
//	if (images_num < 2) {
//		cout << "图片太少，不可用于标定\n";
//		return;
//	}
//
//	// 保存世界坐标
//	world_points_vect.resize(images_num);
//	for (i = 0; i < images_num; i++) {
//		for (j = 0; j < board_size.height; j++)
//			for (k = 0; k < board_size.width; k++)
//				world_points_vect[i].push_back(Point3f(j * square_size, k * square_size, 0));
//	}
//
//	cout << "开始立体定标 ...\n";
//
//	Mat intrinsic_matrix[2], distortion_coeffs[2];
//	intrinsic_matrix[0] = Mat::eye(3, 3, CV_64F);
//	intrinsic_matrix[1] = Mat::eye(3, 3, CV_64F);
//	Mat R, T, E, F;
//	// 立体定标
//	double rms = stereoCalibrate(world_points_vect, corners_vect[0], corners_vect[1],
//		intrinsic_matrix[0], distortion_coeffs[0],
//		intrinsic_matrix[1], distortion_coeffs[1],
//		image_size, R, T, E, F,
//		TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
//		CV_CALIB_FIX_ASPECT_RATIO +
//		CV_CALIB_ZERO_TANGENT_DIST +
//		CV_CALIB_SAME_FOCAL_LENGTH +
//		CV_CALIB_RATIONAL_MODEL +
//		CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
//	cout << "标定结束\nRMS误差:" << rms << endl;
//
//	// CALIBRATION QUALITY CHECK
//	// because the output fundamental matrix implicitly
//	// includes all the output information,
//	// we can check the quality of calibration using the
//	// epipolar geometry constraint: m2^t*F*m1=0
//	cout << "检测标定质量" << endl;
//	double err = 0;
//	int npoints = 0;
//	vector<Vec3f> lines[2];
//	for (i = 0; i < images_num; i++) {
//		int npt = (int)corners_vect[0][i].size();
//		Mat imgpt[2];
//		for (k = 0; k < 2; k++) {
//			imgpt[k] = Mat(corners_vect[k][i]);
//			undistortPoints(imgpt[k], imgpt[k], intrinsic_matrix[k], distortion_coeffs[k], Mat(), intrinsic_matrix[k]);
//			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
//		}
//		for (j = 0; j < npt; j++) {
//			double errij = fabs(corners_vect[0][i][j].x * lines[1][j][0] +
//				corners_vect[0][i][j].y * lines[1][j][1] + lines[1][j][2]) +
//				fabs(corners_vect[1][i][j].x * lines[0][j][0] +
//				corners_vect[1][i][j].y * lines[0][j][1] + lines[0][j][2]);
//			err += errij;
//		}
//		npoints += npt;
//	}
//	cout << "重投影的平均误差:" << err / npoints << endl;
//
//	// 存储相机内参数
//	FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
//	if (fs.isOpened()) {
//		fs << "M1" << intrinsic_matrix[0] << "D1" << distortion_coeffs[0] <<
//			"M2" << intrinsic_matrix[1] << "D2" << distortion_coeffs[1];
//		fs.release();
//	}
//	else {
//		cout << "保存相机内参数失败\n";
//	}
//
//	Mat R1, R2, P1, P2, Q;
//	Rect validRoi[2];
//
//	stereoRectify(intrinsic_matrix[0], distortion_coeffs[0],
//		intrinsic_matrix[1], distortion_coeffs[1],
//		image_size, R, T, R1, R2, P1, P2, Q,
//		CALIB_ZERO_DISPARITY, 1, image_size, &validRoi[0], &validRoi[1]);
//
//	fs.open("extrinsics.yml", CV_STORAGE_WRITE);
//	if (fs.isOpened()) {
//		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
//		fs.release();
//	}
//	else {
//		cout << "保存相机内参数失败\n";
//	}
//
//	// OpenCV can handle left-right
//	// or up-down camera arrangements
//	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));
//
//	// COMPUTE AND DISPLAY RECTIFICATION
//	if (!show_rectified) {
//		return;
//	}
//
//	Mat rmap[2][2];
//
//	//Precompute maps for cv::remap()
//	initUndistortRectifyMap(intrinsic_matrix[0], distortion_coeffs[0], R1, P1, image_size, CV_16SC2, rmap[0][0], rmap[0][1]);
//	initUndistortRectifyMap(intrinsic_matrix[1], distortion_coeffs[1], R2, P2, image_size, CV_16SC2, rmap[1][0], rmap[1][1]);
//
//	Mat canvas;
//	double sf;
//	int w, h;
//	if (!isVerticalStereo) {
//		sf = 600. / MAX(image_size.width, image_size.height);
//		w = cvRound(image_size.width * sf);
//		h = cvRound(image_size.height * sf);
//		canvas.create(h, w * 2, CV_8UC3);
//	}
//	else {
//		sf = 300. / MAX(image_size.width, image_size.height);
//		w = cvRound(image_size.width * sf);
//		h = cvRound(image_size.height * sf);
//		canvas.create(h * 2, w, CV_8UC3);
//	}
//
//	for (i = 0; i < images_num; i++) {
//		for (k = 0; k < 2; k++) {
//			Mat img = imread(good_image_list[i * 2 + k], 0), rimg, cimg;
//			remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
//			cvtColor(rimg, cimg, COLOR_GRAY2BGR);
//			Mat canvasPart = !isVerticalStereo ? canvas(Rect(w * k, 0, w, h)) : canvas(Rect(0, h * k, w, h));
//			resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
//
//			Rect vroi(cvRound(validRoi[k].x * sf), cvRound(validRoi[k].y * sf),
//				cvRound(validRoi[k].width * sf), cvRound(validRoi[k].height * sf));
//			rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
//		}
//
//		if (!isVerticalStereo) {
//			for (j = 0; j < canvas.rows; j += 16) {
//				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
//			}
//		}
//		else {
//			for (j = 0; j < canvas.cols; j += 16) {
//				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
//			}
//		}
//		imshow("rectified", canvas);
//		char c = (char)waitKey(1000);
//		if (c == 27 || c == 'q' || c == 'Q') {
//			break;
//		}
//	}
//}
//
//
//
//int main()
//{
//	StereoCalibate(false);
//	return 0;
//}
