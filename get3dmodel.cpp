// kyle 2015-07-15

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
// // Ϊ��ʹ��ͼ���㷨����ӵ�ͷ�ļ�
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
	// �ɺ�������Ĳ���������ʹ�õ�ƥ���㷨
	int SADWindowSize = 0, numberOfDisparities = 0;
	bool no_display = false;
	float scale = 1.f;

	StereoBM bm;
	StereoSGBM sgbm;
	StereoVar var;
	// BM�㷨ֻ�������ڻҶ�ͼ
	int color_mode = algorithm_no == STEREO_BM ? 0 : -1;
	//Mat img1 = imread(left_image, color_mode);
	//Mat img2 = imread(right_image, color_mode);
	Mat img1, img2;
	left_image.copyTo(img1);
	right_image.copyTo(img2);
	// ͼƬ�Ƿ������Ų���;scale����������
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
	//�����ڲξ����ļ�
	FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
	if (!fs.isOpened()) {
		cout << intrinsic_filename << "�ļ���ʧ��" << endl;
	}
	//˫����ڲ�
	Mat M1, D1, M2, D2;
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;

	M1 *= scale;
	M2 *= scale;

	fs.open(extrinsic_filename, CV_STORAGE_READ);
	if (!fs.isOpened()) {
		cout << extrinsic_filename << "�ļ���ʧ��" << endl;
	}
	// ˫������
	Mat R, T, R1, P1, R2, P2;
	fs["R"] >> R;
	fs["T"] >> T;
	// �������ʹ��ͼ��ƽ��ƽ��
	stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);
	// ��ʼ�������������ģ��
	Mat map11, map12, map21, map22;
	initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
	initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
	// ��������ͼ��ʹ��Լ���ƽ��;�������õ��ľ��Ǳ�Ե������ͼ��;������֮��Ϳ���ʹ��ƥ���㷨
	Mat img1r, img2r;
	remap(img1, img1r, map11, map12, INTER_LINEAR);
	remap(img2, img2r, map21, map22, INTER_LINEAR);

	img1 = img1r;
	img2 = img2r;
	imwrite("leftr.jpg", img1r);
	imwrite("rightr.jpg", img2r);

	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;
	// bm�㷨�Ĳ�������
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

	// sgbm�㷨�Ĳ�������
	// SADWindowSize��SAD���ڴ�С������Χ��[1,11]��һ��Ӧ���� 3x3 �� 11x11 ֮�䣬����������������int ��
	sgbm.preFilterCap = 63;
	sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
	int cn = img1.channels();
	sgbm.P1 = 8 * cn * sgbm.SADWindowSize * sgbm.SADWindowSize;
	sgbm.P2 = 32 * cn * sgbm.SADWindowSize * sgbm.SADWindowSize;
	sgbm.minDisparity = 0;// ���û��Ѳ���
	sgbm.numberOfDisparities = numberOfDisparities;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleWindowSize = bm.state->speckleWindowSize;
	sgbm.speckleRange = bm.state->speckleRange;
	sgbm.disp12MaxDiff = 1;
	sgbm.fullDP = algorithm_no == STEREO_HH;

	// var�㷨�Ĳ�������
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

	Mat disp, disp8;// �����Ӳ�ͼ
	//Mat img1p, img2p, dispp;
	//copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
	//copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

	// ��������Ĳ���ѡ��ͬ���㷨����������ƥ��
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
	// ͼ���㷨��ʱ�������Ч����֪Ϊ��û�бȽϺ�
	// else if (algorithm_no == STEREO_GC) {
	//     CvStereoGCState* state = cvCreateStereoGCState( 16, 2 );
	//     //  ��Matת��const* CvArr,��ʹ�ãǣ��㷨
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
	printf("����ʱ��: %fms\n", t * 1000 / getTickFrequency());

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
		namedWindow("�Ӳ�ͼ", 1);
		imshow("�Ӳ�ͼ", disp8);
		fflush(stdout);
	}

	if (disparity_filename) {
		imwrite(disparity_filename, disp8);
	}

	Mat xyz;
	// ��ά��ͶӰ��������Ϣ;����λ����Ϣ
	reprojectImageTo3D(disp, xyz, Q, true);

	if (true) {
		vector<vector<Point> > contours;//����洢�߽�����ĵ�
		vector<Vec4i> hierarchy;//����洢��ε�����
		// cv::Point

		Mat disp_fore;;
		// disp8.copyTo(disp_fore);
		img1.copyTo(disp_fore);
		// cvtColor(disp_fore, disp_fore, CV_BGR2GRAY);
		/// ʹ�� 3x3�ں˽���
		// blur(disp_fore, disp_fore, Size(3, 3));
		Canny(disp_fore, disp_fore, 50, 150, 3);//������������ͼ��,���ͼ��,����ֵ,����ֵ����opencv�����ǵ���ֵ��3��

		//��ͼ�ζ�ֵ��������ֵ���ڵ��������������ص㣬����ֵ������Ϊ���ĸ�����������Ϊ0
		threshold(disp_fore, disp_fore, 5, 255, THRESH_BINARY);

		//��������������ع����
		findContours(disp_fore, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
		//������������
		// drawContours(frame, contours, -1, Scalar(255, 0, 0));
		vector<Rect> rects;//�洢������������Ӿ��Σ���boundingRect��������
		int idx = 0;//��������ѭ��
		if (contours.size())//������ϴ��жϣ�������Ƶ��ֻ�б���ʱ�����
		{
			for (; idx >= 0; idx = hierarchy[idx][0])//�ҵ��������������hierarchy[idx][0]��ָ����һ����������û����һ��������hierarchy[idx][0]Ϊ������
			{
				drawContours(disp_fore, contours, idx, Scalar(255, 0, 0));//������������

				if (fabs(contourArea(Mat(contours[idx]))) > 5000)//�����ǰ�������������ֵ���򱣴�����Ӿ��Σ����ڿ��Ըı���Ӿ��ο�ĸ���
				{
					rects.push_back(boundingRect(contours[idx]));//ѹջ���������������Ӿ���
				}
			}
		}

		for (vector<Rect>::iterator it = rects.begin(); it != rects.end(); it++)//�������з�����������Ӿ���
		{
			rectangle(disp_fore, *it, Scalar(255, 0, 255), 3, 8, 0);//��ǰ������������������Ӿ��ο�
			// rectangle(outline, *it, Scalar(255, 255, 0), 3, 8, 0);//����Ƶ�л���������������Ӿ��ο�
			// ����ȡĿ�����
			float object_depth = 0;
			// ���ú�ѡ����Ϊ��Ȳο���׼
			vector<float> candidate_point;
			int candidate_x = 200;// x����ֱ�ߵ�����
			int candidate_y = 100;// y����ֱ�ߵ�����
			int candidate_save_num = 1;// ������ѡ�������һ��
			for (int i = 1; i <= candidate_y; ++i) {
				for (int j = 1; j <= candidate_x; ++j) {
					object_depth = xyz.at<Vec3f>((int)(it->x + (1.0 / (candidate_x + 1)) * j * it->width), (int)(it->y + (1.0 / (candidate_y + 1)) * i * it->height))[2];

					// �����ѡ���x,y,z����
					// cout << xyz.at<Vec3f>((int)(it->x + (1.0 / (candidate_x + 1)) * j * it->width), (int)(it->y + (1.0 / (candidate_y + 1)) * i * it->height))[0] << " "  \
					                    //      << xyz.at<Vec3f>((int)(it->x + (1.0 / (candidate_x + 1)) * j * it->width), (int)(it->y + (1.0 / (candidate_y + 1)) * i * it->height))[1] << " "  \
					                    //      << xyz.at<Vec3f>((int)(it->x + (1.0 / (candidate_x + 1)) * j * it->width), (int)(it->y + (1.0 / (candidate_y + 1)) * i * it->height))[2] << endl;

					candidate_point.push_back(object_depth);
				}
			}
			// ����ѡ������
			std::sort(candidate_point.begin(), candidate_point.end());
			// ֻ����ָ�����м�ļ�����ѡ�㣬���ų�����
			int num = 0;
			object_depth = 0;
			for (vector<float>::iterator it_point = candidate_point.begin() + (candidate_x * candidate_y / 2 - candidate_save_num); it_point != candidate_point.end() - (candidate_x * candidate_y / 2 - candidate_save_num); ++it_point) {
				object_depth += *it_point;
				num++;
			}
			object_depth /= num;
			cout << "Ŀ�����" << object_depth * 8 / 10 << "cm" << endl;

			// ����ȡĿ��ĺ�������
			// ������ʹ��ͼ���к������������м伸��λ�õ�ƽ��ֵ��������ʹ��ͼƬ�м�һ�е��������������м伸��ֵ��ƽ��ֵ
			int object_left_x = 0;
			int object_left_y = 0;
			int object_right_x = 0;
			int object_right_y = 0;

			// ��ץȡ��������x
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

			// ��ץȡ��������y
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

			// ��ץȡ��������x
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

			// ��ץȡ��������y
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

			cout << "��ȡ�����X1:(" << object_left_x * 8 / 10 << "," << object_left_y * 8 / 10 << ") " << "X2:(" << object_right_x * 8 / 10 << "," << object_right_y * 8 / 10 << ")" << endl;
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