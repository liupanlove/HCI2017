/*
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
	Ptr<TrackerKCF> tracker = TrackerKCF::create();
	VideoCapture video(0);
	if (!video.isOpened()) {
		cerr << "cannot read video!" << endl;
		return -1;
	}
	Mat frame;
	video.read(frame);
	Rect2d box(270, 120, 180, 260);
	tracker->init(frame, box);
	while (video.read(frame)) {
		tracker->update(frame, box);
		rectangle(frame, box, Scalar(255, 0, 0), 2, 1);
		imshow("Tracking", frame);
		int k = waitKey(1);
		if (k == 27) break;
	}
	return 0;
}
*/

/*
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
	VideoCapture video(0);

	Mat srcImage;

	while (video.read(srcImage)) {

		Mat srcGrayImage;
		if (srcImage.channels() == 3)
			cvtColor(srcImage, srcGrayImage, CV_RGB2GRAY);
		else
			srcImage.copyTo(srcGrayImage);

		Mat new_srcGrayImage;

		GaussianBlur(srcGrayImage, new_srcGrayImage, Size(5, 5), 0, 0);
		vector<KeyPoint>detectKeyPoint;
		Mat keyPointImage;
		
		Ptr<FastFeatureDetector> fast = FastFeatureDetector::create();
		fast->detect(new_srcGrayImage, detectKeyPoint);
		//drawKeypoints(srcImage, detectKeyPoint, keyPointImage, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		drawKeypoints(srcImage, detectKeyPoint, keyPointImage, Scalar(0, 0, 255), DrawMatchesFlags::DEFAULT);
		

		imshow("src image", srcImage);
		imshow("keyPoint image2", keyPointImage);

		int k = waitKey(1);
		if (k == 27) break;   //ESC
	}
	return 0;
}
*/



#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"


#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;

static void help()
{
	// print a welcome message, and the OpenCV version
	cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
		"Using OpenCV version " << CV_VERSION << endl;
	cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
	cout << "\nHot keys: \n"
		"\tESC - quit the program\n"
		"\tr - auto-initialize tracking\n"
		"\tc - delete all the points\n"
		"\tn - switch the \"night\" mode on/off\n"
		"To add/remove a feature point click it\n" << endl;
}

Point2f point;
bool addRemovePt = false;

static void onMouse(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		point = Point2f((float)x, (float)y);
		addRemovePt = true;
	}
}

int main(int argc, char** argv)
{
	help();

	VideoCapture cap;
	TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
	Size subPixWinSize(10, 10), winSize(31, 31);

	const int MAX_COUNT = 500;
	bool needToInit = false;
	bool nightMode = false;

	if (argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
		cap.open(argc == 2 ? argv[1][0] - '0' : 0);
	else if (argc == 2)
		cap.open(argv[1]);

	if (!cap.isOpened())
	{
		cout << "Could not initialize capturing...\n";
		return 0;
	}

	namedWindow("LK Demo", 1);
	setMouseCallback("LK Demo", onMouse, 0);

	Mat gray, prevGray, image;
	vector<Point2f> points[2]; // 0 for prev, 1 for next

	Ptr<FastFeatureDetector> m_fastDetector = FastFeatureDetector::create();
	vector<KeyPoint> keyPoints;

	for (;;)
	{
		Mat frame;
		cap >> frame;
		if (frame.empty())
			break;

		frame.copyTo(image);
		cvtColor(image, gray, COLOR_BGR2GRAY);

		Mat tmp_image;
		GaussianBlur(image, tmp_image, Size(5, 5), 0, 0); //  Gauss

		image = tmp_image;

		if (nightMode)
			image = Scalar::all(0);

		if (needToInit)
		{
			// automatic initialization
			//goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, false, 0.04);
			//cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);

			m_fastDetector->detect(gray, keyPoints);
			KeyPoint::convert(keyPoints, points[1]);

			addRemovePt = false;
		}
		else if (!points[0].empty())
		{
			vector<uchar> status;
			vector<float> err;
			if (prevGray.empty())
				gray.copyTo(prevGray);
			calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
				3, termcrit, 0, 0.001);
			size_t i, k;
			for (i = k = 0; i < points[1].size(); i++)
			{
				if (addRemovePt)
				{
					if (norm(point - points[1][i]) <= 5)
					{
						addRemovePt = false;
						continue;
					}
				}

				if (!status[i])
					continue;

				points[1][k++] = points[1][i];
				circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
			}
			points[1].resize(k);
		}

		if (addRemovePt && points[1].size() < (size_t)MAX_COUNT)
		{
			vector<Point2f> tmp;
			tmp.push_back(point);
			cornerSubPix(gray, tmp, winSize, cvSize(-1, -1), termcrit);
			points[1].push_back(tmp[0]);
			addRemovePt = false;
		}

		needToInit = false;
		imshow("LK Demo", image);

		char c = (char)waitKey(10);
		if (c == 27)
			break;
		switch (c)
		{
		case 'r':
			needToInit = true;
			break;
		case 'c':
			points[0].clear();
			points[1].clear();
			break;
		case 'n':
			nightMode = !nightMode;
			break;
		}

		std::swap(points[1], points[0]);
		cv::swap(prevGray, gray);
	}

	return 0;
}
