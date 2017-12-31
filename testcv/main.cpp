#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <deque>
#include <iostream>
#include <ctype.h>
#include <windows.h>
#include <string>
#include <stdio.h>

using namespace cv;
using namespace std;

#define PI 3.1415926

deque<vector<Point2f>> windows;
const int MAX_WINDOWS_SIZE = 60;
const int UPDATE_CYCLE = 120;
const double MIN_DISTANCE = 3;
const int circle_num = 4;
deque<Point2f> track_circle[circle_num];
bool clockwise[circle_num] = { true, false, true, false };
double phase0[circle_num] = { .0, .0, PI, PI };
Point2f m_point;

const double TH_IN = 0.05;
struct Action {
	/*
	0 do nothing
	1 play/pause
	2 set progress
		0 safe zone
		1 warning zone
		2 close
	3 set sound
		0 safe zone
		1 warning zone
		2 close
	4 press up
	5 press down
	*/
	int action_type, sub_action_type;
	double value;
};

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

static void onMouse(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		printf("x: %f y: %f\n",(float)x,(float)y);
	}
}


Point2f get_coordinate(int i, double t)
{
	double arc = phase0[i] + t * PI;
	return Point2f((float)cos(arc), (float)sin(arc));
}

bool acceptTrackedPoint(int i)
{
	double cnt = 0;
	for (int j = 1; j < windows.size(); ++j)
		cnt += (abs(windows.at(j)[i].x - windows.at(j - 1)[i].x) + abs(windows.at(j)[i].y - windows.at(j - 1)[i].y));
	if (windows.size() <= 1) return false;

	cnt = cnt / (windows.size() - 1);
	if (cnt > MIN_DISTANCE)
		return true; // 说明在移动
	return false;
}

template<class InputIt1, class InputIt2>
double pearson(InputIt1 firstX, InputIt2 firstY, int n) {
	double xy_sum = inner_product(firstX, firstX + n, firstY, 0);
	double x2_sum = inner_product(firstX, firstX + n, firstX, 0);
	double y2_sum = inner_product(firstY, firstY + n, firstY, 0);
	double x_sum = accumulate(firstX, firstX + n, 0);
	double y_sum = accumulate(firstY, firstY + n, 0);

	double deno = sqrt((x2_sum - 1.0 * pow(x_sum, 2) / n)*(y2_sum - 1.0 * pow(y_sum, 2) / n));
	return (xy_sum - 1.0 * x_sum * y_sum / n) / deno;
}

double set_escape(Point2f point)
{
	double value = point.y - m_point.y;
	return abs(value);
}

double set_bar(Point2f point)
{
	double value = point.x - m_point.x;
	return value;
}

/*
if (!status[0])
	action.sub_action_type = 2;
double point_y = set_escape(point)
if (point_y > threshold_escape)
	action.sub_action_type = 2;
else
{
	action.sub_action_type = 0 if (point_y < threshold_warning) else 1;
	action.value = set_bar(points[1][0]);
}
*/

void print_point(Point2f point)
{
	printf("%f %f\n", point.x, point.y);
}

struct CircleData
{
	Point2f center;
	double radius;

	void print()
	{
		print_point(center);
		printf("%f\n", radius);
	}
};

CircleData findCircle(Point2f pt1, Point2f pt2, Point2f pt3)      //应该没问题
{
	//浠わ細
	//A1 = 2 * pt2.x - 2 * pt1.x      B1 = 2 * pt1.y - 2 * pt2.y       C1 = pt1.y虏 + pt2.x虏 - pt1.x虏 - pt2.y虏
	//A2 = 2 * pt3.x - 2 * pt2.x      B2 = 2 * pt2.y - 2 * pt3.y       C2 = pt2.y虏 + pt3.x虏 - pt2.x虏 - pt3.y虏
	float A1, A2, B1, B2, C1, C2, temp;
	A1 = pt1.x - pt2.x;
	B1 = pt1.y - pt2.y;
	C1 = (pow(pt1.x, 2) - pow(pt2.x, 2) + pow(pt1.y, 2) - pow(pt2.y, 2)) / 2;
	A2 = pt3.x - pt2.x;
	B2 = pt3.y - pt2.y;
	C2 = (pow(pt3.x, 2) - pow(pt2.x, 2) + pow(pt3.y, 2) - pow(pt2.y, 2)) / 2;
	//涓轰簡鏂逛究缂栧啓绋嬪簭锛屼护temp = A1*B2 - A2*B1
	temp = A1 * B2 - A2 * B1;
	//瀹氫箟涓�涓渾鐨勬暟鎹殑缁撴瀯浣撳璞D
	CircleData CD;
	//鍒ゆ柇涓夌偣鏄惁鍏辩嚎
	if (temp == 0) {
		//鍏辩嚎鍒欏皢绗竴涓偣pt1浣滀负鍦嗗績
		CD.center.x = pt1.x;
		CD.center.y = pt1.y;
	}
	else {
		//涓嶅叡绾垮垯姹傚嚭鍦嗗績锛?
		//center.x = (C1*B2 - C2*B1) / A1*B2 - A2*B1;
		//center.y = (A1*C2 - A2*C1) / A1*B2 - A2*B1;
		CD.center.x = (C1*B2 - C2 * B1) / temp;
		CD.center.y = (A1*C2 - A2 * C1) / temp;
	}

	CD.radius = sqrtf((CD.center.x - pt1.x)*(CD.center.x - pt1.x) + (CD.center.y - pt1.y)*(CD.center.y - pt1.y));
	// maybe should be changed
	return CD;
}

double get_distance(Point2f point1, Point2f point2)
{
	return sqrtf((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));
}

vector<int> get_rand_points(int max) // int i,     // max >= 3;    // 应该没问题
{
	vector<int> points;
	int flag, tmp;
	for (int i = 0; i < 3; ++i) // 濡傛灉鍙傛暟鍒楄〃涓湁i锛屽垯瑕佹妸i鍙樻垚j
	{
		flag = 0;
		tmp = rand() % max + 1;
		for (int j = 0; j < i; ++j)
		{
			if (tmp == points[j])
			{
				flag = 1;
				break;
			}
		}
		if (flag == 1)
			i--;
		else
			points.push_back(tmp); //points[i] = tmp;
	}

	return points;
}

bool is_circle(int i)
{
	// max = size;
	int size = windows.size();
	//printf("%d\n", size);
	if (size < 60) return false;
	int max = 59;

	vector<int> points = get_rand_points(10);      // 0-59
	CircleData circle_data1 = findCircle(windows.at(points[0])[i], windows.at(points[1])[i], windows.at(points[2])[i]);

	points = get_rand_points(10);
	CircleData circle_data2 = findCircle(windows.at(points[0])[i], windows.at(points[1])[i], windows.at(points[2])[i]);

	points = get_rand_points(10);
	CircleData circle_data3 = findCircle(windows.at(points[0])[i], windows.at(points[1])[i], windows.at(points[2])[i]);

	//circle_data1.print();
	//circle_data2.print();
	//circle_data3.print();


	if (circle_data1.radius > 200)  return false;
	for (int j = 0; j < 60; ++j)
	{
		if ((get_distance(windows.at(size - j - 1)[i], circle_data1.center) >((1 + TH_IN) * circle_data1.radius)) || (get_distance(windows.at(size - j - 1)[i], circle_data1.center) < ((1 - TH_IN) * circle_data1.radius)))
			return false;
	}

	if (circle_data2.radius > 200) return false;
	for (int j = 0; j < 60; ++j)
	{
		if ((get_distance(windows.at(size - j - 1)[i], circle_data2.center) >((1 + TH_IN) * circle_data2.radius)) || (get_distance(windows.at(size - j - 1)[i], circle_data2.center) < ((1 - TH_IN) * circle_data2.radius)))
			return false;
	}

	if (circle_data3.radius > 200) return false;
	for (int j = 0; j < 60; ++j)
	{
		if ((get_distance(windows.at(size - j - 1)[i], circle_data3.center) >((1 + TH_IN) * circle_data3.radius)) || (get_distance(windows.at(size - j - 1)[i], circle_data3.center) < ((1 - TH_IN) * circle_data3.radius)))
			return false;
	}
	//circle_data1.print();
	//circle_data1.print();
	//circle_data2.print();
	//circle_data3.print();

	//printf("\n");
	//for (int j = 0; j < max; ++j)
	//{
	//	printf("%f %f\n", windows.at(size - j - 1)[i].x, windows.at(size - j - 1)[i].y);
	//}
	//cout << circle_data.center << endl;
	//cout << circle_data.radius << endl;
	//printf("%d\n", size);
	//printf("%f\n", circle_data.radius);
	return true;
}

int main(int argc, char** argv)
{
	help();
	
	double start = (double)getTickCount();
	VideoCapture cap;
	TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
	Size  winSize(51, 51);
	//Size subPixWinSize(10, 10);

	const int MAX_COUNT = 500;
	bool needToInit = true;
	bool nightMode = false;
	const int threshold = 20;

	char fps_string[10];

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

	Ptr<FastFeatureDetector> m_fastDetector = FastFeatureDetector::create(threshold);
	vector<KeyPoint> keyPoints;

	double t = 0;
	double now = 0;
	double fps = 0;
	size_t i;
	for (int cnt = 0;; cnt++)
	{
		t = (double)getTickCount();

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

			windows.clear();
			for (i = 0; i < circle_num; i++)
				track_circle[i].clear();
		}
		else if (!points[0].empty())
		{
			vector<uchar> status;
			vector<float> err;

			if (prevGray.empty())
				gray.copyTo(prevGray);
			calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
				3, termcrit, 0, 0.001);
			windows.push_back(points[1]);
			now = ((double)getTickCount() - start) / getTickFrequency();
			for (i = 0; i < circle_num; i++)
			{
				track_circle[i].push_back(get_coordinate((int)i,now));
				if (track_circle[i].size() > MAX_WINDOWS_SIZE)
					track_circle[i].pop_front();
				if (i == 0)
					circle(image, track_circle[i].back(), 3, Scalar(0, 0, 255), -1, 8);
			}
			if (windows.size() > MAX_WINDOWS_SIZE)
				windows.pop_front();

			for (i = 0; i < points[1].size(); i++)
			{
				if (!status[i]) //missing
					continue;
				if (!acceptTrackedPoint(int(i)))  //relative stable --> non-candidate
					circle(image, points[1][i], 3, Scalar(255, 0, 0), -1, 8);
				else
				{
					if (is_circle(i))
					{
						circle(image, points[1][i], 3, Scalar(0, 0, 255), -1, 8);
						printf("a circle has been find\n");
					}
					else
						circle(image, points[1][i], 3, Scalar(0, 255, 0), -1, 8);
				}

			}
		}

		needToInit = false;
		if (cnt == UPDATE_CYCLE)
		{
			needToInit = true;
			cnt = 0;
		}

		t = ((double)getTickCount() - t) / getTickFrequency();
		fps = 1.0 / t;
		sprintf(fps_string, "%.2f", fps);

		string fps_string_show("fps:");
		fps_string_show += fps_string;
		putText(image, fps_string_show,
			cv::Point(5, 20),           // 文字坐标，以左下角为原点
			cv::FONT_HERSHEY_SIMPLEX,   // 字体类型
			0.5, // 字体大小
			cv::Scalar(0, 0, 0));       // 字体颜色

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

