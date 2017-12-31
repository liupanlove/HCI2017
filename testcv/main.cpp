#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <deque>
#include <list>
#include <iostream>
#include <ctype.h>
#include <windows.h>
#include <string>
#include <stdio.h>
#include <numeric>

using namespace std;
using namespace cv;

#define PI 3.1415926

list<deque<Point2f>> windows;
const int MAX_WINDOWS_SIZE = 64;
const int MAX_POINT_SIZE = 512;
const int MAX_ERROR = 16;
const int UPDATE_CYCLE = 120;
const double MIN_DISTANCE = 3;
const double TH_IN = 0.3;
const double TH_CORR = 0.8;
const int TH_ESCAPE = 30;
const int TH_WARNING = 20;
const int CIRCLE_NUM = 4;
deque<Point2f> track_circle[CIRCLE_NUM];
bool clockwise[CIRCLE_NUM] = { true, false, true, false };
double phase0[CIRCLE_NUM] = { .0, .0, PI, PI };
Point2f m_point;
bool state_setting;

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

	Action(int _action_type) :action_type(_action_type){ sub_action_type = 0; value = .0; };
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
	arc = clockwise[i] ? arc : -arc;
	return Point2f(50+50*i+10*(float)cos(-arc), 50+10*(float)sin(-arc));
}

bool judgeTrackedPoint(deque<Point2f> points)
{
	double cnt = 0;
	size_t size = points.size();
	if (size <= 4) return true;

	for (int j = 1; j < size; ++j)
		cnt += (abs(points.at(j).x - points.at(j - 1).x) + abs(points.at(j).y - points.at(j - 1).y));
	cnt = cnt / (size - 1);
	if (cnt > MIN_DISTANCE)
		return true; // 说明在移动
	return false;
}

vector<bool> acceptTrackedPoint()
{
	vector<bool> ans;

	for (list<deque<Point2f>>::iterator iter = windows.begin(); iter != windows.end(); ++iter)
		ans.push_back(judgeTrackedPoint(*iter));
	return ans;
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

bool is_circle(deque<Point2f> deque_point)
{
	size_t size = deque_point.size();
	if (size < MAX_WINDOWS_SIZE) return false;

	vector<int> points = get_rand_points(10);
	CircleData circle_data = findCircle(deque_point[size - points[0] - 1], deque_point[size - points[1] - 1], deque_point[size - points[2] - 1]);

	if (circle_data.radius > 200)  return false;
	for (int j = 0; j < MAX_WINDOWS_SIZE; ++j)
	{
		if ((get_distance(deque_point[j], circle_data.center) >((1 + TH_IN) * circle_data.radius)) || (get_distance(deque_point[j], circle_data.center) < ((1 - TH_IN) * circle_data.radius)))
			return false;
	}
	return true;
}

vector<deque<Point2f>> get_circle()
{
	vector<deque<Point2f>> ans;
	for (list<deque<Point2f>>::iterator iter = windows.begin(); iter != windows.end(); ++iter)
		if (is_circle(*iter))
			ans.push_back(*iter);
		else
			ans.push_back(deque<Point2f>());
	return ans;
}

bool response_comparator(const KeyPoint& p1, const KeyPoint& p2) {
	return p1.response > p2.response;
}

int main(int argc, char** argv)
{
	help();
	
	double start = (double)getTickCount();
	VideoCapture cap;
	TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
	Size  winSize(51, 51);
	//Size subPixWinSize(10, 10);

	//const int MAX_COUNT = 500;
	bool nightMode = false;

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

	Ptr<FastFeatureDetector> m_fastDetector = FastFeatureDetector::create(20);
	vector<KeyPoint> keyPoints;
	vector<Point2f> points_key;

	double t = 0, now = 0, fps = 0;
	size_t i;
	points[0].clear(), points[1].clear();
	for (state_setting = false;;)
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

		vector<uchar> status;
		vector<float> err;
		if (!state_setting)
		{
			// automatic initialization
			//goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, false, 0.04);
			//cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);

			m_fastDetector->detect(gray, keyPoints);
			std::sort(keyPoints.begin(), keyPoints.end(), response_comparator);
			KeyPoint::convert(keyPoints, points_key);
			int new_size = (int)points_key.size();
			if (points[0].size() + new_size > MAX_POINT_SIZE)
				new_size = MAX_POINT_SIZE - (int)points[0].size();
			points[0].insert(points[0].end(), points_key.begin(), points_key.begin() + new_size);

			if (prevGray.empty())
				gray.copyTo(prevGray);
			calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
				3, termcrit, 0, 0.001);

			i = 0;
			points[0].clear();
			vector<bool> accept = acceptTrackedPoint();
			for (auto iter = windows.begin(); iter != windows.end();)
			{
				if (!status[i] || !accept[i] || err[i] > MAX_ERROR)
				{
					iter = windows.erase(iter);
					i++;
					continue;
				}
				if (iter->size() == MAX_WINDOWS_SIZE)
					iter->pop_front();
				iter->push_back(points[1][i]);
				points[0].push_back(points[1][i]);
				i++;
				iter++;
			}
			while (i < points[1].size())
			{
				deque<Point2f> que_point(1, points[1][i]);
				windows.push_back(que_point);
				points[0].push_back(points[1][i]);
				i++;
			}

			now = ((double)getTickCount() - start) / getTickFrequency();
			for (i = 0; i < CIRCLE_NUM; i++)
			{
				track_circle[i].push_back(get_coordinate((int)i, now));
				if (track_circle[i].size() > MAX_WINDOWS_SIZE)
					track_circle[i].pop_front();
				if (i == 0 || i == 1)
					circle(image, track_circle[i].back(), 3, Scalar(0, 0, 255), -1, 8);
			}

			vector<deque<Point2f>> m_circles = get_circle();
			vector<float> circle_x[CIRCLE_NUM], circle_y[CIRCLE_NUM];
			for (i = 0; i < CIRCLE_NUM; i++)
				for (auto j = track_circle[i].begin(); j != track_circle[i].end(); j++)
				{
					circle_x[i].push_back(j->x);
					circle_y[i].push_back(j->y);
				}
			for (i = 0; i < points[0].size(); i++)
			{

				if (m_circles[i].empty()) //no circle
					circle(image, points[0][i], 3, Scalar(255, 0, 0), -1, 8);
				else
				{
					vector<float> m_x, m_y;
					for (auto j = m_circles[i].begin(); j != m_circles[i].end(); j++)
					{
						m_x.push_back(j->x);
						m_y.push_back(j->y);
					}
					bool flag = false;
					for (int j = 0; j < CIRCLE_NUM; j++)
						if (pearson(m_x.begin(), circle_x[j].begin(), MAX_WINDOWS_SIZE) > TH_CORR
							&& pearson(m_y.begin(), circle_y[j].begin(), MAX_WINDOWS_SIZE) > TH_CORR)
						{
							flag = true;
							printf("CIRCLE %d !!\n", j);
							circle(image, points[0][i], 3, Scalar(0, 0, 255), -1, 8);
							if (j >= 2)
							{
								state_setting = true;
								m_point = points[0][i];
								points[0].clear();
								points[0].push_back(m_point);
								windows.clear();
								deque<Point2f> que_point(1, m_point);
								windows.push_back(que_point);
							}
							break;
						}
					if (!flag)
						circle(image, points[0][i], 3, Scalar(0, 255, 0), -1, 8);
				}
				if (state_setting)
					break;
			}
		}
		else
		{
			Action action(2);
			calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
				3, termcrit, 0, 0.001);
			if (!status[0])
			{
				action.sub_action_type = 2;
				state_setting = false;
				points[0].clear();
				windows.clear();
			}
			else
			{
				double point_y = set_escape(points[1].front());
				if (point_y > TH_ESCAPE)
				{
					action.sub_action_type = 2;
					state_setting = false;
					points[0].clear();
					windows.clear();
				}
				else
				{
					action.sub_action_type = point_y < TH_WARNING ? 0 : 1;
					action.value = set_bar(points[1][0]);
					circle(image, points[1][0], 3, Scalar(0, 0, 255), -1, 8);
					std::swap(points[0], points[1]);
				}
			}
		}

		t = ((double)getTickCount() - t) / getTickFrequency();
		fps = 1.0 / t;
		sprintf(fps_string, "%.2f", fps);

		string fps_string_show("fps:");
		fps_string_show += fps_string;
		putText(image, fps_string_show,
			Point(5, 20),           // 文字坐标，以左下角为原点
			FONT_HERSHEY_SIMPLEX,   // 字体类型
			0.5, // 字体大小
			Scalar(0, 0, 0));       // 字体颜色

		imshow("LK Demo", image);

		char c = (char)waitKey(10);
		if (c == 27)
			break;
		switch (c)
		{
		case 'c':
			points[0].clear();
			points[1].clear();
			break;
		case 'n':
			nightMode = !nightMode;
			break;
		}

		cv::swap(prevGray, gray);
	}

	return 0;
}

