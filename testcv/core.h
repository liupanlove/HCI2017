#ifndef CORE_H
#define CORE_H

#include <QPixmap>
#include <opencv2/opencv.hpp>
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

/*
    圆圈的属性，依次为角速度(rad/s),方向(0/1，0表示顺时针)和初始相位(0~2pi)
*/
struct Round {
    double speed, phase;
    bool direction;
};

struct Action {
    /*
    ///0 do nothing
    1 play/pause   1
    2 set progress  2
        0 safe zone
        1 warning zone
        2 close
    3 set sound     3
        0 safe zone
        1 warning zone
        2 close
    4 press up     0     
    5 press down   1
    */
    int action_type, sub_action_type;
    double value;

    Action(int _action_type) :action_type(_action_type){ sub_action_type = 0; value = .0; }
};


class Core {
public:
    /*
        程序启动时前端会依次请求五种圆，后端依次返回五种圆的属性并记录
    */
    const double PI = 3.1415926;
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
    bool clockwise[4];// = { true, false, true, false, true, false};
    double phase0[4];// = { .0, .0, PI, PI, PI / 2, PI / 2};

    list<deque<Point2f>> windows;
    deque<Point2f> track_circle[4];
    Point2f m_point;
    bool state_setting;

    vector<Point2f> points[2];

    const int PLAY_PAUSE = 1;
    const int PROGRESS = 2;
    const int SOUND = 3;
    const int PAGE_UP = 4;
    const int PAGE_DOWN = 5;

    double start;
    TermCriteria * termcrit;
    char fps_string[10];
    Ptr<FastFeatureDetector> m_fastDetector;

    vector<KeyPoint> keyPoints;
    vector<Point2f> points_key;

    Action * action;
    int state_num;
    int state_cnt;


    Core()
    {
        //clockwise[6] = { true, false, true, false, true, false};
        //phase0[6] = { .0, .0, PI, PI, PI / 2, PI / 2};
        clockwise[0] = true;
        clockwise[1] = false;
        clockwise[2] = true;
        clockwise[3] = false;
       // clockwise[4] = true;
       // clockwise[5] = false;

        phase0[0] = .0;
        phase0[1] = .0;
        phase0[2] = PI;
        phase0[3] = PI;
      //  phase0[4] = PI / 2;
      //  phase0[5] = PI / 2;

        state_setting = false;
        state_num = 0;
        state_cnt = 0;

        termcrit = new TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03); ////////////


        m_fastDetector = FastFeatureDetector::create(20);

        action = new Action(0);
        points[0].clear(), points[1].clear();
    }

    Round get_round(int type)  /* 1-5 */
    {
        Round tmp;
        tmp.direction = clockwise[type % 4];
        tmp.phase = phase0[type % 4];
        tmp.speed = PI;
        return tmp;
    }
    /*
        前端每录制到一帧都会把录制到的帧发往后端，后端需要通过分析这一帧及之前记录的状态来返回前端需要执行的操作（如翻页）。
    */
    Action process(Mat img)
    {
        Action tmp(0);
        Action ans = judge_process(img);
        if((ans.action_type != 0) && state_cnt < 30)
        {
            state_cnt = 0;
            return ans;
        }
        else
        {
            state_cnt ++;
            return tmp;
        }
    }


    Point2f get_coordinate(int i, double t) //
    {
        double arc = phase0[i] + t * PI;
        arc = clockwise[i] ? arc : -arc;
        return Point2f(50+50*i+10*(float)cos(-arc), 50+10*(float)sin(-arc));
    }

    bool judgeTrackedPoint(deque<Point2f> points)
    {
        double cnt = 0;
        int size = points.size();
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

        //void print()
        //{
         //   print_point(center);
         //   printf("%f\n", radius);
        //}
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
        //瀹氫箟涓€涓渾鐨勬暟鎹殑缁撴瀯浣撳璞D
        CircleData CD;
        //鍒ゆ柇涓夌偣鏄惁鍏辩嚎
        if (temp == 0) {
            //鍏辩嚎鍒欏皢绗竴涓偣pt1浣滀负鍦嗗績
            CD.center.x = pt1.x;
            CD.center.y = pt1.y;
        }
        else {
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
        for (int i = 0; i < 3; ++i)
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
        //printf("haipa\n");
        int size = deque_point.size();
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

    static bool response_comparator(const KeyPoint& p1, const KeyPoint& p2) {
        return p1.response > p2.response;
    }

    Action judge_process(Mat img)
    {
        action->action_type = 0;
        action->sub_action_type = 0;
        action->value = 0;


        start = (double)getTickCount();
        Size winSize(51, 51);          /////////////

        Mat gray, prevGray, image;
        double now = 0;//, fps = 0;
        int i;

        img.copyTo(image);
        cvtColor(image, gray, COLOR_BGR2GRAY);

        Mat tmp_image;
        GaussianBlur(image, tmp_image, Size(5, 5), 0, 0); //  Gauss

        image = tmp_image;


        vector<uchar> status;
        vector<float> err;
        if (!state_setting)
        {

            m_fastDetector->detect(gray, keyPoints);
            std::sort(keyPoints.begin(), keyPoints.end(), response_comparator);
            KeyPoint::convert(keyPoints, points_key);

            int new_size = (int)points_key.size();
            if (points[0].size() + new_size > MAX_POINT_SIZE)
                new_size = MAX_POINT_SIZE - (int)points[0].size();
            points[0].insert(points[0].end(), points_key.begin(), points_key.begin() + new_size);

            if (prevGray.empty())
                gray.copyTo(prevGray);
            if (!points[0].empty())
				calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
					3, termcrit, 0, 0.001);
			else
				points[1].clear();

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
                //if (i == 0 || i == 1)
                    //circle(image, track_circle[i].back(), 3, Scalar(0, 0, 255), -1, 8);
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

                //if (m_circles[i].empty()) //no circle
                    //circle(image, points[0][i], 3, Scalar(255, 0, 0), -1, 8);
                if(!m_circles[i].empty())
                {
                    vector<float> m_x, m_y;
                    for (auto j = m_circles[i].begin(); j != m_circles[i].end(); j++)
                    {
                        m_x.push_back(j->x);
                        m_y.push_back(j->y);
                    }
                    //bool flag = false;
                    for (int j = 0; j < CIRCLE_NUM; j++)
                        if (pearson(m_x.begin(), circle_x[j].begin(), MAX_WINDOWS_SIZE) > TH_CORR
                            && pearson(m_y.begin(), circle_y[j].begin(), MAX_WINDOWS_SIZE) > TH_CORR)
                        {
                            //flag = true;
                            //printf("CIRCLE %d !!\n", j);
                            //circle(image, points[0][i], 3, Scalar(0, 0, 255), -1, 8);
							if(j == 0)
								j = 4;
							action->action_type = j;
							
                            qDebug("Get Round #%d\n",j);
                            if (j == 2 || j == 3)
                            {
                                state_setting = true;

                                state_num = j;
                                m_point = points[0][i];
                                points[0].clear();
                                points[0].push_back(m_point);
                                windows.clear();
                                deque<Point2f> que_point(1, m_point);
                                windows.push_back(que_point);
                            }
                            break;
                        }
                    //if (!flag)
                        //circle(image, points[0][i], 3, Scalar(0, 255, 0), -1, 8);
                }
                if (state_setting)
                    break;
            }
        }
        else
        {
            action->action_type = state_num;

            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                3, *termcrit, 0, 0.001);
            if (!status[0])
            {
                action->sub_action_type = 2;
                state_setting = false;
                points[0].clear();
                windows.clear();
            }
            else
            {
                double point_y = set_escape(points[1].front());
                if (point_y > TH_ESCAPE)
                {
                    action->sub_action_type = 2;
                    state_setting = false;
                    points[0].clear();
                    windows.clear();
                }
                else
                {
                    action->sub_action_type = point_y < TH_WARNING ? 0 : 1;
                    action->value = set_bar(points[1][0]);
                    //circle(image, points[1][0], 3, Scalar(0, 0, 255), -1, 8);
                    std::swap(points[0], points[1]);
                }
            }
        }


        cv::swap(prevGray, gray);
        return *action;
    }
};


#endif // CORE_H
