#include <opencv2/opencv.hpp>
#include <vector>

using p64=std::pair<int,int>;
template<typename T>
using Matrix=std::vector<std::vector<T>>;

std::vector<p64> bresenham4(int x0, int y0, int x1, int y1)
{
    std::vector<p64> points;
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sgnX = x0 < x1 ? 1 : -1;
    int sgnY = y0 < y1 ? 1 : -1;
    int e = 0;
    for (int i = 0; i <= dx + dy; ++i)
    {
        points.push_back(std::make_pair(x0, y0));
        int e1 = e + dy;
        int e2 = e - dx;
        if (abs(e1) < abs(e2))
        {
            x0 += sgnX;
            e = e1;
        }
        else
        {
            y0 += sgnY;
            e = e2;
        }
    }
    return points;
}

int main()
{
    cv::Mat final(256,256,CV_8UC3,cv::Scalar(255,255,255));

    p64 p1={50,50};
    p64 p2={200,200};

    std::vector<p64> path=bresenham4(p1.first,p1.second,p2.first,p2.second);

    for(const auto&p:path)
    {
        final.at<cv::Vec3b>(p.first,p.second)=cv::Vec3b(255,0,0);
    }

    std::vector<p64> path1=bresenham4(path[10].first,path[10].second,path[290].first,path[290].second);
    std::vector<p64> path2=bresenham4(path[20].first,path[20].second,path[280].first,path[280].second);

    for(const auto&p:path1)
    {
        final.at<cv::Vec3b>(p.first,p.second)=cv::Vec3b(0,255,0);
    }

    for(const auto&p:path2)
    {
        final.at<cv::Vec3b>(p.first,p.second)=cv::Vec3b(0,0,255);
    }

    cv::imshow("final",final);
    cv::waitKey();

    return 0;

}

