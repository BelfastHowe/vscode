// helloworld.cpp
#include <iostream>
#include <vector>
#include <set>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

using i16=int16_t;

int main()
{
	std::cout << "Hello, world!" << std::endl;
	//return 0;

	
	

	
	return 0;

}


std::vector<std::pair<int, int>> createSingleContour(
    std::vector<std::pair<int, int>>& outline,
    std::vector<std::vector<std::pair<int,int>>>& obstacles) {

    // 定义距离平方计算函数
    auto distSq = [](const std::pair<int, int>& p1, const std::pair<int, int>& p2) {
        return (p1.first - p2.first) * (p1.first - p2.first) + (p1.second - p2.second) * (p1.second - p2.second);
    };

    // 用于存储结果的列表
    std::vector<std::pair<int, int>> result = outline;
    std::set<std::pair<int, int>> forbidden_points; // 禁区点集

    // 处理每个内部孔洞
    for (const auto& obstacle : obstacles) {
        long long min_dist = LLONG_MAX;
        std::pair<int, int> closest_outline_point;
        std::pair<int, int> closest_obstacle_point;

        // 寻找外部轮廓和内部孔洞之间的最近点对
        for (const auto& opoint : result) {
            if (forbidden_points.find(opoint) != forbidden_points.end()) continue; // 跳过禁区点
            for (const auto& hpoint : obstacle) {
                long long dist = distSq(opoint, hpoint);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_outline_point = opoint;
                    closest_obstacle_point = hpoint;
                }
            }
        }

        // 将最近点对添加到禁区
        forbidden_points.insert(closest_outline_point);
        forbidden_points.insert(closest_obstacle_point);

        // 找到外部轮廓中最近点的位置
        auto it_outline_closest = std::find(result.begin(), result.end(), closest_outline_point);

        // 在孔洞轮廓中找到最近点位置
        auto it_hole_closest = std::find(obstacle.begin(), obstacle.end(), closest_obstacle_point);

        // 创建一个新的轮廓，包括到外部轮廓最近点的部分，孔洞轮廓的部分，以及返回到最近点的部分
        std::vector<std::pair<int, int>> new_result;
        new_result.insert(new_result.end(), result.begin(), it_outline_closest + 1);
        new_result.insert(new_result.end(), it_hole_closest, obstacle.end());
        new_result.insert(new_result.end(), obstacle.begin(), it_hole_closest + 1);
        
        new_result.insert(new_result.end(), it_outline_closest, result.end());

        // 更新结果为新的轮廓
        result = new_result;
    }

    // 返回最终合成的轮廓
    return result;
}


void waterSeg(cv::Mat&DT)
{
    for(int y=1;y<DT.rows-1;y++)
    {
        for(int x=1;x<DT.cols-1;x++)
        {
            int min=INT_MAX;
            for(int k=-1;k<2;k++)
            {
                for(int r=-1;r<2;r++)
                {
                    int xx=DT.at<uchar>(y+k,x+r)+abs(k)+abs(r);
                    if(xx<min) min=xx;
                }
            }

            DT.at<uchar>(y,x)=min;
        }
    }


    for(int y=DT.rows-2;y>1;y--)
    {
        for(int x=DT.cols-2;x>1;x--)
        {
            int min=INT_MAX;
            for(int k=-1;k<2;k++)
            {
                for(int r=-1;r<2;r++)
                {
                    int xx=DT.at<uchar>(y+k,x+r)+abs(k)+abs(r);
                    if(xx<min) min=xx;
                }
            }

            DT.at<uchar>(y,x)=min;
        }
    }

}

