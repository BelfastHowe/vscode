#include <opencv2/opencv.hpp>
#include <vector>
#include "relocation.h"
#include <Eigen/Dense>

using namespace std;
using namespace cv;

inline int Round(const float x)
{
    return static_cast<int>(std::floor(x + 0.5));
}

static void dda(uint8_t *omap, const uint8_t *pmap, int w, int h, int x0, int y0, int x1, int y1, int in_value, int out_value)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float adx = std::abs(dx);
    float ady = std::abs(dy);
    float step = adx >= ady ? adx : ady;
    dx = dx / step;
    dy = dy / step;
    float x = x0;
    float y = y0;
    int i = 1;
    int obstacle_count = 0;

    while(i<=step)
    {
        int xx = (int)x;
        int yy = (int)y;
        if (0 <= xx && xx < w && 0 <= yy && yy < h)
        {
            uint8_t &m = omap[xx + yy * w];
            uint8_t p = pmap[xx + yy * w];
            if (p < 40) obstacle_count++;
            if (obstacle_count < 4)
                m = in_value;
            else if (m == 0)
                m = out_value;
        }
        x += dx;
        y += dy;
        i++;
    }
}

bool Relocation::calc_score_with_dda(const std::unique_ptr<Grid2D> &ccg, const struct laser_ranges &laser_ranges, const Rigid2d &pose, float &score, int &debug_idx, float enable_debug)
{
    const ProbabilityGrid *probability_grid = dynamic_cast<const ProbabilityGrid *>(ccg.get());
    const cartographer::mapping::CellLimits &cell_limits = probability_grid->limits().cell_limits();
    const int &w = cell_limits.num_x_cells;
    const int &h = cell_limits.num_y_cells;
    const float &resolution = probability_grid->limits().resolution();
    const double &max_x = probability_grid->limits().max().x();
    const double &max_y = probability_grid->limits().max().y();
    const int origin_x = static_cast<int>(max_y / resolution);
    const int origin_y = static_cast<int>(max_x / resolution);
    const int &pose_x = static_cast<int>((max_y - pose.translation().y()) / resolution);
    const int &pose_y = static_cast<int>((max_x - pose.translation().x()) / resolution);
    const float &pose_theta = pose.normalized_angle();
    constexpr float noise_spec = 0.5f;

    std::vector<cv::Point> list;
    std::vector<cv::Point> laser_list;
    std::vector<cv::Point> cross_list;
    std::vector<cv::Point> laser_points;
    list.reserve(360);
    float sum=0;
    float sum_laser = 0;
    float sum_cross = 0;
    for (size_t i = 0; i < 360;++i)
    {
        float range = laser_ranges.ranges[i];
        if(range<ROBOT_RADIUS_M||range>6.0)
            continue;
        float theta = M_PI_F / 2.f + pose_theta - i * M_PI_F / 180.f;
        int tx = static_cast<int>(range * cos(theta) / resolution + pose_x);
        int ty = static_cast<int>(-range * sin(theta) / resolution + pose_y);
        laser_points.emplace_back(tx, ty);

        float dx = tx - pose_x;
        float dy = ty - pose_y;
        float steps = std::max(std::abs(dx), std::abs(dy));
        float xIncrement = dx / steps;
        float yIncrement = dy / steps;

        for (float x = pose_x, y = pose_y, k = 0;
             x < w && y < h && x > 0 && y > 0;
             x+=xIncrement,y+=yIncrement,k+=1)
             {
                Eigen::Array2i xy_index = {Round(x), Round(y)};

                float pro = probability_grid->IsKnown(xy_index) ? (1. - probability_grid->GetProbability(xy_index)) : 0.5;
                float dis_pose = std::hypot(x - pose_x, y - pose_y);
                if(pro<0.5&&dis_pose>ROBOT_RADIUS_M*2)
                {
                    if(k>steps)
                    {
                        cross_list.emplace_back(tx, ty);
                    }
                    else
                    {
                        cross_list.emplace_back(xy_index.x(), xy_index.y());
                    }
                    laser_list.emplace_back(tx, ty);
                    list.emplace_back(xy_index.x(), xy_index.y());
                    break;
                }
             }
    }

    if(list.empty()||cross_list.empty()||laser_list.empty())
    {
        score = 0;
        return false;
    }
    size_t size = list.size();
    for (size_t i = 0; i < size;i++)
    {
        int j = (i + 1) % size;
        sum += list[i].x * list[j].y - list[i].y * list[j].x;
        sum_laser += laser_list[i].x * laser_list[j].y - laser_list[i].y * laser_list[j].x;
        sum_cross += cross_list[i].x * cross_list[j].y - cross_list[i].y * cross_list[j].x;
    }
    sum = sum / 400 / 2;
    sum_laser = sum_laser / 400 / 2;
    sum_cross = sum_cross / 400 / 2;
    score = (sum_cross / (sum + sum_laser - sum_cross));

    if(enable_debug)
    {
        std::vector<uint8_t> omap;
        omap.reserve(w * h);
        for(const Eigen::Array2i&xy_index:cartographer::mapping::XYIndexRangeIterator(cell_limits))
        {
            CHECK(probability_grid->limits().Contains(xy_index));
            uint8_t prob =
                probability_grid->IsKnown(xy_index)
                    ? cartographer::common::RoundToInt((1. - probability_grid->GetProbality(xy_index)) * 255)
                    : 128;
            omap.emplace_back(prob);
        }
        cv::Mat draw;
        cv::Mat in(h, w, CV_8UC1, &omap[0]);
        cv::cvtColor(in, draw, CV_GRAY2BGR);
        cv::polylines(draw, list, true, cv::Scalar(255, 0, 0));
        cv::polylines(draw, laser_list, true, cv::Scalar(0, 255, 0));
        cv::polylines(draw, cross_list, true, cv::Scalar(0, 0, 255));
        cv::circle(draw, cv::Point(pose_x, pose_y), 3, cv::Scalar(0, 0, 0), -1);
        cv::imshow("draw" + std::to_string(debug_idx), draw);
    }
    return true;
}

bool Relocation::calc_matched_and_missed_area(const std::unique_ptr<Grid2D> &ccg, const struct laser_ranges &laser_ranges,const Rigid2d &pose, float &matched_area, float &missed_area, int &debug_index, float enable_debug)
{
    enable_debug = false;
    //1. dda to 360 points
    //2.dilated
    //3.check point not obstacle or free

    float pose_x = pose.translation().x();
    float pose_y = pose.translation().y();
    float pose_theta = pose.normalized_angle();
    const ProbabilityGrid *probability_grid = static_cast<const ProbabilityGrid *>(ccg.get());
    const cartographer::mapping::CellLimits &cell_limits = probability_grid->limits().cell_limits();
    const int w = cell_limits.num_x_cells;
    const int h = cell_limits.num_y_cells;
    const float resolution = probability_grid->limits().resolution();
    const double grid_max_x = probability_grid->limits().max().x();
    const double grid_max_y = probability_grid->limits().max().y();
    int cx = (int)((grid_max_y - pose_y) / resolution);
    int cy = (int)((grid_max_x - pose_x) / resolution);
    std::string debug_string = std::to_string(debug_index) + "-relo-global-";

    /*1*/
    std::vector<Eigen::Array2i> points;
    for (size_t i = 0; i < 360; ++i)
    {
        float range = laser_ranges.ranges[i];
        float theta = M_PI_F / 2.f + pose_theta - i * M_PI_F / 180.f;
        int tx = (int)((range * cos(theta)) / resolution);
        int ty = (int)((-range * sin(theta)) / resolution);
        Eigen::Array2i xy_index = {tx, ty};
        points.push_back(xy_index);
    }
    int min_x = std::numeric_limits<int>::max();
    int max_x = std::numeric_limits<int>::lowest();
    int min_y = std::numeric_limits<int>::max();
    int max_y = std::numeric_limits<int>::lowest();
    for (const auto &p : points)
    {
        float x = p[0];
        float y = p[1];
        if(min_x>x) min_x = x;
        if(max_x<x) max_x = x;
        if(min_y>y) min_y = y;
        if(max_y<y) max_y = y;
    }
    Eigen::Array2i origin = {-min_x, -min_y};
    for (auto &p : points)
    {
        p += origin;
    }
    int ww = max_x - min_x + 1;
    int hh = max_y - min_y + 1;
    int ox = origin[0];
    int oy = origin[1];
    std::vector<uint8_t> omap(ww * hh, 0);
    std::vector<uint8_t> pmap(ww * hh);
    for (int i = 0; i < hh; ++i)
    {
        for (int j = 0; j < ww; ++j)
        {
            int tx = (int)(cx + j - ox);
            int ty = (int)(cy + i - oy);
            Eigen::Array2i xy_index = {tx, ty};
            uint8_t v = probability_grid->limits().Contains(xy_index) ? cartographer::common::RoundToInt((1. - probability_grid->GetProbability(xy_index)) * 255) : 0;
            pmap[i * ww + j] = v;
        }
    }
    const uint8_t before_obs = 255;
    const uint8_t after_obs = m_auto_detect_level >= 2 ? 128 : 255;
    for (const auto &p : points)
    {
        int x = p[0];
        int y = p[1];
        dda(&omap[0], &pmap[0], ww, hh, ox, oy, p[0], p[1], before_obs, after_obs);
    }

    /*2*/
    GenericGrid<uint8_t> g(ww, hh, resolution, grid_coord_t(0, 0), omap);
    g.apply_if_in_radius(g.cell_size() * 1, IsVal(255), SetVal(255));
    g.apply_if_in_radius(g.cell_size() * 1, IsVal(128), SetVal(128));
    std::swap(omap, g.grid().cells);

    /*3*/
    for (int i = 0; i < hh; ++i)
    {
        for (int j = 0; j < ww; ++j)
        {
            if (omap[i * ww + j] == 255)
            {
                int tx = (int)(cx + j - ox);
                int ty = (int)(cy + i - oy);
                Eigen::Array2i xy_index = {tx, ty};
                bool is_free = probability_grid->limits().Contains(xy_index) &&
                               probability_grid->IsKnown(xy_index) &&
                               cartographer::common::RoundToInt((1. - probability_grid->GetProbability(xy_index)) * 255) > 148;
                omap[i * ww + j] = is_free ? 255 : 128;
            }
        }
    }

    matched_area = std::count(omap.begin(), omap.end(), 255) * resolution * resolution;
    missed_area = std::count(omap.begin(), omap.end(), 128) * resolution * resolution;

    if(m_auto_detect_level>=2)
    {
        const float min_ff_area = 0.f;
        if (matched_area > min_ff_area)
        {
            float total_area = matched_area + missed_area;
            int r = ROBOT_RADIUS_M / resolution;
            for (int y = oy - r; y <= oy + r; ++y)
            {
                for (int x = ox - r; x <= ox + r; ++x)
                {
                    if (0 <= x && x < ww && 0 <= y && y < hh)
                    {
                        omap[y * ww + x] = 255;
                    }
                }
            }
            FloodFill(&omap[0], ww, hh, ox, oy, 255, 64);
            float ff_match = std::count(omap.begin(), omap.end(), 64) * resolution * resolution;
            if (ff_match > min_ff_area)
            {
                matched_area = ff_match;
                missed_area = total_area - ff_match;
            }
        }
    }
    return true;
}

int check_global_match_result(const std::unique_ptr<Grid2D> &ccg, const struct laser_ranges &laser_ranges, const cartographer::sensor::PointCloud &point_cloud, const Rigid2d &pose, float score, float dda_score, bool is_just_check_result = false, bool enable_debug = false)
{
    //1.dilate map to find >0.76 cases,let them pass.<0.65 let it failed. between them, then check behind.
    //2.them cal the line info from laser, calc the coverage rate of laser in dilate line, check if over 70%, if yes, then failed. else, check behind
    //3.dilate map 5,5, check >0.8 if yes, pass. if no then check dda score. if dda >0.55 then pass.
    float pose_x = pose.translation().x();
    float pose_y = pose.translation().y();
    float pose_theta = pose.normalized_angle();
    bool matched = true;
    const ProbabilityGrid *probability_grid = static_cast<const ProbabilityGrid *>(ccg.get());
    const cartographer::mapping::CellLimits &cell_limits = probability_grid->limits().cell_limits();
    const int w = cell_limits.num_x_cells;
    const int h = cell_limits.num_y_cells;
    const float resolution = probability_grid->limits().resolution();
    const double grid_max_x = probability_grid->limits().max().x();
    const double grid_max_y = probability_grid->limits().max().y();
    int cx = (int)((grid_max_y - pose_y) / resolution);
    int cy = (int)((grid_max_x - pose_x) / resolution);
    constexpr float noise_spec = 0.025f;
    int ox = (int)(grid_max_y / resolution);
    int oy = (int)(grid_max_x / resolution);

    struct dda_info
    {
        cv::Point xy;
        std::vector<cv::Point> unknowns;
        std::vector<cv::Point> obss;
        bool is_outside_map;
    };

    if(matched)
    {
        //1.check matched points
        std::vector<uint8_t> omap;
        omap.reserve(w * h);
        for (const Eigen::Array2i &xy_index : cartographer::mapping::XYIndexRangeIterator(cell_limits))
        {
            CHECK(probability_grid->limits().Contains(xy_index));
            uint8_t prob =
                probability_grid->IsKnown(xy_index) ? cartographer::common::RoundToInt((1. - probability_grid->GetProbability(xy_index)) * 255) : 128;
            omap.emplace_back(prob);
        }
        cv::Mat mat_origin(h, w, CV_8UC1, &omap[0]);
        cv::copyMakeBorder(mat_origin, mat_origin, 2, 2, 2, 2, CV_HAL_BORDER_CONSTANT, cv::Scalar(128));

        //0.check dda in laser line to find unknown and obs point
        cv::Mat frame_show_dda = mat_origin.clone();
        frame_show_dda.setTo(0);

        cv::Point origin_p = cv::Point((grid_max_y - pose_y) / resolution, (grid_max_x - pose_x) / resolution) + cv::Point(2, 2);
        int inside_count = 0;
        int no_inside_count = 0;
        for (auto i : point_cloud)
        {
            dda_info tmp_dda_info;
            Eigen::Vector3f pc = i;
            float theta = M_PI_F / 2.f + pose_theta;
            float x = pc[0] * cos(theta) - pc[1] * sin(theta);
            float y = pc[0] * sin(theta) + pc[1] * cos(theta);
            std::tuple<float, float> xy_index = {x / resolution + (grid_max_y - pose_y) / resolution + 2, -y / resolution + (grid_max_x - pose_x) / resolution + 2};
            float x1 = std::get<0>(xy_index);
            float y1 = std::get<1>(xy_index);
            if (x1 >= 0 && x1 <= (frame_show_dda.cols - 1) && y1 >= 0 && y1 <= (frame_show_dda.rows - 1))
            {
                cv::line(frame_show_dda, origin_p, cv::Point(x1, y1), cv::Scalar(255));
            }
        }

        bitwise_and(frame_show_dda, mat_origin, frame_show_dda);
        Mat frame_show_dda_obs;
        Mat frame_show_dda_unk;
        cv::inRange(frame_show_dda, cv::Scalar(1), cv::Scalar(127), frame_show_dda_obs);
        cv::inRange(frame_show_dda, cv::Scalar(128), cv::Scalar(128), frame_show_dda_unk);

        vector<dda_info> vec_dda_info;
        for(auto i:point_cloud)
        {
            frame_show_dda.setTo(0);
            dda_info tmp_dda_info;
            Eigen::Vector3f pc = i;
            float theta = M_PI_F / 2.f + pose_theta;
            float x = pc[0] * cos(theta) - pc[1] * sin(theta);
            float y = pc[0] * sin(theta) + pc[1] * cos(theta);
            std::tuple<float, float> xy_index = {x / resolution + (grid_max_y - pose_y) / resolution + 2, -y / resolution + (grid_max_x - pose_x) / resolution + 2};
            float x1 = std::get<0>(xy_index);
            float y1 = std::get<1>(xy_index);
            tmp_dda_info.xy = cv::Point(x1, y1);
            if (x1 >= 0 && x1 <= (frame_show_dda.cols - 1) && y1 >= 0 && y1 <= (frame_show_dda.rows - 1))
            {
                cv::line(frame_show_dda, origin_p, Point(x1, y1), Scalar(255));
                Mat frame_show_dda_info_tmp;
                bitwise_and(frame_show_dda, mat_origin, frame_show_dda);
                cv::inRange(frame_show_dda, Scalar(1), Scalar(127), frame_show_dda_info_tmp);
                vector<Point> vev_tmo_p;
                findNonZero(frame_show_dda_info_tmp, vev_tmo_p);
                tmp_dda_info.obss = vev_tmo_p;
                vev_tmo_p.clear();

                cv::inRange(frame_show_dda, Scalar(128), Scalar(128), frame_show_dda_info_tmp);
                findNonZero(frame_show_dda_info_tmp, vev_tmo_p);
                tmp_dda_info.unknowns = vev_tmo_p;
                tmp_dda_info.is_outside_map = 0;
                vec_dda_info.push_back(tmp_dda_info);
            }
            else
            {
                tmp_dda_info.is_outside_map = 1;
                vec_dda_info.push_back(tmp_dda_info);
            }
        }
        float across_ratio = 0;
        int across_count = 0;
        for (const auto &i_d : vec_dda_info)
        {
            int tmp_count = 0;
            for (const auto &p_obs : i_d.obss)
            {
                if (hypot(p_obs.x - i_d.xy.x, p_obs.y - i_d.xy.y) > 3)
                {
                    tmp_count++;
                }
            }
            across_count = tmp_count >= 3 ? across_count + 1 : across_count;
        }
        across_ratio = (float)across_count / (float)vec_dda_info.size();
        if(!enable_debug)
        {
            frame_show_dda.release();
        }

        typedef struct
        {
            float x0, y0, x1, y1;
        } line_seg_t;
        typedef struct 
        {
            line_seg_t l;
            int count;
        } line_seg_ex_t;
        typedef struct
        {
            int id;
            float length;
            float theta;
            float distance;
        } line_segment_info_t;

        //2.coverage rate of laser in dilate line
        Mat frame_show_line = mat_origin.clone();
        frame_show_line.setTo(0);
        struct laser_ranges laser_ranges_tmp = laser_ranges;
        vector<line_seg_t> lines = extract_line_segment(laser_ranges_tmp.ranges, 0.25);
        vector<line_segment_info_t> line_info = calc_line_segment_info(lines);
        int index_line = 1;
        for (auto l : lines)
        {
            Eigen::Vector2f p1 = Eigen::Vector2f(l.x0, l.y0);
            Eigen::Vector2f p2 = Eigen::Vector2f(l.x1, l.y1);
            float theta = pose_theta;

            p1[0] = l.x0 * cos(theta) - l.y0 * sin(theta);
            p1[1] = l.x0 * sin(theta) + l.y0 * cos(theta);

            p2[0] = l.x1 * cos(theta) - l.y1 * sin(theta);
            p2[1] = l.x1 * sin(theta) + l.y1 * cos(theta);

            p1[0] = p1[0] / resolution + (grid_max_y - pose_y) / resolution;
            p1[1] = -p1[1] / resolution + (grid_max_x - pose_x) / resolution;

            p2[0] = p2[0] / resolution + (grid_max_y - pose_y) / resolution;
            p2[1] = -p2[1] / resolution + (grid_max_x - pose_x) / resolution;

            cv::line(frame_show_line, cv::Point(p1[0 + 2], p1[1] + 2), cv::Point(p2[0] + 2, p2[1] + 2), Scalar(255));
            index_line++;
        }
        cv::dilate(frame_show_line, frame_show_line, cv::getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
        int match_point_check1 = 0;
        int miss_point_check1 = 0;
        vector<tuple<float, float>> match_points;
        vector<tuple<float, float>> miss_points;
        for (auto i : point_cloud)
        {
            auto pc = i;
            float theta = M_PI_F / 2.f + pose_theta;
            float x = pc[0] * cos(theta) - pc[1] * sin(theta);
            float y = pc[0] * sin(theta) + pc[1] * cos(theta);
            std::tuple<float, float> xy_index = {x / resolution + (grid_max_y - pose_y) / resolution + 2, -y / resolution + (grid_max_x - pose_x) / resolution + 2};
            float x1 = std::get<0>(xy_index);
            float y1 = std::get<1>(xy_index);
            if (x1 >= 0 && x1 <= (frame_show_line.cols - 1) && y1 >= 0 && y1 <= (frame_show_line.rows - 1))
            {
                if (frame_show_line.at<uchar>(get<1>(xy_index), get<0>(xy_index)) == 255)
                {
                    match_point_check1++;
                    match_points.push_back(xy_index);
                }
                else
                {
                    miss_point_check1++;
                    miss_points.push_back(xy_index);
                }
            }
            else
            {
                miss_point_check1++;
            }
        }
        float ratio_match1 = (float)match_point_check1 / (float)(match_point_check1 + miss_point_check1);

        Mat draw_realtime;
        Mat in_realtime = mat_origin.clone();
        cvtColor(in_realtime, draw_realtime, COLOR_GRAY2BGR);
        Mat frame(h, w, CV_8UC1, &omap[0]);
        Mat frame_show;
        vector<tuple<float, float>> draw_points_match;
        vector<tuple<float, float>> draw_points_miss;
        cv::erode(in_realtime, in_realtime, cv::getStructuringElement(MORPH_ELLIPSE, cv::Size(3, 3)));
        int scale = 1;

        int match_point_check = 0;
        int miss_point_check = 0;
        for (auto i : point_cloud)
        {
            auto pc = i;
            float theta = M_PI_F / 2.f + pose_theta;
            float x = pc[0] * cos(theta) - pc[1] * sin(theta);
            float y = pc[0] * sin(theta) + pc[1] * cos(theta);
            std::tuple<float, float> xy_index = {x / resolution + (grid_max_y - pose_y) / resolution + 2, -y / resolution + (grid_max_x - pose_x) / resolution + 2};
            draw_points_match.push_back(xy_index);
            float x1 = std::get<0>(xy_index);
            float y1 = std::get<1>(xy_index);
            if (x1 >= 0 && x1 <= (in_realtime.cols - 1) && y1 >= 0 && y1 <= (in_realtime.rows - 1))
            {
                if (in_realtime.at<uchar>(get<1>(xy_index), get<0>(xy_index)) < 128)
                {
                    match_point_check++;
                }
                else
                {
                    miss_point_check++;
                }
            }
            else
            {
                miss_point_check++;
            }
        }
        float ratio_match = (float)match_point_check / (float)(match_point_check + miss_point_check);

        float thresh_1 = 0.75;
        if (ratio_match1 > 0.8)
        {
            thresh_1 = 0.85;
        }
        else if (ratio_match1 > 0.7)
        {
            thresh_1 = 0.8;
        }
        else
        {
            thresh_1 = 0.75;
        }
        if (ratio_match > thresh_1 && across_ratio < 0.4)
        {
            return 1;
        }
        if (ratio_match < 0.55)
        {
            return 0;
        }


        //3. dilate map 5,5, check result
        Mat mat_dilate_55 = mat_origin.clone();
        cv::erode(mat_dilate_55, mat_dilate_55, cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        match_point_check = 0;
        miss_point_check = 0;
        for (auto i : point_cloud)
        {
            auto pc = i;
            float theta = M_PI_F / 2.f + pose_theta;
            float x = pc[0] * cos(theta) - pc[1] * sin(theta);
            float y = pc[0] * sin(theta) + pc[1] * cos(theta);
            std::tuple<float, float> xy_index = {x / resolution + (grid_max_y - pose_y) / resolution + 2, -y / resolution + (grid_max_x - pose_x) / resolution + 2};
            float x1 = std::get<0>(xy_index);
            float y1 = std::get<1>(xy_index);
            if (x1 >= 0 && x1 <= (mat_dilate_55.cols - 1) && y1 >= 0 && y1 <= (mat_dilate_55.rows - 1))
            {
                if (mat_dilate_55.at<uchar>(get<1>(xy_index), get<0>(xy_index)) < 128)
                {
                    match_point_check++;
                }
                else
                {
                    miss_point_check++;
                }
            }
            else
            {
                miss_point_check++;
            }
        }
        ratio_match = (float)match_point_check / (float)(match_point_check + miss_point_check);

        float thresh_2 = 0.75;
        if (ratio_match1 > 0.8)
        {
            thresh_2 = 0.85;
        }
        else if (ratio_match1 > 0.7)
        {
            thresh_2 = 0.8;
        }
        else
        {
            thresh_2 = 0.8;
        }

        if (ratio_match > thresh_2 && across_ratio < 0.4)
        {
            return 1;
        }
        else
        {
            if (ratio_match1 > 0.7)
                return 0;
            if (ratio_match < 0.70)
                return 0;
        }

        if ((dda_score > 0.55 || score > 0.55) && across_ratio < 0.4)
        {
            //Relocation: check result success
            return 1;
        }
        else
        {
            //false
            return 0;
        }
    }
    return matched;
}