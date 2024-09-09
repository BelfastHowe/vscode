#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>


#define M_PI_F 3.14159265358979323846f

class Relocation
{
    public:
        bool calc_score_with_dda(const std::unique_ptr<Grid2D> &ccg, const struct laser_ranges &laser_ranges, const Rigid2d &pose, float &score, int &debug_idx, float enable_debug);

        bool calc_matched_and_missed_area(const std::unique_ptr<Grid2D> &ccg, const struct laser_ranges &laser_ranges,
                                                      const Rigid2d &pose, float &matched_area, float &missed_area, int &debug_index, float enable_debug);
};
class Grid2D;
struct laser_ranges;
class Rigid2d;
class ProbabilityGrid;
struct CellLimits;
namespace cartographer
{
    namespace sensor
    {
        //class PointCloud;
        typedef std::vector<Eigen::Vector3f> PointCloud;
    };
};

template <class Cell>
class Grid
{
    private:
        int32_t width_, height_;
    public:
        std::vector<Cell> cells;
};

template <class CellType>
class GenericGrid
{
    public:
        typedef float scalar_t;
        typedef CellType GridCellType;
        typedef Eigen::Vector2f Vec2;
        typedef Vec2 coord_t;
        typedef Vec2 grid_coord_t;

    private:
        scalar_t _cell_size;
        scalar_t _cells_per_unit;
        grid_coord_t _orig_cell;
        Grid<CellType> _grid;

    public:
        GenericGrid(int32_t w, int32_t h, scalar_t cell_size,
                    const grid_coord_t &center, std::vector<CellType> &vc)
            : _grid(w, h, vc)
            {
                _cell_size = cell_size;
                _cells_per_unit = 1.0 / _cell_size;
                _orig_cell = center;
            }
};