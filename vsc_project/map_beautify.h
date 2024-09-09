#include <vector>

struct LineCell
{
    LineCell(int i, int n) : index(i), num(n) {}
    LineCell() : index(0), num(0) {}

    int index;
    int num;
};

using LineCellVector = std::vector<LineCell>;

static const uint8_t KObstacleValue = 254;
static const uint8_t KPassedValue = 0;
static const uint8_t KFreeValue = 1;
static const uint8_t KNoInformationValue = 255;
static const uint8_t KMinThresholdValue = 64;
static const uint8_t KMaxThresholdValue = 128;