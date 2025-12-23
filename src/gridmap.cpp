#include "gridmap.h"

Gridmap::Gridmap() {}

void Gridmap::update(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    is_grid_set_ = true;
    occupancy_grid_ = *msg;
    frame_id_ = msg->header.frame_id;
    size_x_ = msg->info.width;
    size_y_ = msg->info.height;
    resolution_ = msg->info.resolution;
    origin_x_ = msg->info.origin.position.x;
    origin_y_ = msg->info.origin.position.y;
    // ROS_INFO("Update map: w %d, h %d, res %.2f, ox %.2f, oy %.2f", 
    //     size_x_, size_y_, resolution_, origin_x_, origin_y_);
}

bool Gridmap::worldToMap(double wx, double wy, int &mx, int &my) const {
    if (wx < origin_x_ || wy < origin_y_)
        return false;
 
    mx = (int)((wx - origin_x_) / resolution_);
    my = (int)((wy - origin_y_) / resolution_);

    if (mx < size_x_ && my < size_y_)
        return true;

    return false;
}

void Gridmap::mapToWorld(int mx, int my, double &wx, double &wy) const
{
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
}

std::string Gridmap::getFrameID() const {
    return frame_id_;
}

int Gridmap::getSizeInCellsX() const
{
    return size_x_;
}

int Gridmap::getSizeInCellsY() const
{
    return size_y_;
}

double Gridmap::getOriginX() const
{
    return origin_x_;
}

double Gridmap::getOriginY() const
{
    return origin_y_;
}

double Gridmap::getResolution() const
{
    return resolution_;
}

int Gridmap::getCost(int mx, int my) const
{
    return occupancy_grid_.data[getIndex(mx, my)];
}

std::vector<int8_t> Gridmap::getData() const {
    std::vector<int8_t> map_data;
    map_data.reserve(occupancy_grid_.data.size());
    map_data.insert(map_data.end(), occupancy_grid_.data.begin(), occupancy_grid_.data.end());
    return map_data;
}

double Gridmap::is_grid_set() const
{
    return is_grid_set_;
}
