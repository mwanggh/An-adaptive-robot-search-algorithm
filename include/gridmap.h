#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <mutex>

class Gridmap {
public:
    static const int NO_INFORMATION = -1;
    static const int LETHAL_OBSTACLE = 100;

    Gridmap();
    void update(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    inline int getIndex(int mx, int my) const{
        return my * size_x_ + mx;
    }
    bool worldToMap(double wx, double wy, int &mx, int &my) const;
    void mapToWorld(int mx, int my, double &wx, double &wy) const;
    std::string getFrameID() const;
    int getSizeInCellsX() const;
    int getSizeInCellsY() const;
    double getOriginX() const;
    double getOriginY() const;
    double getResolution() const;
    int getCost(int mx, int my) const;
    std::vector<int8_t> getData() const;
    double is_grid_set() const;

    std::mutex& getMutex() const { return mutex_; }
    void lock() const { mutex_.lock(); }
    void unlock() const { mutex_.unlock(); }
    
private:
    nav_msgs::OccupancyGrid occupancy_grid_;
    std::string frame_id_ = "";
    int size_x_ = 0;
    int size_y_ = 0;
    double resolution_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    bool is_grid_set_ = false;
    mutable std::mutex mutex_;
};