#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <olfaction_msgs/gas_sensor.h>
#include <olfaction_msgs/anemometer.h>
#include <chrono>
#include <thread>
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <geometry_msgs/Point.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/format.hpp>
#include "frontier_finder.h"
#include "rrt_sampler.h"
#include "gridmap.h"
#include "goal.h"

double get_current_time() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(high_resolution_clock::now().time_since_epoch()).count();
}

void save_vector_to_csv(const std::vector<std::vector<double>>& data, const std::string& filename, const std::string& header="");
void save_gridmap(std::vector<std::vector<int8_t>> data, const std::string& filename);


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Adsm {
public:
    Adsm();
    void loop();

    inline double distance(double x, double y) {
        return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
    }

    const int GOAL_RANDOM_TYPE = 2;
    const int GOAL_EPR_TYPE = 1;
    const int GOAL_EPI_TYPE = 0;

private:
    double k1_;
    double random_sample_r_;
    double rrt_max_r_;
    double rrt_min_r_;
    double frontier_search_th_;
    int goal_cluster_num_;
    double obs_r_;
    double goal_reach_th_;
    double resample_time_th_;
    double gas_max_ = 63000.0;
    double gas_low_th_ = 500.0;
    double gas_high_th_ = 3000.0;
    double sensor_window_len_ = 6.0;
    std::vector<std::pair<double, double>> gas_msg_queue_; // <time, concentration>

    int iter_ = 1;
    double iter_start_rostime_;
    double iter_rate_;
    int max_iter_;
    double source_x_;
    double source_y_;
    double source_th_;
    double stuck_th_;
    std::string random_run_id_;
    std::string ros_run_id_;
    bool visual_;
    std::string data_path_;

    bool do_sample_ = false;
    bool do_sample_again_ = false;
    GoalNode goal_;
    // goal type: 0 epi, 1 epr, 2 random
    std::vector<GoalNode> goals_;
    std::vector<std::pair<double, double>> pose_history_;
    std::vector<GoalNode> epi_set_;
    std::vector<GoalNode> epr_set_;
    std::vector<RRTNode*> rrt_nodes_;
    Gridmap map_;
    RRTSampler rrt_sampler_;
    FrontierFinder frontier_finder_;
    double last_resample_time_;
    double cal_start_time_ = 0.0, cal_duration_ms_ = 0.0;
    double dis_to_source_ = 0.0;
    std::vector<double> stuck_info_{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}; // x, y, last_stuck_time
    double set_random_goal_ = false;
    std::string result_ = "";
    std::vector<std::vector<double>> info_log_;
    std::vector<std::vector<double>> targets_log_;
    std::vector<std::vector<double>> rrt_log_;
    std::vector<std::vector<double>> map_info_log_;
    std::vector<std::vector<int8_t>> map_log_;

    ros::Subscriber pose_sub_;
    ros::Subscriber real_pose_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber gas_sub_;
    ros::Subscriber anemometer_sub_;
    // std::unique_ptr<MoveBaseClient> ac_;
    MoveBaseClient* ac_;
    ros::Publisher visual_points_pub_;
    ros::Publisher visual_lines_pub_;
    ros::Publisher visual_text_pub_;
    double x_ = std::numeric_limits<double>::quiet_NaN(), y_, z_;
    double roll_, pitch_, yaw_;
    double real_x_ = std::numeric_limits<double>::quiet_NaN(), real_y_, real_z_;
    double real_roll_, real_pitch_, real_yaw_;
    double gas_ = std::numeric_limits<double>::quiet_NaN();
    double gas_hit_ = false;
    double wind_speed_ = std::numeric_limits<double>::quiet_NaN();
    double wind_direction_ = std::numeric_limits<double>::quiet_NaN();
    
    void pose_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void real_pose_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void gas_sensor_callback(const olfaction_msgs::gas_sensor& msg);
    void anemometer_callback(const olfaction_msgs::anemometer& msg);

    double probability(double x, double y);
    bool reached_point(double x, double y);
    void create_random_gaol(double start_x, double start_y, double r, double& goal_x, double& goal_y);
    void observe();
    void estimate();
    void evaluate();
    void navigate();
    bool check_terminal();
    void visualize();
    void record_data();
    void save_data();
    void shutdown_ros();
};
