#include "pear.h"

Pear::Pear() {
    ros::NodeHandle nh("~");
    ros::NodeHandle global_nh;
    // ros::Time::waitForValid();

    ROS_INFO("Read parameters.");
    nh.getParam("k1", k1_);
    nh.getParam("random_sample_r", random_sample_r_);
    nh.getParam("goal_cluster_num", goal_cluster_num_);
    nh.getParam("obs_r", obs_r_);
    nh.getParam("goal_reach_th", goal_reach_th_);
    nh.getParam("resample_time_th", resample_time_th_);
    nh.getParam("gas_max", gas_max_);
    nh.getParam("gas_high_th", gas_high_th_);
    nh.getParam("gas_low_th", gas_low_th_);
    nh.getParam("sensor_window_length", sensor_window_len_);
    ROS_INFO("k1 %.2f", k1_);
    ROS_INFO("gas_max %.2f, gas_high_th %.2f, gas_low_th %.2f", gas_max_, gas_high_th_, gas_low_th_);
    ROS_INFO("sensor_window_length %.2f", sensor_window_len_);

    nh.getParam("iter_rate", iter_rate_);
    nh.getParam("max_iter", max_iter_);
    nh.getParam("source_x", source_x_);
    nh.getParam("source_y", source_y_);
    nh.getParam("source_th", source_th_);
    nh.getParam("stuck_duration_th", stuck_th_);
    global_nh.getParam("run_id", ros_run_id_);
    nh.getParam("visual", visual_);
    nh.getParam("data_path", data_path_);
    boost::uuids::random_generator generator;
    boost::uuids::uuid uuid = generator();
    random_run_id_ = boost::uuids::to_string(uuid);
    data_path_ = data_path_ + '/' + random_run_id_;
    ROS_INFO("iter_rate %.2f, max_iter %d", iter_rate_, max_iter_);
    ROS_INFO("source_x %.2f, source_y %.2f", source_x_, source_y_);

    ROS_INFO("Subscribe to topics.");
    ros::Rate rate(5);
    std::string pose_topic, real_pose_topic, map_topic; 
    nh.getParam("pose_topic", pose_topic);
    nh.getParam("real_pose_topic", real_pose_topic);
    pose_sub_ = nh.subscribe(pose_topic, 5, &Pear::pose_callback, this);
    real_pose_sub_ = nh.subscribe(real_pose_topic, 5, &Pear::real_pose_callback, this);
    while (std::isnan(x_) || std::isnan(real_x_)) {
        ROS_INFO("Waiting %s and %s.", pose_topic.c_str(), real_pose_topic.c_str());
        ros::spinOnce();
        rate.sleep();
    }

    nh.getParam("map_topic", map_topic);
    map_sub_ = nh.subscribe(map_topic, 1, &Pear::map_callback, this);
    while (!map_.is_grid_set()) {
        ROS_INFO("Waiting %s.", map_topic.c_str());
        ros::spinOnce();
        rate.sleep();
    }

    std::string gas_sensor_topic, anemometer_topic;
    nh.getParam("gas_sensor_topic", gas_sensor_topic);
    nh.getParam("anemometer_topic", anemometer_topic);
    gas_sub_ = nh.subscribe(gas_sensor_topic, 1, &Pear::gas_sensor_callback, this);
    anemometer_sub_ = nh.subscribe(anemometer_topic, 1, &Pear::anemometer_callback, this);
    while (std::isnan(gas_) || std::isnan(wind_speed_)) {
        ROS_INFO("Waiting %s and %s.", gas_sensor_topic.c_str(), anemometer_topic.c_str());
        ros::spinOnce();
        rate.sleep();
    }

    std::string move_base_name;
    nh.getParam("move_base_name", move_base_name);
    ac_ = new MoveBaseClient(move_base_name, true);
    ROS_INFO("Waiting %s.", move_base_name.c_str());
    ac_->waitForServer(); 

    ROS_INFO("Initialize variables.");
    iter_ = 1;
    goal_.x = x_;
    goal_.y = y_;

    nh.getParam("frontier_search_th", frontier_search_th_);
    frontier_finder_ = FrontierFinder(frontier_search_th_, obs_r_);
    int rrt_max_iter;
    double rrt_step_size;
    nh.getParam("rrt_max_iter", rrt_max_iter);
    nh.getParam("rrt_max_r", rrt_max_r_);
    nh.getParam("rrt_min_r", rrt_min_r_);
    nh.getParam("rrt_step_size", rrt_step_size);
    rrt_sampler_ = RRTSampler(rrt_max_iter, rrt_max_r_, obs_r_, rrt_step_size);
    last_resample_time_ = ros::Time::now().toSec();

    visual_points_pub_ = nh.advertise<visualization_msgs::Marker>("visual_points", 1);
    visual_lines_pub_ = nh.advertise<visualization_msgs::Marker>("visual_lines", 1);
    visual_text_pub_ = nh.advertise<jsk_rviz_plugins::OverlayText>("visual_text", 1);

    ROS_INFO("Pear initialized.");
}

void Pear::loop() {
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    ROS_INFO("Start loop. Rate: %.2f", iter_rate_);
    ros::Rate rate(iter_rate_);
    while (ros::ok()) {
        iter_start_rostime_ = ros::Time::now().toSec();
        ROS_INFO("=========================PEAR=========================");
        ROS_INFO("Iteration %2d, ROS time %.2f", iter_, iter_start_rostime_);
        ros::spinOnce();
        observe();
        estimate();
        evaluate();
        navigate();
        record_data();
        if (visual_)
            visualize();
        if (check_terminal()) {
            ac_->cancelAllGoals();
            save_data();
            shutdown_ros();
        }
        
        iter_ = iter_ + 1;
        rate.sleep();
    }
}

void Pear::pose_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    nav_msgs::Odometry pose_raw = *msg;
    x_ = pose_raw.pose.pose.position.x;
    y_ = pose_raw.pose.pose.position.y;
    z_ = pose_raw.pose.pose.position.z;
    tf::Quaternion q(
        pose_raw.pose.pose.orientation.x,
        pose_raw.pose.pose.orientation.y,
        pose_raw.pose.pose.orientation.z,
        pose_raw.pose.pose.orientation.w
    );

    tf::Matrix3x3 m(q);
    m.getRPY(roll_, pitch_, yaw_);
}

void Pear::real_pose_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    nav_msgs::Odometry pose_raw = *msg;
    real_x_ = pose_raw.pose.pose.position.x;
    real_y_ = pose_raw.pose.pose.position.y;
    real_z_ = pose_raw.pose.pose.position.z;
    tf::Quaternion q(
        pose_raw.pose.pose.orientation.x,
        pose_raw.pose.pose.orientation.y,
        pose_raw.pose.pose.orientation.z,
        pose_raw.pose.pose.orientation.w
    );

    tf::Matrix3x3 m(q);
    m.getRPY(real_roll_, real_pitch_, real_yaw_);
}

void Pear::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_.update(msg);
    ROS_INFO("Map: width %d, height %d", map_.getSizeInCellsX(), map_.getSizeInCellsX());
}

void Pear::gas_sensor_callback(const olfaction_msgs::gas_sensor& msg) {
    gas_ = gas_max_ - msg.raw;

    double msg_time = msg.header.stamp.toSec();
    double rate = 0.0;
    gas_msg_queue_.push_back({msg_time, gas_});
    while (msg_time - gas_msg_queue_.front().first > sensor_window_len_) {
        gas_msg_queue_.erase(gas_msg_queue_.begin());
    }

    if (gas_ < gas_low_th_) {
        gas_hit_ = false;
        ROS_INFO("Gas: concentration %.2f < %.2f, hit %d", gas_, gas_low_th_, (int)gas_hit_);
    } else if (gas_ > gas_high_th_) {
        gas_hit_ = true;
        ROS_INFO("Gas: concentration %.2f > %.2f, hit %d", gas_, gas_high_th_, (int)gas_hit_);
    } else {
        bool in_rec = true;
        for (std::size_t i = 1; i < gas_msg_queue_.size(); ++i) {
            if (gas_msg_queue_[i].second - gas_msg_queue_[i-1].second >= 0.0 || 
                gas_msg_queue_[i-1].second - gas_high_th_ >= 0.0) {
                in_rec = false;
            }
        }
        gas_hit_ = in_rec ? false : true;
        ROS_INFO("Gas: concentration %.2f, in_rec %d , hit %d", gas_, (int)in_rec, (int)gas_hit_);
    }
}

void Pear::anemometer_callback(const olfaction_msgs::anemometer& msg) {
    wind_speed_ = msg.wind_speed;
    wind_direction_ = msg.wind_direction;
    ROS_INFO("Wind: speed %.2f, direction %.2f", wind_speed_, wind_direction_);
}

double Pear::probability(double x, double y) {
    double Q = 4.0;
    double D = 1.0;
    double tau = 250;
    double V = wind_speed_;
    double phi = yaw_ - wind_direction_;
    double lam = std::sqrt(D * tau / (1 + V * V * tau / (4 * D)));
    double dis = std::sqrt((x - x_) * (x - x_) + (y - y_) * (y - y_));
    double dx = x_ - x;
    double dy = y_ - y;

    double pa = Q / (4 * M_PI * D * (std::abs(dis + 0.0001)));
    double pb = std::exp(-dis / lam);
    double pc = std::exp(-dx * V * std::cos(phi) / (2 * D));
    double pd = std::exp(-dy * V * std::sin(phi) / (2 * D));
    double p = pa * pb * pc * pd;
    
    return p;
}

bool Pear::reached_point(double x, double y) {
    for (auto& pose : pose_history_) {
        if (distance(pose.first - x, pose.second - y) < goal_reach_th_) {
            return true;
        }
    }
    return false;
}

void Pear::create_random_gaol(double start_x, double start_y, double r, double& goal_x, double& goal_y) {
    double rand_theta = static_cast<double>(rand()) / RAND_MAX * 2 * M_PI;
    double rand_r = static_cast<double>(rand()) / RAND_MAX * r;
    goal_x = start_x + rand_r * cos(rand_theta);
    goal_y = start_y + rand_r * sin(rand_theta);
}

void Pear::observe() {
    cal_start_time_ = get_current_time();

    ROS_INFO("Pose: x %.2f, y %.2f, yaw %.2f", x_, y_, yaw_);
    ROS_INFO("Real pose: x %.2f, y %.2f, roll %.2f, pitch %.2f, yaw %.2f", 
        real_x_, real_y_, real_roll_, real_pitch_, real_yaw_);

    pose_history_.push_back({x_, y_});

    do_sample_ = false;
    double dis = distance(x_ - goal_.x, y_ - goal_.y);
    double t = ros::Time::now().toSec();
    double t_duration = t - last_resample_time_;
    if (dis < goal_reach_th_ || t_duration > resample_time_th_) {
        do_sample_ = true;
        last_resample_time_ = t;
    }
    if (do_sample_again_) {
        do_sample_ = true;
        do_sample_again_ = false;
    }
    ROS_INFO("sample: %d, duration %.2f th %.2f, distance %.2f th %.2f", 
        (int)do_sample_, t_duration, resample_time_th_, dis, goal_reach_th_);
}

void Pear::estimate() {
    if (do_sample_) {
        // Resampling using the RRT method
        int MAX_SAMPLE_TIME = 10;
        for (int i = 0; i < MAX_SAMPLE_TIME; ++i) {
            std::vector<RRTNode*> rrt_nodes_temp;
            if (i == 0) {
                rrt_nodes_temp = rrt_sampler_.sample(&map_, x_, y_);
            } else {
                double new_x, new_y;
                create_random_gaol(x_, y_, obs_r_, new_x, new_y);
                rrt_nodes_temp = rrt_sampler_.sample(&map_, new_x, new_y);
            }
            ROS_INFO("Sample try: %d/%d, nodes size: %zu", i, MAX_SAMPLE_TIME, rrt_nodes_temp.size());

            if (rrt_nodes_temp.size() < 2) {
                for (RRTNode * node : rrt_nodes_temp) {
                    delete node;
                }
                rrt_nodes_temp.clear();
            } else {
                ROS_INFO("Clear epi_set_ and rrt_nodes_");
                epi_set_.clear();
                for (RRTNode* node : rrt_nodes_) {
                    delete node;
                }
                rrt_nodes_.clear();
                rrt_nodes_ = rrt_nodes_temp;
                break;
            }
        }
    }
    if (rrt_nodes_.size() < 2) {
        do_sample_again_ = true;
    }
    ROS_INFO("do_sample_again_: %d", (int)do_sample_again_);
    if (do_sample_ && (!do_sample_again_)) {
        // Points are divided into goal_cluster_num_ categories according to their angle with the robot
        std::vector<int> categories(rrt_nodes_.size(), -1);
        for (std::size_t i = 0; i < rrt_nodes_.size(); ++i) {
            // Ignore the first element as it is the robot position
            if (i == 0) {
                continue;
            }
            double angle = atan2(rrt_nodes_[i]->y - y_, rrt_nodes_[i]->x - x_);
            if (angle < 0) {
                angle += 2 * M_PI;
            }
            categories[i] = static_cast<int>((angle / (2 * M_PI)) * goal_cluster_num_);
        }
        std::vector<double> farthest_length(goal_cluster_num_, -1.0);
        std::vector<int> fartest_index(goal_cluster_num_, -1);
        // Find the point in each category that is farthest from the robot
        for (std::size_t i = 0; i < rrt_nodes_.size(); ++i) {
            if (categories[i] < 0) {
                continue;
            }
            double dist = distance(rrt_nodes_[i]->x - x_, rrt_nodes_[i]->y - y_);
            if (dist > farthest_length[categories[i]]) {
                farthest_length[categories[i]] = dist;
                fartest_index[categories[i]] = i;
            }
        }
        int f_count = 0;
        std::vector<int> is_farthest(rrt_nodes_.size(), 0);
        for (const auto& idx : fartest_index) {
            if (idx >= 0) {
                is_farthest[idx] = 1;
                f_count += 1;
            }
        }
        ROS_INFO("Categories number: %d, max num: %d", f_count, goal_cluster_num_);

        // The farthest point in each category that has frontier points around it will be added to epr_set
        // all other points will be added to epi_set_
        epi_set_.clear();
        for (std::size_t i = 0; i < rrt_nodes_.size(); ++i) {
            if (categories[i] < 0) {
                continue;
            }
            
            if (!is_farthest[i]) {
                epi_set_.push_back(GoalNode(iter_, rrt_nodes_[i]->x, rrt_nodes_[i]->y, GOAL_EPI_TYPE));
                continue;
            }

            bool do_add_epr = true;
            for (auto& point : epr_set_) {
                double dis = distance(rrt_nodes_[i]->x - point.x, rrt_nodes_[i]->y - point.y);
                if (dis < goal_reach_th_ / 2.0) {
                    do_add_epr = false;
                    break;
                }
            }
            if (!do_add_epr) {
                epi_set_.push_back(GoalNode(iter_, rrt_nodes_[i]->x, rrt_nodes_[i]->y, GOAL_EPI_TYPE));
                continue;
            }

            double f_size = static_cast<double>(frontier_finder_.find(&map_, rrt_nodes_[i]->x, rrt_nodes_[i]->y, false).size());
            if (f_size == 0.0) {
                epi_set_.push_back(GoalNode(iter_, rrt_nodes_[i]->x, rrt_nodes_[i]->y, GOAL_EPI_TYPE));
                continue;
            }
            
            GoalNode temp_goal_node = GoalNode(iter_, rrt_nodes_[i]->x, rrt_nodes_[i]->y, GOAL_EPR_TYPE);
            temp_goal_node.frontier_size = f_size;
            epr_set_.push_back(temp_goal_node);
        }
        ROS_INFO("size: epi_set %zu, epr_set %zu", epi_set_.size(), epr_set_.size());
    }

    ROS_INFO("Combine epr_set_ and epi_set_.");
    ROS_INFO("Calculate probability and frontier size.");
    goals_.clear();

    std::vector<int> epi_set_delete;
    for (int i = 0; i < epi_set_.size(); ++i) {
        if (distance(epi_set_[i].x - x_, epi_set_[i].y - y_) < rrt_min_r_) {
            epi_set_delete.push_back(i);
        }
    }
    for (int i = epi_set_delete.size() - 1; i >= 0; --i) {
        epi_set_.erase(epi_set_.begin() + epi_set_delete[i]);
    }
    for (auto& temp_goal : epi_set_) {
        temp_goal.probability = gas_hit_ ? probability(temp_goal.x, temp_goal.y) : 0.0;
        temp_goal.frontier_size = 0.0;
        goals_.push_back(temp_goal);
    }

    std::vector<int> epr_set_delete;
    for (int i = 0; i < epr_set_.size(); ++i) {
        double dis = distance(x_ - epr_set_[i].x, y_ - epr_set_[i].y);
        double f_size = epr_set_[i].frontier_size;
        if (dis < rrt_max_r_ + frontier_search_th_) {
            f_size = static_cast<double>(frontier_finder_.find(&map_, epr_set_[i].x, epr_set_[i].y, false).size());
        }
        epr_set_[i].frontier_size = f_size;
        // Remove points whose information gain (number of surrounding frontier points) is 0.
        if (reached_point(epr_set_[i].x, epr_set_[i].y) || f_size == 0.0) {
            epr_set_delete.push_back(i);
        }
    }
    for (int i = epr_set_delete.size() - 1; i >= 0; --i) {
        epr_set_.erase(epr_set_.begin() + epr_set_delete[i]);
    }
    for (auto& temp_goal : epr_set_) {
        temp_goal.probability = gas_hit_ ? probability(temp_goal.x, temp_goal.y) : 0.0;
        temp_goal.frontier_size = temp_goal.frontier_size;
        goals_.push_back(temp_goal);
    }

    ROS_INFO("size: goals_ %zu, epi_set_ %zu, epr_set_ %zu", goals_.size(), epi_set_.size(), epr_set_.size());
}

void Pear::evaluate() {
    if ((!goals_.empty()) && (!set_random_goal_)) {
        // Normalize j_p, j_i, and j
        ROS_INFO("Calculate j.");
        double sum_probability = 0.0;
        double sum_frontier_size = 0.0;
        for (auto& temp_goal : goals_) {
            sum_probability += temp_goal.probability;
            sum_frontier_size += temp_goal.frontier_size;
        }
        ROS_INFO("sum: probability %.2f, frontier_size %.2f", sum_probability, sum_frontier_size);

        for (auto& temp_goal : goals_) {
            temp_goal.j_p = sum_probability > 0 ? temp_goal.probability / sum_probability : 0.0;
            temp_goal.j_i = sum_frontier_size > 0 ? temp_goal.frontier_size / sum_frontier_size : 0.0;
            double f = temp_goal.iteration / static_cast<double>(iter_);
            temp_goal.j = temp_goal.j_p + k1_ * f * temp_goal.j_i;
        }
    } else {
        goals_.clear();
        ROS_INFO("Generate a random goal. r: %.2f", random_sample_r_);
        double new_x, new_y;
        create_random_gaol(goal_.x, goal_.y, random_sample_r_, new_x, new_y);
        goals_.push_back(GoalNode(iter_, new_x, new_y, GOAL_RANDOM_TYPE));
    }

    // Select the point with the largest j as the navigation goal
    goal_ = goals_[0];
    for (const auto& temp_goal : goals_) {
        if (temp_goal.j > goal_.j) {
            goal_ = temp_goal;
        }
    }

    ROS_INFO("Select navigation goal: x %.2f y %.2f", goal_.x, goal_.y);
    ROS_INFO("goal: j %.2f, j_p %.2f, j_i %.2f, type %d", goal_.j ,goal_.j_p, goal_.j_i, goal_.type);
}

void Pear::navigate() {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = map_.getFrameID();
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = goal_.x;
    goal.target_pose.pose.position.y = goal_.y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    ac_->sendGoal(goal);
    ROS_INFO("Send goal: x %.2f y %.2f", goal_.x, goal_.y);

    cal_duration_ms_ = (get_current_time() - cal_start_time_)*1000;
    ROS_INFO("Loop cost time: %.2f ms", cal_duration_ms_);
}

bool Pear::check_terminal() {
    // find source check
    dis_to_source_ = distance(real_x_ - source_x_, real_y_ - source_y_);
    if (dis_to_source_ <= source_th_) {
        result_ = "FIND_SOURCE";
        ROS_INFO("FIND_SOURCE: distance %.2f", dis_to_source_);
        return true;
    }
    
    // max iteration check
    if (iter_ >= max_iter_) {
        result_ = "REACH_MAX_ITER";
        ROS_INFO("REACH_MAX_ITER: iter %.d, max_iter %d", iter_, max_iter_);
        return true;
    }

    // robot fall check
    if (abs(real_roll_) > M_PI / 4.0 || abs(real_pitch_) > M_PI / 4.0) {
        result_ = "ROBOT_FALL";
        ROS_INFO("ROBOT_FALL: roll %.2f, pitch %.2f", real_roll_, real_pitch_);
        return true;
    }

    // robot stuck check
    if (std::isnan(stuck_info_[0])) {
        stuck_info_[0] = real_x_;
        stuck_info_[1] = real_y_;
        stuck_info_[2] = ros::Time::now().toSec();
    } else {
        double dis = distance(real_x_ - stuck_info_[0], real_y_ - stuck_info_[1]);
        if (dis > 0.15) {
            stuck_info_[0] = real_x_;
            stuck_info_[1] = real_y_;
            stuck_info_[2] = ros::Time::now().toSec();
        }
    }

    double stuck_duration = ros::Time::now().toSec() - stuck_info_[2];
    if (stuck_duration > stuck_th_) {
        result_ = "ROBOT_STUCK";
        ROS_INFO("ROBOT_STUCK: duration %.2f, th %.2f", stuck_duration, stuck_th_);
        return true;
    }
    set_random_goal_ = stuck_duration > stuck_th_ / 3.0 ? true : false;
    ROS_INFO("dis2source: %.2f th %.2f, stuck_d %.2f th %.2f set_r_goal %d", 
        dis_to_source_, source_th_, stuck_duration, stuck_th_, (int)set_random_goal_);
    
    return false;
}

void Pear::visualize() {
    ROS_INFO("Visualize.");
    std::string map_frame = map_.getFrameID();

    // targets
    visualization_msgs::Marker marker_t;
    marker_t.ns = "visual_points";
    marker_t.id = 0;
    marker_t.header.frame_id = map_frame;
    marker_t.header.stamp = ros::Time::now();
    marker_t.action = visualization_msgs::Marker::ADD;
    marker_t.type = visualization_msgs::Marker::POINTS;
    marker_t.lifetime = ros::Duration(1.0/iter_rate_);
    if (goals_.size() > 1) {
        double max_j = goals_[0].j;
        double min_j = goals_[0].j;
        for (const auto& temp_goal : goals_) {
            if (temp_goal.j > max_j) {
                max_j = temp_goal.j;
            }
            if (temp_goal.j < min_j) {
                min_j = temp_goal.j;
            }
        }
        for (const auto& temp_goal : goals_) {
            geometry_msgs::Point point;
            point.x = temp_goal.x;
            point.y = temp_goal.y;
            point.z = 0.0;
            marker_t.points.push_back(point);
            std_msgs::ColorRGBA goal_color;
            double color_level = max_j > 0 ? temp_goal.j / max_j : 0.0;
            if (temp_goal.type == GOAL_EPI_TYPE) {
                goal_color.r = 1.0 - color_level;
                goal_color.g = 1.0 - color_level;
                goal_color.b = 1.0;
            }
            if (temp_goal.type == GOAL_EPR_TYPE) {
                goal_color.r = 1.0 - color_level;
                goal_color.g = 1.0;
                goal_color.b = 1.0 - color_level;
            }
            goal_color.a = 1.0;
            marker_t.colors.push_back(goal_color);
        }
    }

    // goal
    geometry_msgs::Point point;
    point.x = goal_.x;
    point.y = goal_.y;
    point.z = 0.0;
    marker_t.points.push_back(point);
    std_msgs::ColorRGBA goal_color;
    goal_color.r = 1.0;
    goal_color.g = 0.0;
    goal_color.b = 0.0;
    goal_color.a = 1.0;
    marker_t.colors.push_back(goal_color);

    marker_t.pose.orientation.w = 1.0;
    marker_t.scale.x = 0.05;
    marker_t.scale.y = 0.05;
    marker_t.scale.z = 0.05;
    visual_points_pub_.publish(marker_t);

    // RRT tree
    visualization_msgs::Marker marker_rrt;
    marker_rrt.ns = "visual_lines";
    marker_rrt.id = 0;
    marker_rrt.header.frame_id = map_frame;
    marker_rrt.header.stamp = ros::Time::now();
    marker_rrt.action = visualization_msgs::Marker::ADD;
    marker_rrt.type = visualization_msgs::Marker::LINE_LIST;
    marker_rrt.lifetime = ros::Duration(1.0/iter_rate_);
    for (const auto& node : rrt_nodes_) {
        if (node->parent_idx == -1) {
            continue;
        }
        const auto& parent_node = rrt_nodes_[node->parent_idx];
        geometry_msgs::Point parent_point;
        parent_point.x = parent_node->x;
        parent_point.y = parent_node->y;
        parent_point.z = 0.0;
        marker_rrt.points.push_back(parent_point);
        point.x = node->x;
        point.y = node->y;
        point.z = 0.0;
        marker_rrt.points.push_back(point);
    }
    std_msgs::ColorRGBA line_color;
    line_color.r = 0.0;
    line_color.g = 1.0;
    line_color.b = 1.0;
    line_color.a = 1.0;
    marker_rrt.color = line_color;
    marker_rrt.pose.orientation.w = 1.0;
    marker_rrt.scale.x = 0.01;
    marker_rrt.scale.y = 0.01;
    marker_rrt.scale.z = 0.01;
    visual_lines_pub_.publish(marker_rrt);

    // source
    visualization_msgs::Marker marker_source;
    marker_source.ns = "visual_source";
    marker_source.id = 0;
    marker_source.header.frame_id = map_frame;
    marker_source.header.stamp = ros::Time::now();
    marker_source.action = visualization_msgs::Marker::ADD;
    marker_source.type = visualization_msgs::Marker::CUBE;
    marker_source.lifetime = ros::Duration(1.0/iter_rate_);
    marker_source.pose.position.x = source_x_;
    marker_source.pose.position.y = source_y_;
    marker_source.pose.position.z = 0.3;
    marker_source.pose.orientation.x = 0.0;
    marker_source.pose.orientation.y = 0.0;
    marker_source.pose.orientation.z = 0.0;
    marker_source.pose.orientation.w = 1.0;
    marker_source.color.r = 0.0;
    marker_source.color.g = 1.0;
    marker_source.color.b = 0.0;
    marker_source.color.a = 1.0;
    marker_source.scale.x = 0.2;
    marker_source.scale.y = 0.2;
    marker_source.scale.z = 0.6;
    visual_lines_pub_.publish(marker_source);

    // text
    jsk_rviz_plugins::OverlayText text_msg;
    text_msg.width = 600;
    text_msg.height = 200;
    text_msg.text_size = 35;
    text_msg.line_width = 1;
    std::string gas_hit_str = gas_hit_ ? "True" : "False";
    boost::format fmt = boost::format("Iteration: %d\nGas hit: %s\nWind direction: %.2f");
    fmt % iter_ % gas_hit_str % wind_direction_;
    text_msg.text = fmt.str();
    std_msgs::ColorRGBA fg_color;
    fg_color.r = 1.0;
    fg_color.g = 0.0;
    fg_color.b = 0.0;
    fg_color.a = 1.0;
    text_msg.fg_color = fg_color;
    std_msgs::ColorRGBA bg_color;
    bg_color.r = 0.0;
    bg_color.g = 0.0;
    bg_color.b = 0.0;
    bg_color.a = 0.2;
    text_msg.bg_color = bg_color;
    visual_text_pub_.publish(text_msg);

    // frontier
    // std::vector<std::pair<double, double>> frontiers = frontier_finder_.find(&map_, goal_.x, goal_.y, false);
    // visualization_msgs::Marker marker_f;
    // marker_f.ns = "visual_frontiers";
    // marker_f.id = 0;
    // marker_f.header.frame_id = map_frame;
    // marker_f.header.stamp = ros::Time::now();
    // marker_f.action = visualization_msgs::Marker::ADD;
    // marker_f.type = visualization_msgs::Marker::POINTS;
    // marker_f.lifetime = ros::Duration(1.0/iter_rate_);
    // if (frontiers.size() > 1) {
    //     for (const auto& pair : frontiers) {
    //         geometry_msgs::Point point;
    //         point.x = pair.first;
    //         point.y = pair.second;
    //         point.z = 0.0;
    //         marker_f.points.push_back(point);
    //         std_msgs::ColorRGBA goal_color;
    //         goal_color.r = 0.5;
    //         goal_color.g = 1.0;
    //         goal_color.b = 1.0;
    //         goal_color.a = 1.0;
    //         marker_f.colors.push_back(goal_color);
    //     }
    // }
    // marker_f.pose.orientation.w = 1.0;
    // marker_f.scale.x = 0.05;
    // marker_f.scale.y = 0.05;
    // marker_f.scale.z = 0.05;
    // visual_points_pub_.publish(marker_f);
}

void Pear::record_data() {
    ROS_INFO("Record data.");
    // double r_st = get_current_time();
    // info
    std::vector<double> info_iter = {
        static_cast<double>(iter_), iter_start_rostime_, cal_duration_ms_, goal_.x, goal_.y, static_cast<double>(goal_.type),
        x_, y_, yaw_, real_x_, real_y_, real_yaw_,
        gas_, gas_hit_, wind_speed_, wind_direction_,
        static_cast<double>(do_sample_), static_cast<double>(epi_set_.size()), static_cast<double>(epr_set_.size())
    };
    info_log_.push_back(info_iter);

    // goals_
    for (auto& target : goals_) {
        std::vector<double> targets_iter = {
            static_cast<double>(iter_), target.iteration, target.x, target.y, static_cast<double>(target.type), target.yaw, target.j, 
            target.j_p, target.j_i, target.probability, target.frontier_size
        };
        targets_log_.push_back(targets_iter);
    }

    // rrt_nodes_
    for (RRTNode* node : rrt_nodes_) {
        std::vector<double> rrt_iter = {
            static_cast<double>(iter_), static_cast<double>(node->idx), node->x, node->y, static_cast<double>(node->parent_idx), node->cost
        };
        rrt_log_.push_back(rrt_iter);
    }

    // map_ (info)
    std::vector<double> map_info_iter = {
        double(iter_), map_.getResolution(), static_cast<double>(map_.getSizeInCellsX()), static_cast<double>(map_.getSizeInCellsY()),  
        map_.getOriginX(), map_.getOriginY()
    };
    map_info_log_.push_back(map_info_iter);

    // map_ (grid data)
    map_log_.push_back(map_.getData());

    // ROS_INFO("Record data: cost time %.2f ms", (get_current_time()-r_st)*1000.0);
}

void Pear::save_data() {
    ROS_INFO("Save data.");
    std::string make_dirs_cmd = "mkdir -p " + data_path_;
    ROS_INFO("%s", make_dirs_cmd.c_str());
    system(make_dirs_cmd.c_str());
    // result.txt
    std::ostringstream result_str;
    result_str << std::fixed << std::setprecision(2)
                << "result:" << result_ << "\n"
                << "x:" << x_ << "\n"
                << "y:" << y_ << "\n"
                << "roll:" << roll_ << "\n"
                << "pitch:" << pitch_ << "\n"
                << "source_x:" << source_x_ << "\n"
                << "source_y:" << source_y_ << "\n"
                << "distance_to_source:" << dis_to_source_ << "\n"
                << "iteration:" << iter_ << "\n"
                << "max_iteration:" << max_iter_ << "\n"
                << "stuck_start_time:" << stuck_info_[2] << "\n"
                << "time:" << ros::Time::now().toSec() << "\n"
                << "run_id:" << random_run_id_ << "\n"
                << "ros_run_id:" << ros_run_id_;
    std::string result = result_str.str();
    ROS_INFO("%s", result.c_str());
    std::string result_file = data_path_ + "/result.txt";
    ROS_INFO("Save file: %s", result_file.c_str());
    std::ofstream file(result_file);
    if (file.is_open()) {
        file << result;
        file.close();
    } else {
        ROS_ERROR("Unable to open file");
    }

    std::string info_header = "iter,ros_time,cal_duration_ms,goal_x,goal_y,goal_type,";
    info_header += "robot_x,robot_y,robot_yaw,robot_real_x,robot_real_y,robot_real_yaw,";
    info_header += "gas,gas_hit,wind_speed,wind_direction,do_sample,epi_set_size,epr_set_size";
    std::string targets_header = "iteration,iteration_create,x,y,type,yaw,j,j_p,j_i,probability,frontier_size";
    std::string rrt_header = "iter,idx,x,y,parent_idx,cost";
    std::string map_info_header = "iter,resolution,size_x,size_y,origin_x,origin_y";

    save_vector_to_csv(info_log_, data_path_+"/info.csv", info_header); // info
    save_vector_to_csv(targets_log_, data_path_+"/targets.csv", targets_header); // goals_
    save_vector_to_csv(rrt_log_, data_path_+"/rrt_nodes.csv", rrt_header); // rrt_nodes_
    save_vector_to_csv(map_info_log_, data_path_+"/map_info.csv", map_info_header); // map_ (info)
    save_gridmap(map_log_, data_path_+"/map.txt"); // map_ (grid data)
}

void Pear::shutdown_ros() {
    std::string param_file = data_path_ + "/params.yaml";
    std::string save_params_cmd = "touch " + param_file + " && rosparam dump " + param_file;
    ROS_INFO("%s", save_params_cmd.c_str());
    system(save_params_cmd.c_str());

    std::string cp_ros_log_cmd = "cp -r ~/.ros/log/" + ros_run_id_ + "/ " + data_path_ + " 2>/dev/null";
    ROS_INFO("%s", cp_ros_log_cmd.c_str());
    system(cp_ros_log_cmd.c_str());

    size_t pos = data_path_.find_last_of('/');
    std::string dir_name = data_path_.substr(pos + 1);
    std::string zip_cmd = "cd " + data_path_ + "/.. && zip -rm " + dir_name + ".zip " + dir_name + " > /dev/null 2>&1";
    ROS_INFO("%s", zip_cmd.c_str());
    system(zip_cmd.c_str());

    ROS_INFO("Shutdown ROS");
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pear_node");
    Pear pear;
    pear.loop();

    return 0;
}

void save_vector_to_csv(const std::vector<std::vector<double>> &data, const std::string &filename, const std::string &header) {
    ROS_INFO("Save file: %s", filename.c_str());
    std::ofstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Unable to open file.");
        return;
    }

    if (!header.empty()) {
        file << header << "\n";
    }

    file << std::fixed << std::setprecision(4);
    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << row[i];
            if (i < row.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();
    if (!file) {
        ROS_ERROR("Could not write to file.");
    }
}

void save_gridmap(std::vector<std::vector<int8_t>> data, const std::string &filename) {
    ROS_INFO("Save file: %s", filename.c_str());
    std::ofstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Unable to open file.");
        return;
    }

    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            file << (int)row[i];
            if (i < row.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();
    if (!file) {
        ROS_ERROR("Could not write to file.");
    }
}
