#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <camera_info_manager/camera_info_manager.h>

#include "opencv2/core.hpp"
#include "masker_util/maskerConfig.h"

/*
 * A ROS package for image masking & cropping utility for fisheye / wide-angle / omnidirectional image.
 * masker_util.hpp
 *
 *  __ _  _   ___ ______ ____                    _                   
 * /_ | || | / _ \____  / __ \                  | |                  
 *  | | || || (_) |  / / |  | |_   _  __ _ _ __ | |_ _   _ _ __ ___  
 *  | |__   _> _ <  / /| |  | | | | |/ _` | '_ \| __| | | | '_ ` _ \ 
 *  | |  | || (_) |/ / | |__| | |_| | (_| | | | | |_| |_| | | | | | |
 *  |_|  |_| \___//_/   \___\_\\__,_|\__,_|_| |_|\__|\__,_|_| |_| |_|
 *
 * Copyright (C) 2020 1487Quantum
 * 
 * 
 * Licensed under the MIT License.
 * 
 */

class masker_util_node {
public:
    masker_util_node(const ros::NodeHandle& nh_);
    bool init(); //Init
    // === CALLBACK & PUBLISHER ===
    void dr_callback(const masker_util::maskerConfig& config, const uint32_t& level);
    void imgCallback(const sensor_msgs::ImageConstPtr& imgp, const sensor_msgs::CameraInfoConstPtr& cam_info); //Image Input callback

private:
    std::string topic_out; //Output topic name
    ros::NodeHandle nh; //Node handle
    // Pub/Sub
    image_transport::Publisher imgPub;
    ros::Publisher pub_info_camera;
    image_transport::CameraSubscriber imgSub;
    //Dynamic reconfig
    dynamic_reconfigure::Server<masker_util::maskerConfig> msk_server;
    dynamic_reconfigure::Server<masker_util::maskerConfig>::CallbackType msk_cb;

    //Params
    bool setMask{ true };
    int cirRad{ 720 };
    cv::Point cOffset; //Circle offset from img center: x < 0 is left, y < 0 is up]
    cv::Point borderOffset; //Border offset from circle
};

