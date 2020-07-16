#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>

#include "opencv2/core.hpp"

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

class masker_util {
public:
    masker_util(const ros::NodeHandle& nh_);
    //Init
    bool init();
    // === CALLBACK & PUBLISHER ===
    void imgCallback(const sensor_msgs::ImageConstPtr& imgp); //Image Input callback

private:
    ros::NodeHandle nh; //Node handle
    // Pub/Sub
    image_transport::Publisher imgPub;
    image_transport::Subscriber imgSub;
};

