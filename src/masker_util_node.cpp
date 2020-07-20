#include "masker_util/masker_util_node.hpp"

/*
 * A ROS package for image masking & cropping utility for fisheye / wide-angle / omnidirectional image.
 * masker_util.cpp
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

masker_util_node::masker_util_node(const ros::NodeHandle& nh_)
{
    this->nh = nh_;
};

bool masker_util_node::init()
{
ros::NodeHandle nh_mono(this->nh, "img_out");

camera_info_manager::CameraInfoManager *left_cinfo_;
left_cinfo_ =
    new camera_info_manager::CameraInfoManager(nh_mono);

    // Pub-Sub
    image_transport::ImageTransport it(this->nh);
    imgSub = it.subscribeCamera("img_in", 1, &masker_util_node::imgCallback, this);
 imgPub = it.advertise("img_out/image_raw", 1);
pub_info_camera = this->nh.advertise<sensor_msgs::CameraInfo>("img_out/camera_info", 1); 
   // imgSub = it.subscribe("img_in/image_raw", 1, &masker_util_node::imgCallback, this); //Sub
   // imgPub = it.advertise("img_out/image_raw", 1);

    //start reconfig
    msk_cb = boost::bind(&masker_util_node::dr_callback, this, _1, _2);
    msk_server.setCallback(msk_cb);

    return true;
}

// === CALLBACK & PUBLISHER ===
void masker_util_node::dr_callback(const masker_util::maskerConfig& config, const uint32_t& level)
{
    setMask = config.SET_MASK;
    cirRad = config.RADIUS;
    cOffset = cv::Point(config.CENTER_X, config.CENTER_Y);
    borderOffset = cv::Point(config.BORDER_X, config.BORDER_Y);
}

void masker_util_node::imgCallback(const sensor_msgs::ImageConstPtr& imgp, const sensor_msgs::CameraInfoConstPtr &cam_info)
{
    try {
        cv_bridge::CvImagePtr imagePtrRaw{ cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::BGR8) };
        cv::Mat frame{ imagePtrRaw->image };
        cv::Size f_size(frame.cols, frame.rows);

        //Circle mask
        cv::Scalar circleColor(255, 0, 0);
        cv::Point cPoint(f_size.width / 2 + cOffset.x, f_size.height / 2 + cOffset.y);

        //Draw Crop region
        cv::Rect cropFrame(frame.cols / 2 - (cirRad + borderOffset.x + (frame.cols / 2 - cPoint.x)), 0, 2 * (cirRad + borderOffset.x), frame.rows);

        if (setMask) {
            constexpr int markLen{ 40 };
            cv::circle(frame, cPoint, cirRad, circleColor, 2, cv::LINE_AA); //Add circle
            drawMarker(frame, cPoint, circleColor, cv::MARKER_CROSS, markLen, 2, cv::LINE_AA); //Circle center
            drawMarker(frame, cv::Point(cPoint.x - cOffset.x, cPoint.y - cOffset.y), cv::Scalar(0, 255, 0), cv::MARKER_TILTED_CROSS, markLen / 2, 2, cv::LINE_AA); //Add image centre cross
            cv::rectangle(frame, cropFrame, cv::Scalar(255, 0, 0)); //Crop region
        }

        if (!setMask) {
            //Mask - Remove details outside circle
            cv::Mat pRoi(cv::Mat::zeros(frame.size(), CV_8UC1));
            cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
            cv::circle(mask, cPoint, cirRad, cv::Scalar(255), -1, cv::LINE_AA);
            frame.copyTo(pRoi, mask);
            frame = pRoi(cropFrame); //Crop image
            pRoi.release();
        }

//Publish result
        imgPub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg());
ROS_WARN_STREAM(cam_info->width);

sensor_msgs::CameraInfo info_camera;
    info_camera.header.stamp = ros::Time::now(); 
info_camera.width = 2*(cirRad+borderOffset.x);
info_camera.height = frame.rows+2*borderOffset.y;
    pub_info_camera.publish(info_camera); 

        frame.release();
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imgp->encoding.c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "masker_util_node");
    ros::NodeHandle nh;

    masker_util_node cimg(nh);
    if (!cimg.init()) {
        ROS_WARN("Exiting...");
        ros::shutdown();
        return -1;
    }
    ros::spin();

    return 0;
}

