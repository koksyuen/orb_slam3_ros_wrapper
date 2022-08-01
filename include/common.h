#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ORB-SLAM3-specific libraries. Directory is defined in CMakeLists.txt: ${ORB_SLAM3_DIR}
#include "include/System.h"
#include "include/ImuTypes.h"
#include "include/Converter.h"
#include "include/SerializationUtils.h"
#include "include/Tracking.h"
#include "include/MapPoint.h"
#include "include/KeyFrame.h"
#include "include/Atlas.h"
#include "include/Settings.h"


extern std::string map_frame_id, pose_frame_id;


void publish_ros_tf(cv::Mat, ros::Time);
void publish_tf_transform(tf::Transform, ros::Time);

tf::Transform from_orb_to_ros_tf_transform(cv::Mat);