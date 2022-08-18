/**
* 
* Common functions and variables across all modes (mono/stereo, with or w/o imu)
*
*/

#include "common.h"

std::string map_frame_id, pose_frame_id;

void publish_ros_tf(cv::Mat Tcw, ros::Time current_frame_time)
{
    if (!Tcw.empty())
    {
        tf::Transform tf_transform = from_orb_to_ros_tf_transform (Tcw);

        publish_tf_transform(tf_transform, current_frame_time);
    }
}

void publish_tf_transform(tf::Transform tf_transform, ros::Time current_frame_time)
{
    static tf::TransformBroadcaster tf_broadcaster;

    tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, current_frame_time, map_frame_id, pose_frame_id));
}


tf::Transform from_orb_to_ros_tf_transform(cv::Mat transformation_mat)
{
    float scale_factor=0.96;

    cv::Mat orb_rotation(3, 3, CV_32F);
    cv::Mat orb_translation(3, 1, CV_32F);

    orb_rotation    = transformation_mat.rowRange(0, 3).colRange(0, 3);
    orb_translation = transformation_mat.rowRange(0, 3).col(3);

    tf::Matrix3x3 tf_camera_rotation(
        orb_rotation.at<float> (0, 0), orb_rotation.at<float> (0, 1), orb_rotation.at<float> (0, 2),
        orb_rotation.at<float> (1, 0), orb_rotation.at<float> (1, 1), orb_rotation.at<float> (1, 2),
        orb_rotation.at<float> (2, 0), orb_rotation.at<float> (2, 1), orb_rotation.at<float> (2, 2)
    );

    tf::Vector3 tf_camera_translation(orb_translation.at<float> (0)*scale_factor, orb_translation.at<float> (1)*scale_factor, orb_translation.at<float> (2)*scale_factor);

    // cout << setprecision(9) << "Rotation: " << endl << orb_rotation << endl;
    // cout << setprecision(9) << "Translation xyz: " << orb_translation.at<float> (0) << " " << orb_translation.at<float> (1) << " " << orb_translation.at<float> (2) << endl;

    // Inverse matrix
    tf_camera_rotation    = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

    return tf::Transform(tf_camera_rotation, tf_camera_translation);
}