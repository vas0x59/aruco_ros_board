#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "aruco_detector.h"
// #include <tf2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

// static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle nh_("~");
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber cinfo_sub;

    image_transport::Publisher image_pub_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    Mat R_flip_;
    cv::Ptr<cv::aruco::GridBoard> board_;
    Mat cameraMatrix, distCoeffs;
    ros::Publisher point_pub;
    ros::Publisher pose_pub;
    bool has_camera_info_ = false;
    int markers_x = 1;
    int markers_y = 1;
    float markers_size = 0.17;
    float markers_sep = 0.06;
    int first_marker = 0;
  public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input  video feed and publish output video feed
        nh_.param<int>("markers_x", markers_x, 1);
        nh_.param<int>("markers_y", markers_y, 1);
        nh_.param<float>("markers_size", markers_x, 0.17);
        nh_.param<float>("markers_sep", markers_y, 0.06);
        nh_.param<int>("first_marker", first_marker, 0);


        image_sub_ = it_.subscribe("image", 1,
                                   &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("aruco/aruco_pose", 1);
        point_pub = nh_.advertise<geometry_msgs::Point>("aruco/aruco_point", 1);
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        board_ = board_gen(markers_x, markers_y, markers_size, markers_sep, dictionary_, first_marker);
        cameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
        distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
        R_flip_ = R_flip_gen(1, -1, -1);
        cinfo_sub = nh_.subscribe("camera_info", 1, &ImageConverter::cinfoCallback, this);

        // load_cam(cameraMatrix, distCoeffs, "/home/vasily/Projects/opencv_cpp/logitech.yml"); // cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        // cv::destroyWindow(OPENCV_WINDOW);
    }
    void cinfoCallback(const sensor_msgs::CameraInfoConstPtr &cinfo)
    {
        parseCameraInfo(cinfo, cameraMatrix, distCoeffs);
        has_camera_info_ = true;
        //  cout << "hello2";
    }
    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        // cout << "hello";
        if (has_camera_info_ == true)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            // Draw an example circle on the video stream
            // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

            // Update GUI Window
            // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
            // cv::waitKey(3);

            // Output modified video stream
            Point3d pose, angle;
            estimate_pose(cv_ptr->image, board_, dictionary_, cameraMatrix, distCoeffs, R_flip_, true, pose, angle, cv_ptr->image);
            // cout << pose << endl;
            geometry_msgs::Point msg_point;
            msg_point.x = pose.x;
            msg_point.y = pose.y;
            msg_point.z = pose.z;
            point_pub.publish(msg_point);
            geometry_msgs::Pose msg_pose;

            geometry_msgs::Quaternion quat_msg;
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(angle.x, angle.y, angle.z);
            quat_msg = tf2::toMsg(quat_tf);
            // msg_pose.x = pose.x;
            // msg_pose.y = pose.y;
            // msg_pose.z = pose.z;

            msg_pose.position = msg_point;
            msg_pose.orientation = quat_msg;
            std_msgs::Header msg_header;
            msg_header.frame_id = "aruco";
            msg_header.stamp = ros::Time::now();

            geometry_msgs::PoseStamped msg_pose_st;
            msg_pose_st.pose = msg_pose;
            msg_pose_st.header = msg_header;
            pose_pub.publish(msg_pose_st);

            image_pub_.publish(cv_ptr->toImageMsg());
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}