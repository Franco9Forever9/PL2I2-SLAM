#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

ros::Subscriber subImg;
ros::Subscriber subImu;
ros::Publisher pubImg;
ros::Publisher pubImu;

void image_callback(const sensor_msgs::ImageConstPtr &imgMsg)
{
    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::TYPE_8UC3);
    cv::Mat srcImg = cvPtr->image.clone();
    std::vector<cv::Mat> grayImgs;
    cv::split(srcImg, grayImgs);
    ROS_INFO_ONCE("-- > Img Sub < --");
    
    cvPtr->encoding = std::string("mono8");
    cvPtr->image = grayImgs[0];
    sensor_msgs::ImagePtr imgPtr = cvPtr->toImageMsg();
    // sensor_msgs::Image grayMsg;
    // grayMsg.header = imgMsg->header;
    // grayMsg.step = imgMsg->step / 3;
    // grayMsg.encoding = std::string("mono8");
    // grayMsg.width = imgMsg->width;
    // grayMsg.height = imgMsg->height;
    // grayMsg.is_bigendian = imgMsg->is_bigendian;
    // grayMsg.data = grayImgs[0].clone();

    // pubImg.publish(grayMsg);
    pubImg.publish(*imgPtr);
}

void imu_callback(const sensor_msgs::ImuConstPtr &imuMsg)
{
    pubImu.publish(*imuMsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosmsg_process");
    ros::NodeHandle nh;
    subImg = nh.subscribe<sensor_msgs::Image>("/thermal_image_raw", 1000, image_callback);
    subImu = nh.subscribe<sensor_msgs::Imu>("/handsfree/imu", 2000, imu_callback);
    pubImg = nh.advertise<sensor_msgs::Image>("/m2dgr/thermal_gray_image", 1000);
    pubImu = nh.advertise<sensor_msgs::Imu>("/m2dgr/imu", 2000);

    ros::spin();
    return 0;

}