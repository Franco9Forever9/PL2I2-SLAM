#include <vector>
#include <string>
#include <stdint.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "guided_filter.h"

namespace pl2i2_slam
{
    class GuidedFilter
    {
        public:
            GuidedFilter(std::string &_topic) : topic(_topic)
            {
                subImg = nh.subscribe<sensor_msgs::Image>(topic, 1000, pl2i2_slam::GuidedFilter::imageHandler, this);
                pubImg = nh.advertise<sensor_msgs::Image>("/guided_filtered/iamge", 1000);
            }
            void imageHandler(const sensor_msgs::ImageConstPtr &imgMsg)
            {

            }

        private:
            std::string topic;
            ros::Subscriber subImg;
            ros::Publisher pubImg;
            ros::NodeHandle nh;
            std::vector<sensor_msgs::ImageConstPtr> msgBuf;
    };
}



int32_t main(int32_t argc, char **argv)
{
    ros::init(argc, argv, "guided_filter");
    ros::NodeHandle nh;

}
