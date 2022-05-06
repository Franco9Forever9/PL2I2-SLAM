#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <iostream>


#include "../../include/pl2i2-slam/tic_toc.h"

namespace pl2i2_slam
{
    class ImageFilter
    {
        public:
            ImageFilter(const std::string& _subTopic, const int _picGap) : subTopic(_subTopic), picGap(_picGap)
            {
                subImg = nh.subscribe<sensor_msgs::Image>(this->subTopic, 1000, ImageFilter::imageHandler, this);
                pubImg_Gamma_CLAHE = nh.advertise<sensor_msgs::Image>("/thermal/gamma_clahe", 1000);
                this->tProcess = std::thread(ImageFilter::run, this);
            }

            void imageHandler(const sensor_msgs::ImageConstPtr &imgMsg)
            {
                {
                    std::unique_lock<std::mutex> _lock(queueMux);
                    imgQueue.push(imgMsg);
                }
            }

            void run()
            {
                while (true)
                {
                    if (!imgQueue.empty())
                    {
                        cv_bridge::CvImagePtr cvPtr;
                        {
                            std::unique_lock<std::mutex> lock(queueMux);
                            cvPtr = cv_bridge::toCvCopy(imgQueue.front(), sensor_msgs::image_encodings::TYPE_8UC3);
                            output_intermediate_image(cvPtr->image, "raw");
                            imgQueue.pop();
                        }
                        TicToc tictoc;
                        cv::Mat gammaCLAHEImg = get_gamma_clahe(cvPtr->image, 0.5);

                        this->picCount += 1;
                        if (this->picCount > this->picGap)
                            this->picCount = 0;
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(3));
                }
            }

            cv::Mat get_gamma_clahe(const cv::Mat &srcImg, double gamma = 0.5,double clipLimit = 40.0, int gridSize = 8)
            {
                cv::Mat dstImg; TicToc _clock;
                get_current_dir_name(srcImg, dstImg, gamma);
                output_intermediate_image(dstImg, "gamma_transform");
                ROS_INFO("[Filter Process] gamma transform: %lf ms", _clock.toc());
            
                _clock.tic()
                cv::Mat gammaCLAHEImg = get_clahe_process(dstImg, clipLimit, gridSize);
                output_intermediate_image(dstImg, "clahe_process");
                ROS_INFO("[Filter Process] clahe process: %lf ms", _clock.toc());

            }

            cv::Mat get_clahe_process(cv::Mat &srcImg, double clipLimit = 40.0, int gridSize = 8)
            {
                cv::Mat ycrcbImg = srcImg.clone();
                std::vector<cv::Mat> channels;
                cv::cvtColor(ycrcbImg, ycrcbImg,cv::COLOR_BGR2YCrCb);
                cv::split(ycrcbImg, channels);

                cv::Mat claheImg;
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->setClipLimit(clipLimit);
                clahe->setTilesGridSize(cv::Size(gridSize, gridSize));
                clahe->apply(channels[0],claheImg);
                channels[0].release();
                claheImg.copyTo(channels[0]);

                cv::merge(ycrcbImg, channels);
                return ycrcbImg;
            }

            void get_gamma_transform(const cv::Mat &srcImg, cv::Mat &dstImg, double gamma)
            {
                unsigned char levels[256];

                for(int i = 0; i < 256; i++)
                    levels[i] = std::saturate_cast<uchar>(std::pow((float)i/255.0, gamma) * 255.0f);

                dstImg = srcImg.clone();
                
                if(srcImg.channels() == 1)
                {
                    cv::MatIterator_<uchar> imgBegin = dstImg.begin<uchar>();
                    cv::MatIterator_<uchar> imgEnd = dstImg.end<uchar>();
                    while(imgBegin != imgEnd)
                    {
                        *imgBegin = levels[*imgBegin];
                        imgBegin++;
                    }
                }
                else{
                    cv::MatIterator_<cv::Vec3b> imgBegin = dstImg.begin<cv::Vec3b>();
                    cv::MatIterator_<cv::Vec3b> imgEnd = dstImg.end<cv::Vec3b>();
                    while(imgBegin != imgEnd)
                    {
                        (*imgBegin)[0] = levels[(*imgBegin)[0]];
                        (*imgBegin)[1] = levels[(*imgBegin)[1]];
                        (*imgBegin)[2] = levels[(*imgBegin)[2]];
                    }
                }
                return;
            }

            

            void output_intermediate_image(cv::Mat &img, const std::string &name)
            {
                std::string outPath = this->outFolder + std::to_string(picNum) + name + std::string(".jpg");
                cv::imwrite(outPath, img);
            }

        private:
            int picNum;
            int picGap;
            int picCount;

            std::string subTopic;
            std::string outFolder = "/home/jgl/Documents/Projects/PL2I2-SLAM/src/PL2I2-SLAM/imgs/image_filter/";
            std::queue<sensor_msgs::ImageConstPtr> imgQueue;
            std::mutex queueMux;
            std::thread tProcess;

            ros::NodeHandle nh;
            ros::Subscriber subImg;
            ros::Publisher pubImg_Gamma_CLAHE;


    };
}

int32_t main(int32_t argc, char** argv)
{
    ros::init(argc, argv, "opencv_filter");
    ros::NodeHandle nh;

    pl2i2_slam::ImageFilter imageFilter("/thermal_image_raw", 100);
    ROS_INFO("------> [Image Filter] <------");

    ros::spin();
    return 0;



}