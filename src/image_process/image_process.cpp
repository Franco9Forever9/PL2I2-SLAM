#include <stack>
#include <mutex>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/photo.hpp>
#include "guided_filter.h"

namespace pl2i2_slam
{
    class ImageProcess
    {
        public:
            ImageProcess(const std::string &topic)
            {
                this->imgTopic = topic;
                subRaw = nh.subscribe<sensor_msgs::Image>(imgTopic, 1000, &ImageProcess::imageHandler, this, ros::TransportHints().tcpNoDelay());
                pubBlue = nh.advertise<sensor_msgs::Image>("/thermal/blue", 1000);
                pubRed = nh.advertise<sensor_msgs::Image>("/thermal/red", 1000);
                pubGreen = nh.advertise<sensor_msgs::Image>("/thermal/green", 1000);
                pubCV = nh.advertise<sensor_msgs::Image>("/thermal/CV", 1000);
            }

            void imageHandler(const sensor_msgs::ImageConstPtr &imgMsg)
            {
                // pubSingleChannels(imgMsg);
                // pubBCCEProcess(imgMsg);
                imageCVProcess(imgMsg);

            }

            void imageCVProcess(const sensor_msgs::ImageConstPtr &imgMsg)
            {
                if(imgMsg->encoding == "8UC3")
                {
                    cv_bridge::CvImageConstPtr cvPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::TYPE_8UC3);
                    cv::Mat grayImg;
                    // cv::detailEnhance(cvPtr->image, grayImg, 10, 0.15);
                    sensor_msgs::ImagePtr imgPtr = cv_bridge::CvImage(imgMsg->header, "8UC3", grayImg).toImageMsg();
                    pubCV.publish(*imgPtr);
                }
            }

            void pubBCCEProcess(const sensor_msgs::ImageConstPtr &imgMsg)
            {
                cv_bridge::CvImageConstPtr cvPtr;
                if(imgMsg->encoding == "8UC3")
                {
                    cvPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::TYPE_8UC3);
                    std::vector<cv::Mat> grayImgs(cvPtr->image.channels());
                    cv::split(cvPtr->image, grayImgs);

                    cv::Mat medianImg = cv::Mat::zeros(cvPtr->image.rows, cvPtr->image.cols, CV_8UC1);
                    cv::Mat ContrastImg = cv::Mat::zeros(cvPtr->image.rows, cvPtr->image.cols, CV_8UC1);
                    cv::medianBlur(grayImgs[0], medianImg, 5);
                    for(int row = 0; row < cvPtr->image.rows; row++){
                        for(int col = 0; col < cvPtr->image.rows; col++){
                            ContrastImg.at<uchar>(row,col) = 1.0 * (double)grayImgs[0].at<uchar>(row,col) / medianImg.at<uchar>(row,col);
                        }
                    }
                    
                }
            }

            std::pair<cv::Mat, cv::Mat> get_illumination_channel(cv::Mat I, float w)
            {
                int N = I.size[0];
                int M = I.size[1];
                cv::Mat darkch = cv::Mat::zeros(cv::Size(M, N), CV_32FC1);
                cv::Mat brightch = cv::Mat::zeros(cv::Size(M, N), CV_32FC1);

                int padding = int(w/2);
                cv::Mat padded = cv::Mat::zeros(cv::Size(M + 2*padding, N + 2*padding), CV_32FC3);

                for(int i = padding; i < padding + M; i++){
                    for(int j = padding; j < padding + N; j++){
                        padded.at<cv::Vec3f>(j,i).val[0] = (float)I.at<cv::Vec3b>(j - padding, i - padding).val[0]/255;
                        padded.at<cv::Vec3f>(j,i).val[0] = (float)I.at<cv::Vec3b>(j - padding, i - padding).val[1]/255;
                        padded.at<cv::Vec3f>(j,i).val[0] = (float)I.at<cv::Vec3b>(j - padding, i - padding).val[2]/255;
                    }
                }

                for (int i = 0; i < darkch.size[1]; i++){
                    int col_up, row_up;

                    col_up = int(i + w);

                    for (int j = 0; j < darkch.size[0]; j++){
                        double minVal, maxVal;

                        row_up = int(j + w);

                        cv::minMaxLoc(padded.colRange(i, col_up).rowRange(j, row_up), &minVal, &maxVal);

                        darkch.at<float>(j, i) = minVal;   // dark channel
                        brightch.at<float>(j, i) = maxVal; // bright channel
                    }
                }

                return std::make_pair(darkch, brightch);
            }

            cv::Mat get_atmosphere(cv::Mat I, cv::Mat brightch, float p = 0.1)
            {
                int N = brightch.size[0];
                int M = brightch.size[1];

                // flattening and reshaping image array
                cv::Mat flatI(cv::Size(1, N * M), CV_8UC3);
                std::vector<std::pair<float, int>> flatBright;

                for (int i = 0; i < M; i++){
                    for (int j = 0; j < N; j++){
                        int index = i * N + j;
                        flatI.at<cv::Vec3b>(index, 0).val[0] = I.at<cv::Vec3b>(j, i).val[0];
                        flatI.at<cv::Vec3b>(index, 0).val[1] = I.at<cv::Vec3b>(j, i).val[1];
                        flatI.at<cv::Vec3b>(index, 0).val[2] = I.at<cv::Vec3b>(j, i).val[2];

                        flatBright.push_back(std::make_pair(-brightch.at<float>(j, i), index));
                    }
                }

                // sorting and slicing the array
                sort(flatBright.begin(), flatBright.end());

                cv::Mat A = cv::Mat::zeros(cv::Size(1, 3), CV_32FC1);

                for (int k = 0; k < int(M * N * p); k++)
                {
                    int sindex = flatBright[k].second;
                    A.at<float>(0, 0) = A.at<float>(0, 0) + (float)flatI.at<cv::Vec3b>(sindex, 0).val[0];
                    A.at<float>(1, 0) = A.at<float>(1, 0) + (float)flatI.at<cv::Vec3b>(sindex, 0).val[1];
                    A.at<float>(2, 0) = A.at<float>(2, 0) + (float)flatI.at<cv::Vec3b>(sindex, 0).val[2];
                }

                A = A / int(M * N * p);

                return A / 255;
            }

            cv::Mat get_initial_transmission(cv::Mat A, cv::Mat brightch)
            {
                double A_n, A_x, minVal, maxVal;
                cv::minMaxLoc(A, &A_n, &A_x);
                cv::Mat init_t(brightch.size(), CV_32FC1);
                init_t = brightch.clone();
                // finding initial transmission map
                init_t = (init_t - A_x) / (1.0 - A_x);
                cv::minMaxLoc(init_t, &minVal, &maxVal);
                // normalized initial transmission map
                init_t = (init_t - minVal) / (maxVal - minVal);

                return init_t;
            }

            cv::Mat get_corrected_transmission(cv::Mat I, cv::Mat A, cv::Mat darkch, cv::Mat brightch, cv::Mat init_t, float alpha, float omega, int w)
            {
                cv::Mat im3(I.size(), CV_32FC3);
                // divide pixel values by atmospheric light
                for (int i = 0; i < I.size[1]; i++)
                {
                    for (int j = 0; j < I.size[0]; j++)
                    {
                        im3.at<cv::Vec3f>(j, i).val[0] = (float)I.at<cv::Vec3b>(j, i).val[0] / A.at<float>(0, 0);
                        im3.at<cv::Vec3f>(j, i).val[1] = (float)I.at<cv::Vec3b>(j, i).val[1] / A.at<float>(1, 0);
                        im3.at<cv::Vec3f>(j, i).val[2] = (float)I.at<cv::Vec3b>(j, i).val[2] / A.at<float>(2, 0);
                    }
                }

                cv::Mat dark_c, dark_t, diffch;

                std::pair<cv::Mat, cv::Mat> illuminate_channels = get_illumination_channel(im3, w);
                // dark channel transmission map
                dark_c = illuminate_channels.first;
                // corrected dark transmission map
                dark_t = 1 - omega * dark_c;
                cv::Mat corrected_t = init_t;
                diffch = brightch - darkch; // difference between transmission maps

                for (int i = 0; i < diffch.size[1]; i++)
                {
                    for (int j = 0; j < diffch.size[0]; j++)
                    {
                        if (diffch.at<float>(j, i) < alpha)
                        {
                            // initializing corrected transmission map with initial transmission map
                            corrected_t.at<float>(j, i) = abs(dark_t.at<float>(j, i) * init_t.at<float>(j, i));
                        }
                    }
                }

                return corrected_t;
            }

            cv::Mat get_final_image(cv::Mat I, cv::Mat A, cv::Mat refined_t, float tmin)
            {
                cv::Mat J(I.size(), CV_32FC3);

                for (int i = 0; i < refined_t.size[1]; i++)
                {
                    for (int j = 0; j < refined_t.size[0]; j++)
                    {
                        float temp = refined_t.at<float>(j, i);

                        if (temp < tmin)
                        {
                            temp = tmin;
                        }
                        // finding result
                        J.at<cv::Vec3f>(j, i).val[0] = (I.at<cv::Vec3f>(j, i).val[0] - A.at<float>(0, 0)) / temp + A.at<float>(0, 0);
                        J.at<cv::Vec3f>(j, i).val[1] = (I.at<cv::Vec3f>(j, i).val[1] - A.at<float>(1, 0)) / temp + A.at<float>(1, 0);
                        J.at<cv::Vec3f>(j, i).val[2] = (I.at<cv::Vec3f>(j, i).val[2] - A.at<float>(2, 0)) / temp + A.at<float>(2, 0);
                    }
                }

                double minVal, maxVal;
                cv::minMaxLoc(J, &minVal, &maxVal);

                // normalized image
                for (int i = 0; i < J.size[1]; i++)
                {
                    for (int j = 0; j < J.size[0]; j++)
                    {
                        J.at<cv::Vec3f>(j, i).val[0] = (J.at<cv::Vec3f>(j, i).val[0] - minVal) / (maxVal - minVal);
                        J.at<cv::Vec3f>(j, i).val[1] = (J.at<cv::Vec3f>(j, i).val[1] - minVal) / (maxVal - minVal);
                        J.at<cv::Vec3f>(j, i).val[2] = (J.at<cv::Vec3f>(j, i).val[2] - minVal) / (maxVal - minVal);
                    }
                }

                return J;
            }

            cv::Mat reduce_init_t(cv::Mat init_t)
            {
                cv::Mat mod_init_t(init_t.size(), CV_8UC1);

                for (int i = 0; i < init_t.size[1]; i++)
                {
                    for (int j = 0; j < init_t.size[0]; j++)
                    {
                        mod_init_t.at<uchar>(j, i) = std::min((int)(init_t.at<float>(j, i) * 255), 255);
                    }
                }

                int x[3] = {0, 32, 255};
                int f[3] = {0, 32, 48};

                // creating array [0,...,255]
                cv::Mat table(cv::Size(1, 256), CV_8UC1);

                // Linear Interpolation
                int l = 0;
                for (int k = 0; k < 256; k++)
                {
                    if (k > x[l + 1])
                    {
                        l = l + 1;
                    }

                    float m = (float)(f[l + 1] - f[l]) / (x[l + 1] - x[l]);
                    table.at<int>(k, 0) = (int)(f[l] + m * (k - x[l]));
                }


                // Lookup table
                cv::LUT(mod_init_t, table, mod_init_t);

                for (int i = 0; i < init_t.size[1]; i++)
                {
                    for (int j = 0; j < init_t.size[0]; j++)
                    {
                        // normalizing the transmission map
                        init_t.at<float>(j, i) = (float)mod_init_t.at<uchar>(j, i) / 255;
                    }
                }

                return init_t;
            }

            cv::Mat dehaze(cv::Mat img, float tmin = 0.1, int w = 15, float alpha = 0.4, float omega = 0.75, float p = 0.1, double eps = 1e-3, bool reduce = false)
            {
                std::pair<cv::Mat, cv::Mat> illuminate_channels = get_illumination_channel(img, w);
                cv::Mat Idark = illuminate_channels.first;
                cv::Mat Ibright = illuminate_channels.second;

                cv::Mat A = get_atmosphere(img, Ibright);

                cv::Mat init_t = get_initial_transmission(A, Ibright);

                if (reduce)
                {
                    init_t = reduce_init_t(init_t);
                }

                cv::Mat corrected_t = get_corrected_transmission(img, A, Idark, Ibright, init_t, alpha, omega, w);

                cv::Mat I(img.size(), CV_32FC3), normI;

                for (int i = 0; i < img.size[1]; i++)
                {
                    for (int j = 0; j < img.size[0]; j++)
                    {
                        I.at<cv::Vec3f>(j, i).val[0] = (float)img.at<cv::Vec3b>(j, i).val[0] / 255;
                        I.at<cv::Vec3f>(j, i).val[1] = (float)img.at<cv::Vec3b>(j, i).val[1] / 255;
                        I.at<cv::Vec3f>(j, i).val[2] = (float)img.at<cv::Vec3b>(j, i).val[2] / 255;
                    }
                }

                double minVal, maxVal;
                cv::minMaxLoc(I, &minVal, &maxVal);
                normI = (I - minVal) / (maxVal - minVal);

                // Applying guided filter
                // cv::Mat refined_t(normI.size(), CV_32FC1);
                cv::Mat refined_t;
                refined_t = guidedFilter(normI, corrected_t, w, eps, -1);

                cv::Mat J_refined = get_final_image(I, A, refined_t, tmin);

                cv::Mat enhanced(img.size(), CV_8UC3);

                for (int i = 0; i < img.size[1]; i++)
                {
                    for (int j = 0; j < img.size[0]; j++)
                    {
                        enhanced.at<cv::Vec3b>(j, i).val[0] = std::min((int)(J_refined.at<cv::Vec3f>(j, i).val[0] * 255), 255);
                        enhanced.at<cv::Vec3b>(j, i).val[1] = std::min((int)(J_refined.at<cv::Vec3f>(j, i).val[1] * 255), 255);
                        enhanced.at<cv::Vec3b>(j, i).val[2] = std::min((int)(J_refined.at<cv::Vec3f>(j, i).val[2] * 255), 255);
                    }
                }

                cv::Mat f_enhanced;
                cv::detailEnhance(enhanced, f_enhanced, 10, 0.15);
                cv::edgePreservingFilter(f_enhanced, f_enhanced, 1, 64, 0.2);

                return f_enhanced;
            }

            void brightness_enhancement(const std::string &path)
            {
                cv::Mat img = cv::imread(path);
                cv::Mat out_img = dehaze(img);
                cv::Mat out_img2 = dehaze(img, 0.1, 15, 0.4, 0.75, 0.1, 1e-3, true);
                std::string outPath = "/home/jgl/Documents/Projects/PL2I2-SLAM/src/PL2I2-SLAM/imgs/thermal_filter/out.jpg";
                cv::imwrite(outPath, out_img2);
            }

            cv::Mat balancedCLAHE(const cv::Mat &img)
            {
                int row = img.rows, col = img.cols;

            }

            cv::Mat histStatisticEnhancement(const cv::Mat &input, int size, double k0, double k1, double k2, double E, bool dark)
            {
                if(size % 2 == 0)
                {
                    std::cout << " -- > 滤波器不为奇数，请修改！ < -- " << std::endl;
                }

                double mean_g = 0, sigma_g = 0;

            }

            void pubSingleChannels(const sensor_msgs::ImageConstPtr &imgMsg)
            {
                cv_bridge::CvImageConstPtr cvPtr;
                if(imgMsg->encoding == "8UC3")
                {
                    cvPtr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::TYPE_8UC3);

                    cv::Mat redMat = cv::Mat::zeros(cvPtr->image.rows, cvPtr->image.cols, CV_8UC1);
                    cv::Mat blueMat = cv::Mat::zeros(cvPtr->image.rows, cvPtr->image.cols, CV_8UC1);
                    cv::Mat greenMat = cv::Mat::zeros(cvPtr->image.rows, cvPtr->image.cols, CV_8UC1);
                    for(int row = 0; row < cvPtr->image.rows; row++){
                        for(int col = 0; col < cvPtr->image.cols; col++){
                            redMat.at<uchar>(row,col) = cvPtr->image.at<cv::Vec3b>(row,col)[0];
                            blueMat.at<uchar>(row,col) = cvPtr->image.at<cv::Vec3b>(row,col)[1];
                            greenMat.at<uchar>(row,col) = cvPtr->image.at<cv::Vec3b>(row,col)[2];
                            if(cvPtr->image.at<cv::Vec3b>(row,col)[0] != cvPtr->image.at<cv::Vec3b>(row,col)[1] ||
                                cvPtr->image.at<cv::Vec3b>(row,col)[0] != cvPtr->image.at<cv::Vec3b>(row,col)[2])
                                ROS_INFO(" -- > Differs in Channels < -- ");
                        }
                    }

                    sensor_msgs::ImagePtr rmsgPtr = cv_bridge::CvImage(imgMsg->header, "mono8",  redMat).toImageMsg();
                    sensor_msgs::ImagePtr bmsgPtr = cv_bridge::CvImage(imgMsg->header, "mono8",  blueMat).toImageMsg();
                    sensor_msgs::ImagePtr gmsgPtr = cv_bridge::CvImage(imgMsg->header, "mono8",  greenMat).toImageMsg();
                    
                    pubRed.publish(*rmsgPtr);
                    pubGreen.publish(*gmsgPtr);
                    pubBlue.publish(*bmsgPtr);
                }
            }
        
        private:
            ros::NodeHandle nh;
            ros::Subscriber subRaw;
            ros::Publisher pubCV;
            ros::Publisher pubRed;
            ros::Publisher pubBlue;
            ros::Publisher pubGreen;
            std::string imgTopic;
            
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image");

    std::mutex mtx;

    pl2i2_slam::ImageProcess imageProcess("/thermal_image_raw");

    ROS_INFO("--- > Image Process Start < ---");

    ros::spin();

    return 0;
}