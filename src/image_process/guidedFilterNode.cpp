#include <mutex>
#include <queue>
#include <thread>
#include <string>
#include <stdint.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "guided_filter.h"
#include "../../include/pl2i2-slam/utility.h"

namespace pl2i2_slam
{
    class GuidedFilter
    {
        public:
            GuidedFilter()
            {
                subImg = nh.subscribe<sensor_msgs::Image>(topic, 1000, pl2i2_slam::GuidedFilter::imageHandler, this);
                pubImg_0 = nh.advertise<sensor_msgs::Image>("/guided_filtered/iamge_0", 1000);
                pubImg_1 = nh.advertise<sensor_msgs::Image>("/guided_filtered/iamge_1", 1000);
            }
            GuidedFilter(const std::string& _topic): topic(_topic)
            {
                GuidedFilter();
            }
            void imageHandler(const sensor_msgs::ImageConstPtr &imgMsg)
            {
                {
                    std::unique_lock<std::mutex> _lock(imgMutex);
                    msgBuf.push(imgMsg);
                }
            }
            void run()
            {
                while(true)
                {
                    if(!msgBuf.empty())
                    {
                        cv_bridge::CvImagePtr cvPtr;
                        {
                            std::unique_lock<std::mutex> lock(imgMutex);
                            cvPtr = cv_bridge::toCvCopy(msgBuf.front(), sensor_msgs::image_encodings::TYPE_8UC3);
                            msgBuf.pop();
                        }
                        TicToc tictoc;
                        cv::Mat outImg_0 = dehaze(cvPtr->image);
                        ROS_INFO("[Guided Filter] img_0 : %lf", tictoc.toc());
                        tictoc.tic();
                        cv::Mat outImg_1 = dehaze(cvPtr->image, 0.1, 15, 0.4, 0.75, 0.1, 1e-3, true);
                        ROS_INFO("[Guided Filter] img_1 : %lf", tictoc.toc());

                        cvPtr->image = outImg_0;
                        sensor_msgs::ImagePtr filteredMsg_0 = cvPtr->toImageMsg();
                        cvPtr->image = outImg_1;
                        sensor_msgs::ImagePtr filteredMsg_1 = cvPtr->toImageMsg();
                        pubImg_0.publish(*filteredMsg_0);
                        pubImg_1.publish(*filteredMsg_1);

                    }


                    std::this_thread::sleep_for(std::chrono::milliseconds(3));
                }
            }

            /*
            * img: three-channel image
            */
            cv::Mat dehaze(cv::Mat img, float tmin=0.1, int w=15, float alpha=0.4, float omega=0.75, float p=0.1, float eps=1e-3, bool reduce=false)
            {
                std::pair<cv::Mat, cv::Mat> illuminate_channels = get_illumination_channel(img, w);
                cv::Mat Idark = illuminate_channels.first;
                cv::Mat Ibright = illuminate_channels.second;

                cv::Mat A = get_atmosphere(img, Ibright);
                cv::Mat init_t = get_init_transmission(A, Ibright);

                if(reduce)
                {
                    init_t = reduce_init_t(init_t);
                }

                cv::Mat corrected_t = get_corrected_transmission(img, A, Idark, Ibright, init_t, alpha, omega, w);

                cv::Mat I(img.size(), CV_32FC3), normI;

                for(int i = 0; i < img.size[1]; i++){
                    for(int j = 0; j < img.size[0]; j++){
                        I.at<cv::Vec3f>(j,i).val[0] = (float)img.at<cv::Vec3b>(j,i).val[0] / 255;
                        I.at<cv::Vec3f>(j,i).val[1] = (float)img.at<cv::Vec3b>(j,i).val[1] / 255;
                        I.at<cv::Vec3f>(j,i).val[2] = (float)img.at<cv::Vec3b>(j,i).val[2] / 255;
                    }
                }

                double minVal, maxVal;
                cv::minMaxLoc(I, &minVal, &maxVal);
                normI = (I - minVal) / (maxVal - minVal);

                // Applying guided filter
                cv::Mat refined_t = guidedFilter(normI, corrected_t, w, eps, -1);
                cv::Mat J_refined = get_final_image(I, A, refined_t, tmin);
                cv::Mat enhanced(img.size(), CV_8UC3);

                for(int i = 0; i < img.size[1]; i++){
                    for(int j = 0; j < img.size[0]; j++){
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
            
            cv::Mat get_final_image(cv::Mat I, cv::Mat A, cv::Mat refined_t, float tmin)
            {
                cv::Mat J(I.size(), CV_32FC3);

                for (int i = 0; i < refined_t.size[1]; i++)
                {
                    for (int j = 0; j < refined_t.size[0]; j++)
                    {
                        // Value of refined_t (2D refined map) at (j, i) is considered if it is >= tmin.
                        float temp = refined_t.at<float>(j, i);

                        if (temp < tmin)
                        {
                            temp = tmin;
                        }

                        // Finding result using the formula given at top
                        J.at<cv::Vec3f>(j, i).val[0] = (I.at<cv::Vec3f>(j, i).val[0] - A.at<float>(0, 0)) / temp + A.at<float>(0, 0);
                        J.at<cv::Vec3f>(j, i).val[1] = (I.at<cv::Vec3f>(j, i).val[1] - A.at<float>(1, 0)) / temp + A.at<float>(1, 0);
                        J.at<cv::Vec3f>(j, i).val[2] = (I.at<cv::Vec3f>(j, i).val[2] - A.at<float>(2, 0)) / temp + A.at<float>(2, 0);
                    }
                }

                double minVal, maxVal;
                cv::minMaxLoc(J, &minVal, &maxVal);

                // Normalize the image J
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

            cv::Mat get_corrected_transmission(cv::Mat I, cv::Mat A, cv::Mat darkch, cv::Mat brightch, cv::Mat init_t, float alpha, float omega, int w)
            {
                cv::Mat im3(I.size(), CV_32FC3);

                for(int i = 0; i < I.size[1]; i++){
                    for(int j = 0; j < I.size[0]; j++){
                        im3.at<cv::Vec3f>(j,i).val[0] = (float)I.at<cv::Vec3b>(j,i).val[0] / A.at<float>(0,0);
                        im3.at<cv::Vec3f>(j,i).val[1] = (float)I.at<cv::Vec3b>(j,i).val[1] / A.at<float>(1,0);
                        im3.at<cv::Vec3f>(j,i).val[2] = (float)I.at<cv::Vec3b>(j,i).val[2] / A.at<float>(2,0);
                    }
                }

                cv::Mat dark_c, dark_t, diffch;
                std::pair<cv::Mat, cv::Mat> illuminate_channels = get_illumination_channel(im3, w);
                dark_c = illuminate_channels.first;
                dark_t = 1 - omega * dark_c;

                cv::Mat corrected_t = init_t;
                diffch = brightch - darkch;

                for(int i = 0; i < diffch.size[1]; i++){
                    for(int j = 0; j < diffch.size[0]; j++){
                        if(diffch.at<float>(j,i) < alpha){
                            corrected_t.at<float>(j,i) = std::abs(dark_t.at<float>(j,i)*init_t.at<float>(j,i));
                        }
                    }
                }

                return corrected_t;
            }

            cv::Mat reduce_init_t(cv::Mat init_t)
            {
                cv::Mat mod_init_t(init_t.size(), CV_8UC1);

                for(int i = 0; i < init_t.size[1]; i++){
                    for(int j = 0; j < init_t.size[0]; j++){
                        mod_init_t.at<uchar>(j, i) = std::min((int)(init_t.at<float>(j,i)*255), 255);
                    }
                }

                int x[3] = {0, 32, 255};
                int f[3] = {0, 32, 48};

                cv::Mat table(cv::Size(1,256), CV_8UC1);

                int l = 0;
                for(int k = 0; k < 256; k++)
                {
                    if(k > x[l+1]) l = l + 1;

                    float m = (float)(f[l+1] - f[l]) / (x[l+1] - x[l]);
                    table.at<uchar>(k,0) = (int)(f[l] + m*(k - x[l]));
                }

                cv::LUT(mod_init_t, table, mod_init_t);

                for(int i = 0; i < init_t.size[1]; i++){
                    for(int j = 0; j < init_t.size[0]; j++){
                        init_t.at<float>(j,i) = (float)mod_init_t.at<uchar>(j,i) / 255;
                    }
                }

                return init_t;
            }

            cv::Mat get_init_transmission(cv::Mat A, cv::Mat brightch)
            {
                double A_n, A_x, min_val, max_val;
                cv::minMaxLoc(A, &A_n, &A_x);
                cv::Mat init_t(brightch.size(), CV_32FC1);
                init_t = brightch.clone();

                init_t = (init_t - A_x) / (1.0 - A_x);

                cv::minMaxLoc(init_t, &min_val, &max_val);
                init_t = (init_t - min_val) / (max_val - min_val);

                return init_t;
            }

            /*
            * 作用：计算全局照明环境
            *   
            * 
            */
            cv::Mat get_atmosphere(cv::Mat I, cv::Mat brightch, float p=0.1)
            {
                int height = brightch.size[0];
                int width = brightch.size[1];

                cv::Mat flatI(cv::Size(1, height*width), CV_8UC3);
                std::vector<std::pair<float,int>> flatBright;

                // 按列储存原像素
                for(int i = 0; i < width; i++){
                    for(int j = 0; j < height; j++){
                        int index = i * height + j;
                        flatI.at<cv::Vec3b>(index,0).val[0] = I.at<cv::Vec3b>(j,i).val[0];
                        flatI.at<cv::Vec3b>(index,0).val[1] = I.at<cv::Vec3b>(j,i).val[1];
                        flatI.at<cv::Vec3b>(index,0).val[2] = I.at<cv::Vec3b>(j,i).val[2];
                    
                        flatBright.push_back(std::make_pair(-brightch.at<float>(j,i), index));
                    }
                }

                sort(flatBright.begin(), flatBright.end());

                cv::Mat A = cv::Mat::zeros(cv::Size(1,3), CV_32FC1);

                for(int k=0; k < int(height * width * p); k++){
                    int sindex = flatBright[k].second;
                    A.at<float>(0,0) = A.at<float>(0,0) + (float)flatI.at<cv::Vec3b>(sindex,0).val[0];
                    A.at<float>(1,0) = A.at<float>(1,0) + (float)flatI.at<cv::Vec3b>(sindex,0).val[1];
                    A.at<float>(2,0) = A.at<float>(2,0) + (float)flatI.at<cv::Vec3b>(sindex,0).val[2];

                }

                A = A / int(height * width * p);

                return A / 255;
            }

            /*
            *  参数： w -> window size
            */
            std::pair<cv::Mat,cv::Mat> get_illumination_channel(cv::Mat I, float w)
            {
                int height = I.size[0];
                int width = I.size[1];

                cv::Mat darkch = cv::Mat::zeros(cv::Size(width, height), CV_32FC1);
                cv::Mat brightch = cv::Mat::zeros(cv::Size(width, height), CV_32FC1);

                int padding = int(w / 2);
                cv::Mat padded = cv::Mat::zeros(cv::Size(width + padding * 2, height + padding * 2), CV_32FC3);

                if(I.type() == CV_8UC3)
                {
                    for(int i = padding; i < padding + width; i++){
                        for(int j = padding; j < padding + height; j++){
                            padded.at<cv::Vec3f>(j, i).val[0] = (float)I.at<cv::Vec3b>(j - padding, i - padding).val[0] / 255;
                            padded.at<cv::Vec3f>(j, i).val[1] = (float)I.at<cv::Vec3b>(j - padding, i - padding).val[1] / 255;
                            padded.at<cv::Vec3f>(j, i).val[2] = (float)I.at<cv::Vec3b>(j - padding, i - padding).val[2] / 255;
                        }
                    }
                }
                else
                {
                    for(int i = padding; i < padding + width; i++){
                        for(int j = padding; j < padding + height; j++){
                            padded.at<cv::Vec3f>(j, i).val[0] = (float)I.at<cv::Vec3f>(j - padding, i - padding).val[0] / 255;
                            padded.at<cv::Vec3f>(j, i).val[1] = (float)I.at<cv::Vec3f>(j - padding, i - padding).val[1] / 255;
                            padded.at<cv::Vec3f>(j, i).val[2] = (float)I.at<cv::Vec3f>(j - padding, i - padding).val[2] / 255;
                        }
                    }
                }

                for(int i = 0; i < darkch.size[1]; i++)
                {
                    int col_up, row_up;
                    col_up = int(i + w);

                    for(int j = 0; j < darkch.size[0]; j++)
                    {
                        double min_val, max_val;
                        row_up = int(j + w);

                        cv::minMaxLoc(padded.colRange(i, col_up).rowRange(j, row_up), &min_val, &max_val);
                        darkch.at<float>(j,i) = min_val;
                        brightch.at<float>(j,i) = max_val;
                    }
                }

                return std::make_pair(darkch,brightch);
            }
            

        private:
            std::string topic = "/thermal_image_raw";
            ros::Subscriber subImg;
            ros::Publisher pubImg_0;
            ros::Publisher pubImg_1;
            ros::NodeHandle nh;
            std::queue<sensor_msgs::ImageConstPtr> msgBuf;
            std::mutex imgMutex;
    };
}



int32_t main(int32_t argc, char **argv)
{
    ros::init(argc, argv, "guided_filter");
    ros::NodeHandle nh;
    GuidedFilter guidedFilter("/thermal_image_raw");

    ros::spin();
    return 0;

}
