#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>
#include "relative_nav_msgs/Keyframe.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include "image.h"
#include "misc.h"
#include "segment-image.h"

#define DSC 2
#define DSR 1

using namespace cv;

ros::Publisher image_pub;
ros::Publisher depth_pub;

double fx, fy, cx, cy;
Mat cameraMatrix, distCoeffs;
Mat ave_image, weight_image;

bool isNAN(float val) { return (val != val) ? true : false; }

void keyframeCallback(const relative_nav_msgs::Keyframe& msg)
{
    sensor_msgs::Image image_msg;
    sensor_msgs::Image depth_msg;

    image_msg = msg.rgb;
    depth_msg = msg.depth;

    cv_bridge::CvImagePtr cv_img_ptr, cv_dpth_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image_msg, image_msg.encoding);
        cv_dpth_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg.encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Mat cv_img = cv_img_ptr->image;
    Mat cv_dpth = cv_dpth_ptr->image;

    //    ROS_INFO("rows: %d, cols: %d", cv_dpth.rows, cv_dpth.cols); // 640 x 480
    int rows = cv_dpth.rows;
    int cols = cv_dpth.cols;

    image<rgbdu> *im = new image<rgbdu>(cv_img.cols, cv_img.rows);
    for(int i=0;i<cv_img.rows;i++)
    {
        for(int j=0;j<cv_img.cols;j++)
        {

            rgbdu* p = imPtr(im, j, i);
            p->r = cv_img.at<cv::Vec3b>(i,j)[2];
            p->g = cv_img.at<cv::Vec3b>(i,j)[1];
            p->b = cv_img.at<cv::Vec3b>(i,j)[0];
            p->d = cv_dpth.at<float>(i,j);
            if(p->d != p->d)
                p->u = 18.0;
            else
            {
                p->u = .2*p->d;
                if(i<rows-1 && !isNAN(cv_dpth.at<float>(i+1,j)))
                    p->u += .1*fabs(p->d - cv_dpth.at<float>(i+1,j));
                if(j<cols-1 && !isNAN(cv_dpth.at<float>(i,j+1)))
                    p->u += .1*fabs(p->d - cv_dpth.at<float>(i,j+1));
                if(i>0 && !isNAN(cv_dpth.at<float>(i-1,j)))
                    p->u += .1*fabs(p->d - cv_dpth.at<float>(i-1,j));
                if(j>0 && !isNAN(cv_dpth.at<float>(i,j-1)))
                    p->u += .1*fabs(p->d - cv_dpth.at<float>(i,j-1));

                if(i<rows-2 && !isNAN(cv_dpth.at<float>(i+2,j)))
                    p->u += .05*fabs(p->d - cv_dpth.at<float>(i+2,j));
                if(j<cols-2 && !isNAN(cv_dpth.at<float>(i,j+2)))
                    p->u += .05*fabs(p->d - cv_dpth.at<float>(i,j+2));
                if(i>1 && !isNAN(cv_dpth.at<float>(i-1,j)))
                    p->u += .05*fabs(p->d - cv_dpth.at<float>(i+2,j));
                if(j>1 && !isNAN(cv_dpth.at<float>(i,j-2)))
                    p->u += .05*fabs(p->d - cv_dpth.at<float>(i,j-2));
            }
        }
    }

//    for(int i=0;i<cv_img.rows;i++)
//    {
//        for(int j=0;j<cv_img.cols;j++)
//        {
//            rgbdu* p = imPtr(im, j, i);
//            cv_dpth.at<float>(i,j) = p->u;
//        }
//    }

    float sigma = 0.65;
    float k = 150;
    int min_size = 16500;
    int num_ccs;
    image<rgb> *seg = segment_image(im, sigma, k, 500, &num_ccs);

    Mat cv_img_out = Mat(cv_img.rows, cv_img.cols, cv_img.type());

    for(int i=0;i<cv_img_out.rows;i++)
    {
        for(int j=0;j<cv_img_out.cols;j++)
        {
            rgb* p = imPtr(seg, j, i);
            cv_img_out.at<cv::Vec3b>(i,j)[2] = p->r;
            cv_img_out.at<cv::Vec3b>(i,j)[1] = p->g;
            cv_img_out.at<cv::Vec3b>(i,j)[0] = p->b;
        }
    }


    // convert a depth image to show with opencv
//    Mat show;
//    cv_dpth_blur.convertTo(show,CV_8U,255.0/(max-min));
//    Size k(3,3);
    imshow("Image window", cv_img_out);
    waitKey(3);

    //---- publish the devided keyframe
    image_pub.publish(cv_img_ptr->toImageMsg());
    depth_pub.publish(cv_dpth_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
    ave_image = Mat(480, 640, CV_64F);
    ave_image.zeros(480, 640, CV_64F);
    weight_image = Mat(480, 640, CV_64F);
    weight_image.zeros(480, 640, CV_64F);

    fx = 573.164592404304;
    fy = 574.057033722257;
    cx = 325.529294950075;
    cy = 240.110513622751;
    double cm[3][3] = {{573.164592404304, 0, 325.529294950075}, {0, 574.057033722257, 240.110513622751}, {0, 0, 1}};
    cameraMatrix = Mat(3, 3, CV_64F, cm);

    double d[5] = {-0.0343491737811583, 0.0207245817239776, -0.000162958979267582, 0.00314423882678934, 0};
    distCoeffs = Mat(5, 1, CV_64F, d);

    cv::namedWindow("Image window", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Image window 2", CV_WINDOW_AUTOSIZE);
    ros::init(argc, argv, "keyframe_devide");
    ros::NodeHandle n;
    image_pub = n.advertise<sensor_msgs::Image>("image", 1);
    depth_pub = n.advertise<sensor_msgs::Image>("depth", 1);

    ros::Subscriber sub = n.subscribe("keyframe", 1, keyframeCallback);

    //ROS_INFO("gpu's %d",gpu::getCudaEnabledDeviceCount());
    ros::spin();

    return 0;
}
