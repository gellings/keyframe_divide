#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>
#include "relative_nav_msgs/Keyframe.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include "vertex.h"

#include <fstream>

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
    sensor_msgs::Image image;
    sensor_msgs::Image depth;

    image = msg.rgb;
    depth = msg.depth;

    cv_bridge::CvImagePtr cv_img_ptr, cv_dpth_ptr;
    try
    {
        cv_img_ptr = cv_bridge::toCvCopy(image, image.encoding);
        cv_dpth_ptr = cv_bridge::toCvCopy(depth, depth.encoding);
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

    float edge_threshold = 0.2;

    //----- averaging stuff
    for(int i(0);i<rows;i++)
    {
        for(int j(0);j<cols;j++)
        {
            if(!isNAN(cv_dpth.at<float>(i,j)) && (weight_image.at<double>(i,j) < 3.0 || fabs((float)ave_image.at<double>(i,j) - cv_dpth.at<float>(i,j)) < edge_threshold))
            {
                ave_image.at<double>(i,j) = ( weight_image.at<double>(i,j)*ave_image.at<double>(i,j) + cv_dpth.at<float>(i,j) )/(weight_image.at<double>(i,j) + 1);
                weight_image.at<double>(i,j) += 1.0;
            }
        }
    }

    //depth image blur
    Mat cv_dpth_blur;
    //blur(ave_image,cv_dpth_blur,Size(5,5));
    ave_image.copyTo(cv_dpth_blur);
//    for(int i(0);i<rows;i++)
//    {
//        for(int j(0);j<cols;j++)
//        {

//            if(!(cv_dpth.at<float>(i,j) != cv_dpth.at<float>(i,j))) //not null
//            {
//                int div = 1;
//                float sum = cv_dpth.at<float>(i,j);
//                if(i > DSR && !(cv_dpth.at<float>(i-DSR,j) != cv_dpth.at<float>(i-DSR,j))) //not null
//                {   sum += cv_dpth.at<float>(i-DSR,j); div++;  }
//                if(j > DSC && !(cv_dpth.at<float>(i,j-DSC) != cv_dpth.at<float>(i,j-DSC))) //not null
//                {   sum += cv_dpth.at<float>(i,j-DSC); div++;  }
//                if(i < rows - DSR && !(cv_dpth.at<float>(i+DSR,j) != cv_dpth.at<float>(i+DSR,j))) //not null
//                {   sum += cv_dpth.at<float>(i+DSR,j); div++;  }
//                if(j < cols - DSC && !(cv_dpth.at<float>(i,j+DSC) != cv_dpth.at<float>(i,j+DSC))) //not null
//                {   sum += cv_dpth.at<float>(i,j+DSC); div++;  }
//                float val = sum/div;
//                cv_dpth_blur.at<float>(i,j) = val;
//            }
//        }
//    }


    //----- Mesh file creation
    std::ofstream meshfile("mesh.smf");
    if(meshfile.is_open())
    {
        meshfile << "#$SMF 1.0\n";
    }

    Vertex* verticies[rows*cols];
    int index_count = 1;

    for(int i(0);i<rows;i++)
    {
        for(int j(0);j<cols;j++)
        {
            if(weight_image.at<double>(i,j) < 2.0)//isNAN(cv_dpth.at<float>(i,j)))  // depth value is NAN
            {
                verticies[i*cols + j] = NULL;
            }
            else
            {
                float depth = (float)cv_dpth_blur.at<double>(i,j);//ave_image.at<double>(i,j);//cv_dpth.at<float>(i,j);
//                if(weight_image.at<double>(i,j+1) < 2.0 && weight_image.at<double>(i,j-1) < 2.0)
//                    depth = (depth + (float)ave_image.at<double>(i,j+1) + weight_image.at<double>(i,j-1))/3;
                Vertex* vert = new Vertex;
                verticies[i*cols + j] = vert;
                float s[1][1][2] ={{{j,i}}};
                vector<Point2f> src;
                src.push_back(Point2f(j,i));
                //Mat src(1,1,CV_32FC2,s);
                vector<Point2f> dst;
                // fill src matrix
                undistortPoints(src,dst,cameraMatrix,distCoeffs);
                //cout << i << " " << dst[0].x << " " << j << " " << dst[0].y << endl;
                float x = depth*dst[0].x;//depth*(dst[0].x - cx)/fx;
                float y = depth*dst[0].y;//depth*(dst[0].y - cy)/fy;
                float z = depth;
                vert->pos = Vec3f(x,y,z);


                if(meshfile.is_open() && i%DSR == 0 && j%DSC == 0)
                {
                    meshfile << "v " << x << " " << y << " " << z << "\n";
                    vert->index = index_count;
                    index_count++;
                }
            }
        }
    }

    for(int i(0);i<rows;i+=DSR)
    {
        for(int j(0);j<cols;j+=DSC)
        {
            if(verticies[i*cols + j] != NULL && i+DSR < rows && j+DSC < cols)
            {
                if(verticies[(i + DSR)*cols + j] != NULL && verticies[(i + DSR)*cols + (j + DSC)] != NULL
                        && fabs(verticies[(i + DSR)*cols + (j + DSC)]->pos[2] - verticies[i*cols + j]->pos[2]) < edge_threshold
                        && fabs(verticies[(i + DSR)*cols + j]->pos[2] - verticies[i*cols + j]->pos[2]) < edge_threshold
                        && fabs(verticies[(i + DSR)*cols + (j + DSC)]->pos[2] - verticies[(i + DSR)*cols + j]->pos[2]) < edge_threshold )
                {
                    if(meshfile.is_open())
                    {
                        meshfile << "f " << verticies[i*cols + j]->index << " " << verticies[(i + DSR)*cols + j]->index << " " << verticies[(i + DSR)*cols + (j + DSC)]->index << "\n";
                    }
                }
                if(verticies[(i + DSR)*cols + (j + DSC)] != NULL && verticies[i*cols + (j + DSC)] != NULL
                        && fabs(verticies[(i + DSR)*cols + (j + DSC)]->pos[2] - verticies[i*cols + j]->pos[2]) < edge_threshold
                        && fabs(verticies[i*cols + (j + DSC)]->pos[2] - verticies[i*cols + j]->pos[2]) < edge_threshold
                        && fabs(verticies[(i + DSR)*cols + (j + DSC)]->pos[2] - verticies[i*cols + (j + DSC)]->pos[2]) < edge_threshold)
                {
                    if(meshfile.is_open())
                    {
                        meshfile << "f " << verticies[i*cols + j]->index << " " << verticies[(i + DSR)*cols + (j + DSC)]->index << " " << verticies[i*cols + (j + DSC)]->index << "\n";
                    }
                }
            }
        }
    }
    meshfile.close();




    // convert a depth image to show with opencv
//    Mat show;
//    cv_dpth_blur.convertTo(show,CV_8U,255.0/(max-min));
//    Size k(3,3);
    imshow("Image window", cv_img);
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
