
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "std_msgs/String.h" 
#include <sstream> 

using namespace cv;
using namespace std;
Mat src_gray;
int thresh = 100;
RNG rng(12345);
void thresh_callback(int, void* );
static const std::string TOPIC_NAME = "/camera/color/image_raw";
static const std::string DEPTH_TOPIC_NAME = "/camera/depth/image_rect_raw";
void thresh_callback(int, void* );


void publish_callback(int, void* ){


}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::imwrite("rgb.bmp", cv_ptr->image);
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        cvtColor( cv_ptr->image, src_gray, COLOR_BGR2GRAY );
        blur( src_gray, src_gray, Size(3,3) );
        const char* source_window = "Source";
        namedWindow( source_window );
        imshow( source_window, cv_ptr->image );
        const int max_thresh = 255;
        createTrackbar( "Canny thresh:", source_window, &thresh, max_thresh, thresh_callback );
        thresh_callback( 0, 0 );
        cv::waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                msg->encoding.c_str());
    }
}

void imageDepthCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Save image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");
        cv::Mat mat = cv_ptr->image;
        cv::Mat beConvertedMat(mat.rows, mat.cols, CV_8UC4, mat.data); // provide different view of m1 data
        cv::imwrite("rgbd.bmp", beConvertedMat);
        // Show image
        cv::imshow("depthview", cv_bridge::toCvShare(msg, "16UC1")->image);
        cv::waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to '16UC1'.",
                msg->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_transport_subscriber");
    //ros::init(argc, argv, "simple_publisher_node");
    ros::NodeHandle nh;
    //cv::namedWindow("view");
    cv::namedWindow("depthview");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(TOPIC_NAME, 1,
            imageCallback);
    image_transport::Subscriber sub_depth = it.subscribe(DEPTH_TOPIC_NAME, 1,
            imageDepthCallback);
    ros::spin();
    cv::destroyWindow("view");
    cv::destroyWindow("depthview");
    ros::shutdown();
    return 0;
}

void thresh_callback(int, void* )
{
    Mat canny_output,thr, gray, src;
    Canny( src_gray, canny_output, thresh, thresh*2 );
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    // find moments of the image
    Moments m = moments(thr,true);
    Point p(m.m10/m.m00, m.m01/m.m00);

    // coordinates of centroid
    cout<< Mat(p)<< endl;
    // get the moments
    
    vector<Moments> mu(contours.size());
    for( int i = 0; i<contours.size(); i++ )
    { mu[i] = moments( contours[i], false ); }

    // get the centroid of figures.
    vector<Point2f> mc(contours.size());
    for( int i = 0; i<contours.size(); i++)
    { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
    //////////////////////////
    /////////////////////////////////
    cout<< Mat(p)<< endl; 
    // fit bounding rectangle around contour
    cv::RotatedRect rotatedRect = cv::minAreaRect(contours[i]);
    // read points and angle
    cv::Point2f rect_points[4]; 
    rotatedRect.points( rect_points );
    float  angle = rotatedRect.angle; // angle
    cout << Mat(p) << ',' << angle << endl;
    }
    Mat drawing(canny_output.size(), CV_8UC3, Scalar(255,255,255));
    for( int i = 0; i<contours.size(); i++ )
    {
    Scalar color = Scalar(167,151,0); // B G R values
    drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
    circle( drawing, mc[i], 4, color, -1, 8, 0 );
    }

    // show the resultant image
    namedWindow( "Contours", WINDOW_AUTOSIZE );
    imwrite( "Contours.jpg", drawing );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
        cv::RotatedRect rotatedRect = cv::minAreaRect(contours[i]);

    // read points and angle
    cv::Point2f rect_points[4]; 
    rotatedRect.points( rect_points );
    float  angle = rotatedRect.angle;
    std::stringstream ss;   ss << angle; // convert float to string
    cv::circle(drawing , mc[i], 5, cv::Scalar(0,255,0)); // draw center
    cv::putText(drawing , ss.str(), mc[i] + cv::Point2f(-25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,0,255));
    }
    imshow( "Contours", drawing );
}
