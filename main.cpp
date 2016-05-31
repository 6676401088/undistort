//
// Created by alex on 5/31/16.
//
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

void undistort(const cv::Mat& input,
               cv::Mat& output,
               double f,
               double cx,
               double cy,
               double k1,
               double k2,
               double p1,
               double p2) {
    //cout << "UNDISTORT: Starting..." << endl;
    int rows = input.rows;
    int cols = input.cols;

    cv::Mat mapx(rows,cols,CV_32FC1);
    cv::Mat mapy(rows,cols,CV_32FC1);
    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            double x = (j - cx) / f;
            double y = (i - cy) / f;
            double r = sqrt(x * x + y * y);
            double theta = atan(r);
            double theta2 = theta * theta;
            double theta4 = theta2 * theta2;
            double theta6 = theta4 * theta2;
            double theta8 = theta4 * theta4;
            double theta_d = theta * (1 + k1 * theta2 + k2 * theta4 + p1 * theta6 + p2 * theta8);
            double scale = (r == 0) ? 1.0 : theta_d / r;
            double u = f * x * scale + cx;
            double v = f * y * scale + cy;

            mapx.at<float>(i,j) = (float) u;
            mapy.at<float>(i,j) = (float) v;
        }
    }

    std::vector<cv::Mat> bgrChannels(3);
    cv::split(input,bgrChannels);
    cv::Mat b_img(rows,cols,CV_8UC1);
    cv::Mat g_img(rows,cols,CV_8UC1);
    cv::Mat r_img(rows,cols,CV_8UC1);
    b_img = bgrChannels[0];
    g_img = bgrChannels[1];
    r_img = bgrChannels[2];

    cv::Mat bb_img(rows,cols,CV_8UC1);
    cv::Mat gg_img(rows,cols,CV_8UC1);
    cv::Mat rr_img(rows,cols,CV_8UC1);
    cv::Scalar zeroScalar = cv::Scalar(0);
    cv::remap(b_img,bb_img,mapx,mapy,0,0);
    cv::remap(g_img,gg_img,mapx,mapy,0,0);
    cv::remap(r_img,rr_img,mapx,mapy,0,0);

    //merge undistorted grayscale images back into color image. Replaces mixchannels() with merge() because mixchannels() throws nasty error
    std::vector<cv::Mat> arrays;
    arrays.push_back(bb_img);
    arrays.push_back(gg_img);
    arrays.push_back(rr_img);
    cv::merge(arrays,output);
}

int main() {
    std::cout << "Running Alex's undistort program. Parameters are hard-coded please forgive me." << std::endl;
    /* Camera parameters are defined according to the OpenCV conventions: http://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
     * Parameters for a given camera can be computed using a chessboard optimization procedure e.g. http://wiki.ros.org/ethzasl_ptam/Tutorials/camera_calibration
     * Parameters below are from one of Alex's previous projects.
     */

    //File name
    std::string videoPath = "/home/alex/mono/data/Beck-roof.mp4";
    int startFrameRange = 50; //which frame you want to examine
    int endFrameRange = 60;

    //Calibration parameters
    double fx = 1223.80333938171;
    double fy = 1223.80333938171;
    double cx = 959.5;
    double cy = 539.5;

    //Distortion parameters
    double k1 = -0.0568935551550414;
    double k2 = 0.00111003912518243;
    double p1 = 0.000561203697652756;
    double p2 = -0.00201554554913136;

    //Handle video input
    cv::VideoCapture capture(videoPath);
    if (!capture.isOpened()) {
        cerr << "The video file could not be opened successfully!!!" << endl;
        return 0;
    }
    int f = 0;
    bool readSuccess = true;
    while(readSuccess == true){
        cv::Mat videoFrame;
        readSuccess = capture.read(videoFrame); //extract video frame
        if (readSuccess == true && f >= startFrameRange && f <= endFrameRange) {
            cv::Mat undistortedFrame(videoFrame.rows, videoFrame.cols, CV_8UC3);
            undistort(videoFrame,undistortedFrame,fx,cx,cy,k1,k2,p1,p2); //perform undistortion
            cv::Mat concatenated;
            cv::hconcat(videoFrame,undistortedFrame,concatenated);
            cv::namedWindow("INPUT (LEFT) vs OUTPUT(RIGHT)",cv::WINDOW_NORMAL);
            cv::resizeWindow("INPUT (LEFT) vs OUTPUT(RIGHT)",2560,1440);
            cv::imshow("INPUT (LEFT) vs OUTPUT(RIGHT)",concatenated);
            cv::waitKey(0);
        }
        f++;
    }
    return 0;
}
























