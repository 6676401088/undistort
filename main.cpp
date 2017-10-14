/* *********************************************************************************************************************
 * Straight Lines Should Be Straight Undistortion Procedure
 * **********************************************************************************************************************
 * Set your parameters below. The program will show a side by side comparison of each input video frame
 * and its undistorted result. Press any key to advance to the next frame.Camera parameters are defined according to the
 * OpenCV conventions: http://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
 * Parameters for a given camera can be computed using a chessboard optimization procedure e.g. http://wiki.ros.org/ethzasl_ptam/Tutorials/camera_calibration
 * Parameters below are from one of Alex's previous projects.
 */

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
using namespace std;

/* *********************************************************************************************************************
 * SET YOUR ALGORITHM PARAMETERS HERE:
 * ********************************************************************************************************************/
const string videoPath = "/home/alex/undistort/alex_drone_video.mp4"; //Absolute path to video file
const int startFrameRange = 500; //Examine by hand starting from this frame number
const int endFrameRange = 550; //Examine by hand ending at this frame number
const double fx = 1223.80333938171; //Camera calibration parameter
const double fy = 1223.80333938171; //Camera calibration parameter
const double cx = 959.5; //Camera calibration parameter
const double cy = 539.5; //Camera calibration parameter
const double k1 = -0.0568935551550414; //Camera distortion parameter
const double k2 = 0.00111003912518243; //Camera distortion parameter
const double p1 = 0.000561203697652756; //Camera distortion parameter
const double p2 = -0.00201554554913136; //Camera distortion parameter
/* *********************************************************************************************************************
 * END PARAMETERS SETTINGS
 * ********************************************************************************************************************/

void undistort(const cv::Mat& input,
               cv::Mat& output,
               double f,
               double cx,
               double cy,
               double k1,
               double k2,
               double p1,
               double p2) {
    int rows = input.rows;
    int cols = input.cols;

    //Produce X and Y maps. We use these to know how to map the pixels in the input to pixels in the output.
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

    //Split input image into B G and R channels
    std::vector<cv::Mat> bgrChannels(3);
    cv::split(input,bgrChannels);
    cv::Mat bb_img(rows,cols,CV_8UC1);
    cv::Mat gg_img(rows,cols,CV_8UC1);
    cv::Mat rr_img(rows,cols,CV_8UC1);
    cv::remap(bgrChannels[0],bb_img,mapx,mapy,0,0);
    cv::remap(bgrChannels[1],gg_img,mapx,mapy,0,0);
    cv::remap(bgrChannels[2],rr_img,mapx,mapy,0,0);

    //merge undistorted grayscale images back into color image. Replaces mixchannels() with merge() because mixchannels() throws nasty error
    std::vector<cv::Mat> arrays;
    arrays.push_back(bb_img);
    arrays.push_back(gg_img);
    arrays.push_back(rr_img);
    cv::merge(arrays,output);
}

int main() {
    std::cout << "Running undistort program." << std::endl;
    //Handle video input
    cv::VideoCapture capture(videoPath);
    if (!capture.isOpened()) {
        cerr << "The video file could not be opened." << endl;
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

