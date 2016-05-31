//
// Created by alex on 5/31/16.
//
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

void undistort(const cv::Mat& input, cv::Mat& output, double f, double cx, double cy, const std::vector<double>& distortion) {
    //cout << "UNDISTORT: Starting..." << endl;
    int rows = input.rows;
    int cols = input.cols;
    //double mapxArr[rows, cols];
    //double mapyArr[rows, cols];
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
            double theta_d = theta * (1 + distortion[0] * theta2 + distortion[1] * theta4 + distortion[2] * theta6 + distortion[3] * theta8);
            double scale = (r == 0) ? 1.0 : theta_d / r;
            double u = f * x * scale + cx;
            double v = f * y * scale + cy;

            mapx.at<float>(i,j) = (float) u;
            mapy.at<float>(i,j) = (float) v;
            //mapxArr[i, j, 0] = (float)u;
            //mapyArr[i, j, 0] = (float)v;
        }
    }
    //cout << "UNDISTORT: Maps made." << endl;

    cv::Mat b_img(rows,cols,CV_8UC1);
    cv::Mat g_img(rows,cols,CV_8UC1);
    cv::Mat r_img(rows,cols,CV_8UC1);
    const int input_to_b[] = {0,0};
    const int input_to_g[] = {1,0};
    const int input_to_r[] = {2,0};
    size_t nPairs = 1;
    size_t nSources = 3;
    size_t nDestinations = 1;

    //Line Take blue channel from input image and add it to single channel b_img
    cv::mixChannels(&input, nSources, &b_img, nDestinations, input_to_b, nPairs);
    cv::mixChannels(&input, nSources, &g_img, nDestinations, input_to_g, nPairs);
    cv::mixChannels(&input, nSources, &r_img, nDestinations, input_to_r, nPairs);

    //cout << "UNDISTORT: Single channel images made." << endl;

    cv::Mat bb_img(rows,cols,CV_8UC1);
    cv::Mat gg_img(rows,cols,CV_8UC1);
    cv::Mat rr_img(rows,cols,CV_8UC1);
    cv::Scalar zeroScalar = cv::Scalar(0);
    cv::remap(b_img,bb_img,mapx,mapy,0,0);
    cv::remap(g_img,gg_img,mapx,mapy,0,0);
    cv::remap(r_img,rr_img,mapx,mapy,0,0);

    //cout << "UNDISTORT: Maps applied to single channel images." << endl;

    nSources = 1;
    nDestinations = 3;
    const int bb_to_output[] = {0,0};
    const int gg_to_output[] = {0,1};
    const int rr_to_output[] = {0,2};

    //merge undistorted grayscale images back into color image. Replaces mixchannels() with merge() because mixchannels() throws nasty error
    std::vector<cv::Mat> arrays;
    arrays.push_back(bb_img);
    arrays.push_back(gg_img);
    arrays.push_back(rr_img);
    cv::merge(arrays,output);
}

int main(){
    std::cout << "Running Alex's undistort program. Parameters are hard-coded please forgive me." << std::endl;
    //Fisheye drone camera parameters. Use a chessboard procedure e.g.
    //Calibration parameters
    double fx = 1223.80333938171;
    double fy = 1223.80333938171;
    double cx = 959.5;
    double cy = 539.5;

    //Distortion parameters
    double k1 = -0.0568935551550414;
    double k2 = 0.00111003912518243;
    double p1 =  0.000561203697652756;
    double p2 = -0.00201554554913136;
    return 0;
}
