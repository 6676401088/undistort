## Straight lines should be straight image undistortion procedure.
### Instructions
1. Download the sample drone video dataset from https://www.dropbox.com/s/ys0akvwmcgr3iiv/alex_drone_video.mp4?dl=0
2. Open main.cpp and set the algorithm parameters according to your requirements. Most important is the video filepath which is set to the path on my computer.
3. Forgive me for having hard coded parameters.
4. Build and run according to the standard CMake procedure: cd undistort, mkdir build, cd build, cmake .., make, ./undistort

###Example Result (Input Frame (left) vs. Output Frame (right))
![Undistortion Result: Input(left) vs Output(right)](https://github.com/alexhagiopol/undistort/blob/master/example_result.png)
