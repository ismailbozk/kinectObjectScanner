# README #

### What is this repository for? ###

This project is a rigid object scanner via the Microsoft's Kinect.

### How do I get set up? ###

This project is built on Visual Studio 2012.
There are two additonal libraries need to be initialized.

1. OpenCV 2.4.9 must be initialised and $(OPENCV_DIR) must be added under the sytem paths.
2. Point Cloud library 1.7.2 for VS2012. http://unanancyowen.com/?p=1255&lang=en
    1. PCL 1.7.2 can be found under project's Download section. Here is the guide to install it properly.
        1. Download the All-in-one Installer, and install. In the standard, PCL is installed in the “C:\Program Files\PCL 1.7.2″ (or “C:\Program Files (x86)\PCL 1.7.2″). Please use the Uninstall.exe when you want to uninstall.
        2. Set the environment variable for PCL and 3rdParty.
            1. PCL_ROOT	C:\Program Files\PCL 1.7.2(or C:\Program Files (x86)\PCL 1.7.2)
            2. Path	;%PCL_ROOT%\bin  ;%PCL_ROOT%\3rdParty\FLANN\bin  ;%PCL_ROOT%\3rdParty\VTK\bin
            3. If you installed the OpenNI2, as well setting the following path.
            4. Path	;%OPENNI2_REDIST64%(or %OPENNI2_REDIST%)

You can start to test by PlayGound class with the testData under testData directory.

### What does it do? ###

0. Load the data. 
    1. Get two consecutive kinect rgb-depth frames.
    2. Calibrate depth frame according to the rgb frame.
    3. Convert depth frame into 3D space point cloud.
1. Extract the SURF features on consecutive 2D rgb frames.
2. Match the features of the consecutive frames on both 2D and 3D space.
3. Corresponding point base registration:[A Method for Registration of 3D Shapes!](http://www.cs.virginia.edu/~mjh7v/bib/Besl92.pdf)
4. Now apply Iterative Closest Point Registration:[A Method for Registration of 3D Shapes!](http://www.cs.virginia.edu/~mjh7v/bib/Besl92.pdf). To reduce the error of the previous step.
5. Finally Surface Reconstruction. (I have a bug here) 
6. Repeat the all the steps for the n+1 and n+2 rgb-d frames.

[Playground.cpp!](https://bitbucket.org/ismailbozk/kinectscanner/src/04b1c00e1f2e/PlayGround.cpp?at=master) work pipeline.

You can use different rgb-d frames which are located in the testData directory.

### Some Test Results ###

https://youtu.be/aeZjN-4Y1sg

### A Theoretical and Experimental Research in Computer Vision ###

[Depth-Scale Method in 3D Registration of RGB-D Sensor Outputs!](https://drive.google.com/file/d/0B7zdPQ85ffutMURUQ3BvLWM5eE0/edit?usp=sharing)

### About Author ###

[Linkedin!](https://www.linkedin.com/in/ismailbozk)

### Licence ###

KinectScanner is available under [MIT Licence!](https://bitbucket.org/Llwydbleidd/kinectscanner/src/49e24b79d5bdb652f32437a2f3e87647d2f02536/LICENCE.txt)