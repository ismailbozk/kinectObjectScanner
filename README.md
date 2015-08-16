# README #

### What is this repository for? ###

This is a rigid object scanner project.

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

### How does it work? ###

0. Load the data. 
    1. Get two consecutive kinect rgb-depth frames.
    2. Calibrate depth frame according to the rgb frame.
    3. Convert depth frame into 3D space point cloud.
1. Extract the SURF features on consecutive 2D rgb frames.
2. Match the features of the consecutive frames on both 2D and 3D space.
3. Corresponding point base registration:[A Method for Registration of 3D Shapes!](http://www.cs.virginia.edu/~mjh7v/bib/Besl92.pdf)
4. Now apply Iterative Closest Point Registration:[A Method for Registration of 3D Shapes!](http://www.cs.virginia.edu/~mjh7v/bib/Besl92.pdf). To reduce the error of the previous step.
5. Repeat the all the steps for the n+1 and n+2 rgb-d frames and so on.
6. Finally Surface Reconstruction. (I have a bug here) 
[Playground.cpp!](https://github.com/ismailbozk/kinectObjectScanner/blob/master/PlayGround.cpp) work pipeline.

You can use different rgb-d frames which are located under the testData directory.

### Some Test Results ###

[![Depth-Scale Method in 3D Registration of RGB-D Sensor Outputs - VISAPP 2014 ][1]][2]

 [1]: http://img.youtube.com/vi/aeZjN-4Y1sg/0.jpg "Depth-Scale Method in 3D Registration of RGB-D Sensor Outputs - VISAPP 2014 "
 [2]: https://www.youtube.com/watch?v=aeZjN-4Y1sg

<iframe width="560" height="315" src="https://www.youtube.com/embed/aeZjN-4Y1sg" frameborder="0" allowfullscreen></iframe>

### A Theoretical and Experimental Research in Computer Vision ###

[Depth-Scale Method in 3D Registration of RGB-D Sensor Outputs!](https://drive.google.com/file/d/0B7zdPQ85ffutMURUQ3BvLWM5eE0/edit?usp=sharing)

### About Author ###

[Linkedin!](https://www.linkedin.com/in/ismailbozk)

### Licence ###

KinectScanner is available under [MIT Licence!](https://github.com/ismailbozk/kinectObjectScanner/blob/master/LICENCE.txt)
