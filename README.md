# Dynamic_Object_Tracking_Kalman_Filter
A Matlab implementation of a 2-D dynamic object tracking algorithm using Kalman filters

This project presents the formulation and implementation of a Kalman filter based dynamic object tracking algorithm. Object
tracking is one of the most fundamental problems in the area of visual odometry, that deals with predicting and tracking the
position, velocity, and attitude of a moving body. We have proposed a solution to this problem based on the principle of
adaptive state estimation using Kalman filters. Specifically, we have analyzed the motion of an object across a static
background, and used predictive filter outputs to track it's centroid coordinates across video frames. The Kalman filter was
constructed using a physical model based on Newton's laws of motion, and was augmented with the relevant noise covariance
matrices estimated from each image. The algorithm was found to yield sub-pixel tracking accuracy in the horizontal dimension
of the object's motion. The vertical tracking accuracy was found to be on the order of ca. 5 pixels, owing to greater
uncertainty (noise) in the object's vertical motion. Although the algorithm performs well for motion constrained to two 
dimensions, we predict that for more complex trajectories, similar accuracies can be obtained by use of additional data from
on-board inertial sensors or multiple camera angles.

Implemented using Matlab R2017b, and the Image Processing Toolbox.

Completed towards partial course requirement for CSCI 5454 -- Design and Analysis of Algorithms (Fall 2017) at the University 
of Colorado Boulder.
