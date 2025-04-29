A simple ROS2 Humble node to detect features in an image.

Requires OpenCV. Tested on x86, Ubuntu 22.04, OpenCV 4.5.4.

Subscription:
* /image_raw

Publication:
* /image_annotated

Typical usage:
~~~
ros2 run opencv_features detect_features --ros-args -r /image_raw:=/left/image_raw -p detector_type:=ORB
~~~

Features supported:
* SIFT
* BRISK
* ORB
* AKAZE
* MSER
* FAST (FastFeatureDetector)
* blob (SimpleBlobDetector)
* Agast (AgastFeatureDetector)
* GFTT (GFTTDetector)
