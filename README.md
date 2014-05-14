C-Slam
======

[C-Slam](http://airlab.ws.dei.polimi.it/index.php/C-SLAM), Is a Cognitive Self Localization And Mapping fremework, that works using knowledge about the world to recognise objects,, create a map and localize the robot.
The aim of this project is to work get a realiable SLAM system in noisy and mutable envirorment to perform navigation and path plannning tasks with low computational power, not to get a high precision SLAM system.


USAGE
-----

You need to install the [ROS middleware](http://www.ros.org) to compile and execute the code in this repository. Furthermore, some part of the code need the OpenCV libraries to be installed on your system (this should be automatically done by ROS), `flex` and `bison`.


COMPILING
---------

The system can be build using the ros build tool `catkin`. Just create a catkin workspace, put the content of this repository in the src repository and run `catkin_make` to build the system.
check [this](http://ros.org/wiki/catkin/Tutorials/create_a_workspace) tutorial to get more info on catkin.

RUNNING THE REASONER
--------------------

The reasoner can be used either to evaluate fuzzy rules from a knowledge base or for classify objects by using a fuzzy classifier tree, for which you must specify both a knowledge base for the fuzzy rules and the file of the classifier.

To run the reasoner node:

rosrun c_fuzzy c_fuzzy_reasoner < KNOWLEDGEBASE_PATH > < KNOWLEDGEBASE_CLASSIFIER_PATH > < CLASSIFIER_PATH >

the reasoner will activate two services:
- /classification
- /reasoning


RUNNING WITH BAGS
-----------------

There is a launch file that can be used to launch the system, opening rqt_bag to testthe system with AR Drone bags. It also launch the image_proc node to rectify the AR drone input image.
You can use the launch file with the following command:

`roslaunch c_vision c_vision.launch`



Copyright & contacts
--------------------

This project is distributed under the GNU GPL license, version 3.

(C) 2012 Davide Tateo

(C) 2012 Politecnico Di Milano

For further information, please contact davide.tateo90@gmail.com
