# **Introduction**

This is a general framework for legged robots - ros_gazebo simulation (taking the unitree gol for example)

![test_file](unitree_ros/test_pictures/bipedal_go1.png)





Putting "unitree_ros" under your catkin workspace "src/" dir, then "catkin_make" will work ....

Already tested on Ros Moledic


# **Dependency**

(1) Add catkin dependency, please make sure you already have successfully tested the "unitree_ros" repo from "https://github.com/unitreerobotics/unitree_ros".

(2) Install C++ optimization library "Mosek" (academic license), following a guide from "https://docs.mosek.com/latest/cxxfusion/index.html". 
                                
                                ----- Mosek 9.0.XX or a newer version should work (downloaded from "https://www.mosek.com/downloads/list/9/")

                         Note!!!! replace the mosek library directory in "unitree_ros/mosek_nlp_kmp/CMakeLists.txt" by your own location.  

(3) Install c++ matrix armadillo:

                                sudo apt-get install liblapack-dev
                                
                                sudo apt-get install libblas-dev
                                
                                sudo apt-get install libboost-dev
                                
                                sudo apt-get install libarmadillo-dev
                                
////////////////////////////////////////////////////////////////////

# **Steps**

(1) 'caktin_make'

(2) 'source devel/setup.bash'

(3) load gazebo simulator using GO1 desription: 'roslaunch unitree_gazebo normal.launch rname:=go1 wname:=empty' (Motion planning nodes also start at the same time)

(4) test the locomotion: 'rosrun rosrun go1_rt_control go1_servo'

           ---------change 'gait_mode' to be 101 or 102 (defined in " unitree_ros/go1_rt_control/src/servo_control/servo.cpp"), to see what will happen.
           
 Good luck.

