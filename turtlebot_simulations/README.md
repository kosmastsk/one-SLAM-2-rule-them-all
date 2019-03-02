# Simulations with turtlebot

The folder /worlds contains .world files that can be used directly with Gazebo and contains many different environment categories.  
To use these files, run:  

`roscore` in terminal #1  

`rosrun gazebo_ros gazebo filename.world` in terminal #2

If a world is not loading, it is possible that the models it contains are not installed.  
In that case, download the /models directory and copy the contents.  
`sudo cp models/* ~/.gazebo/models/`

The output of different SLAM algorithms in found in the /maps folder.

An evaluation between the different SLAM algorithms in these environments can be found in the /evaluations folder. For the evaluation the [ogm_evaluation](https://github.com/robotics-4-all/ogm_merging) package was used.
