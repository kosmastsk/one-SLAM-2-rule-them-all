# One SLAM to rule them all _ common repo

The folder /worlds contains .world files that can be used directly with Gazebo and contains many different environment categories.  
To use these files, run:  

`roscore` in terminal #1  

`rosrun gazebo_ros gazebo filename.world` in terminal #2

If a world is not loading, it is possible that the models it contains are not installed.  
In that case, download the /models directory and copy the contents.  
`sudo cp models/* ~/.gazebo/models/`
