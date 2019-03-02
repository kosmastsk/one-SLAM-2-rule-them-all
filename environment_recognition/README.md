# environment_recognition


## Description

Take input from LIDAR and RGBD Camera, and use it to recognize the type of environment we are in. Decide the environment's type from some specific factors, such as features/no features, density, brightness/darkness, doors, indoors/outdoors, corridors.


## Description of each procedure

### 1. recognition_lidar
In general, node "recognition_lidar" subscribes to the /scan topic, and publishes the environmental type at the /environment topic. Also during the whole process, it prints the values of some useful variables, so that we can check at runtime whether the result is the expected one. In detail:

###### laserCallBack()
- Compare every single value of the *ranges*[] list with its previous one, starting from *ranges*[1].
  - If the current element is out of the scope of +-0.35*(previous element), raise the variable that counts the variances of each scan by 1 (*temp_var_*). This means that the continuity has stopped. Thus, the robot probably sees the start/end of a feature and apparently not a wall. (The "do nothing" command is executed, so that it doesn't count it as a feature, if both elements are infinite.)
  - For every element, check if it has infinite value. If yes, raise infinite counter (*counter_inf_*) by 1.
- Give a value to the variable that holds the sum of all the features' sizes the robot sees at the current scan (*temp_features_*). Also, call the function to calculate the average values of the last 100 scans (*averageValues*()).

###### averageValues()
- For the first 100 scans, raise the *samples_* by 1. Then, keep it to 100, so that we always calculate the average value of the last 100 scans. Also, reset the index of the lists (*index_*), everytime it reaches 100.
- At each scan, throw away the value we added 101 scans before, and add the current one. In that way, we always get the average value from the last 100 scans, without having it reseted.

###### caseCheck() 
- Check the environmental type, according to the information received from the whole procedure. We end up with the following variables, which are going to be our metrics for the environmental type decision:

  | Variable | Content |    
  | --- | --- |
  | average_var_ | At each measurement, the robot sees a specific number of variances (big distance between 2 laser ranges). This variable holds the mean value of the variances detected at the last 100 scans. |
  | in_out_ | At each measurement, the robot sees a specific number of non-infinite values (and therefore it sees features). This variable holds the percentage of the mean value of non-infinite values detected at the last 100 scans (out of all the ranges in each scan, which is the value of *ranges_size_*). |
  | threshold_variance_ | A threshold we set, in order for the system to decide if it detects features. |
  | threshold_in_out_ | A threshold we set, in order for the system to decide if it is indoors or outdoors. |
  | threshold_complexity_ | A threshold we set, in order for the system to decide if the envitonment is simple or complex. |      

###### Decision tree
- **in_out_ <= threshold_in_out_**: The robot is outdoors, therefore there are no walls.

  - **average_var_ <= threshold_variance_**: The robot does not see features.

  - **average_var_ > threshold_variance_**: The robot sees features.

- **in_out_ > threshold_in_out_**: The robot is indoors, therefore there are walls.

  - **average_var_ <= threshold_variance_**: The robot does not see features.

  - **average_var_ > threshold_variance_**: The robot sees features.

- **average_var_ <= threshold_complexity_**: The robot sees a small amount of edges, therefore the environment is simple.

- **average_var_ > threshold_complexity_**: The robot sees a large amount of edges, therefore the environment is complex.

### 2. recognition_rgb
In general, node "recognition_rgb" subscribes to the /camera/rgb/image_raw topic, and publishes the environmental type at the /environment topic. Also during the whole procedure, it prints the values of some useful variables, so that we can check on the same time if the result is the expected one. In detail:

###### rgbCallback()
- Make a copy of the RGB image and convert it to Grayscale, so that we can detect brightness.
- Calculate the average of all the Grayscale values of each measurement

###### averageValues()
- For the first 100 scans, raise the *samples_* by 1. Then, keep it to 100, so that we always calculate the average value of the last 100 scans. Also, reset the index of the lists (*index_*), everytime it reaches 100.
- At each scan, throw away the value we added 101 scans before, and add the current one. In that way, we always get the average value from the last 100 scans, without having it reseted.

###### caseCheck() 
- Check the environmental type, according to the information received from the whole procedure. We end up with the following variables, which are going to be our metrics for the environmental type decision:

  | Variable | Content |    
  | --- | --- |
  | average_gray_ | At each measurement, the camera sees some Grayscale data after the convertion we make (RGB -> Grayscale). This variable holds the mean value of the average Grayscale values at the last 100 scans. |
  | threshold_brightness_ | A threshold we set, in order for the system to decide if it detects a bright environment. |
  | threshold_darkness_  | A threshold we set, in order for the system to decide if it detects a dark environment. |      

###### Decision tree
1. **average_gray_ < threshold_darkness_**

   The environment is dark.

2. **average_gray_ > threshold_brightness_**

   The environment is bright.

3. **threshold_darkness_ < average_gray_ < threshold_brightness_**

   The environment is normal (light wise).

## How to run it

1. Launch a turtlebot_gazebo world
2. Launch turtlebot_teleop or a navigation algorithm
3. Echo the topic /environment
4. Launch recognition_lidar.launch and recognition_rgb.launch (change the parameters' values if needed)
