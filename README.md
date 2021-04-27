
![Rociante](https://github.com/deempalme/rocinante/wiki/images/logo.png)

*(spanish: [\[roθiˈnante\]](https://translate.google.com/#es/en/ROS%20Rosy%3F%20Roci!%3F%20Rocin...%20Rocinante!)) ROS message reader library designed for Torero but compatible with your project.*
___

**Rocinante** is a library created to gather **ROS messages** and adapt the received data into information readable for [**Torero** visualization](https://github.com/deempalme/torero). It contains several specialized sections with their own data structure's output that can be easily used in your project, even if you are **not using** the **Torero library**. Please see the `data types` in each [wiki](https://github.com/deempalme/rocinante/wiki) section's description page.

**NOTE:** this library can only be compiled using **catkin**. You could utilize the *boring* traditional way (console) or the *amazing* **QtCreator** to compile with *catkin*. For more information about compiling with **QtCreator** see this [link](https://github.com/deempalme/qt_ros_debugging).

There is a [wiki](https://github.com/deempalme/rocinante/wiki)!

You should read the [installation guide](https://github.com/deempalme/rocinante/wiki/Installation-guide) to setup all the necessary libraries and get **Rocinante** ready to run. There is also a small guide in how to [get started](https://github.com/deempalme/rocinante/wiki/getting-started).

There are guides for each section of the API:

   * [Camera reader](https://github.com/deempalme/rocinante/wiki/ROS-camera-reader) – Reads image data (camera) from `sensor_msgs::Image` message.
   * [Free space reader](https://github.com/deempalme/rocinante/wiki/ROS-free-space-reader) – Obtains the **occupancy grid** matrix data from ROS messages.
   * [Lane reader](https://github.com/deempalme/rocinante/wiki/ROS-lane-reader) – Obtains the **lanes**, **lines**, **trajectories**, etc. from ROS messages.
   * [Laser reader](https://github.com/deempalme/rocinante/wiki/ROS-laser-reader) – Reads the point cloud data from **LaserScans** or **PointCloud2**.
   * [Object reader](https://github.com/deempalme/rocinante/wiki/ROS-object-reader) – Obtains the **dynamic/static objects** from ROS messages.
   * [Street reader](https://github.com/deempalme/rocinante/wiki/ROS-street-reader) – Reads the street information sent through ROS messages.
   * [Vehicle information reader](https://github.com/deempalme/rocinante/wiki/ROS-vehicle-reader) – Obtains vehicle information, suchlike; Navigation, IMU, Odometry, and some extra parameters.

