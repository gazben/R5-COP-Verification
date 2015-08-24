# R5-COP-Verification
This project is a realtime verification software for the R5-COP project. Project website: http://www.r5-cop.eu/en/
The package contains a demo of a monitor code generator and the generated monitor frame which subscribes to the velocity commands of a turtlesim turtle.

Currently supported IDE-s:

  * Visual Studio 2015 - Generator (compile/run)
  * CLion - Generator(compile/run), Monitor(compile/run)

Required Software for the Generator:

  * C++11 compatible C++ compiler
  * Boost 1.58 library
  * CMake
  
Download boost 1.58 from here:
http://www.boost.org/users/history/version_1_58_0.html

Getting all the stuff for the Generator:

  ```
  sudo apt-get update
  sudo apt-get upgrade
  sudo apt-get install build-essential
  sudo add-apt-repository ppa:ubuntu-toolchain-r/test
  sudo apt-get update
  sudo apt-get install gcc-5 g++-5
  sudo rm /usr/bin/gcc
  sudo rm /usr/bin/g++
  sudo ln -s /usr/bin/gcc-5 /usr/bin/gcc
  sudo ln -s /usr/bin/g++-5 /usr/bin/g++
  cd ~/Downloads/
  tar --bzip2 -xf boost_1_59_0.tar.bz2
  cd boost_1_58_0/
  ./bootstrap.sh
  sudo ./b2
  sudo ./b2 install
   
  ```

Monitor:
  * Ubuntu 12.04
  * ROS Groovy
  * turtlesim
  * C++11 compatible compiler
The monitor will only run on Ubuntu 12.04!

Getting all the stuff for the monitor:

  ```
  
  sudo apt-get update
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
  wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
  sudo apt-get update
  sudo apt-get install ros-groovy-desktop-full
  sudo rosdep init
  rosdep update
  echo "source /opt/ros/groovy/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  sudo apt-get install python-rosinstall
  source /opt/ros/groovy/setup.bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  catkin_init_workspace
  cd ~/catkin_ws/
  catkin_make
  source devel/setup.bash
  
  ```

Optional:
You should put a sylink into the catkin_ws/src tha will lead to the generated folder.
For example:
  * Symlink: /home/ros/catkin_ws/src/turtle_monitor
  * Folder: /home/ros/R5-COP-Verification

How to run the demo:
  * Copy the package in your source folder ( ~/ros/src$ )
  * Open a terminal window, change your directiory to your workspace directory
  * Start roscore:  
  
  ```
 roscore 
  ```

  * In a separate terminal window type the following command: 
  
    ```
    source devel/setup.bash
    ```
  * Build the executables in the root folder of your catkin workspace using the command : 
  
    ```
    catkin_make
    ```  
    or

    ```  
    catkin_make install --only-pkg-with-deps turtle_monitor
    ```

  * By running the following command the launchfile will set up the demo:

    ``` 
    roslaunch turtle_monitor turtle_monitor.launch
    ```

If any of the expressions above are new to you, search them on http://wiki.ros.org/
