# Laboratory of Applied Robotics Student Interface
Package used by student to complete the assignment of the course. 

### Students:
* Davide Bertelli
* Davide Dalla Stella

## Scenario
A pursuer robot and a fugitive robot are placed inside an arena characterized by the presence of obstacles and exit gates. The pursuer's goal is to catch the fugitive before it escapes from a possible exit. This scenario is very suitable for studying the applications of motion planning and automated planning in real problems. 

All the information about the environment are processed to code a pddl problem that the Metric-FF planner uses to get the path that a robot must follow. Subsequently this path is processed to obtain a dubins path to be followed from point to point.

## Project objectives

* development and implementation of sensing algorithms to recognise objects and obstacles in the competition field
* development and implementation of motion planning algorithms to move the robot from an initial to a final position in minimum time avoiding obstacles
* development and implementation of intelligent planning algorithms to decide the optimal game strategy
* testing of the solution in simulation and on the field

## Installation

OS Required: Ubuntu 16.04

### Ros Installation

Setup:

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-desktop-full
```

Environment setup:

```
$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

Dependencies for building packages:

```
$ sudo apt install python-rosdep
$ sudo apt install ros-kinetic-rqt-multiplot ros-kinetic-usb-cam
$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
$ sudo apt-get install python-catkin-tools
$ sudo apt install chrpath
$ sudo apt-get install libignition-math2-dev
$ sudo apt install terminator # package to manage multiple terminal session
$ sudo apt install ros-kinetic-jsk-visualization # tool to visualize the detected polygon in RVIZ
```

Initialize rosdep:

```
$ sudo rosdep init
$ rosdep update
```

### Gazebo Installation

Setup:

```
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Install Gazebo:

```
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Check installation:

```
$ gazebo
```

Solve known issue with VMware

```
$ echo "export SVGA_VGPU10=0" >> ~/.profile
```

### OpenCV Installation

Setup

```
$ sudo apt-get update
```

Install dependencies

```
$ sudo apt-get install cmake git libgtk2.0-dev pkg-config # these are mandatory
$ sudo apt-get install -y qt5-default libvtk6-dev # gui
$ sudo apt-get install -y zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libjasper-dev libopenexr-dev libgdal-dev # media I/O
$ sudo apt-get install -y libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev # video I/O
```

Install the library

```
$ wget https://github.com/opencv/opencv/archive/3.3.1.zip
$ unzip 3.3.1.zip
$ rm 3.3.1.zip
$ mv opencv-3.3.1 OpenCV
$ cd OpenCV
$ mkdir build
$ cd build
$ cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DBUILD_EXAMPLES=ON ..
$ make -j4
```

Install OpenCV

```
$ sudo make install
$ sudo ldconfig
```

Check installation

```
$ pkg-config --cflags opencv # get the include path (-I)
$ pkg-config --libs opencv   # get the libraries path (-L) and the libraries (-l)
```

### Setup Simulator

#### Clone the simulator

Clone git:

```
$ mkdir ~/workspace
$ cd workspace
$ git clone https://github.com/AlexRookie/AppliedRoboticsEnvironment.git simulator/
```

Compile the simulator:

```
$ cd simulator
$ catkin build
$ source ./environment.sh # aliases definition
```

#### Clone the project

Clone forked git:

```
$ cd ~/workspace
$ git clone https://github.com/fromstar/AppliedRoboticsStudentInterface.git project
$ cd project
$ mkdir build # call this folder build or the system will not work
$ cd build
```

Compile:

```
$ cmake .. # be sure that before you: source ~/workspace/simulator/environment.sh
$ make     # compile the .cpp file contained in ~/project/src. The cmake is including the header in ~/
$ source ../environment.sh
```


#### Automatic source of the environment

Do this only if the above procedure worked without errors.

```
echo "source ${AR_CATKIN_ROOT}/environment.sh" >> ~/.bashrc
echo "source ${AR_ROOT}/environment.sh > /dev/null 2>&1" >> ~/.bashrc
```

### Planner Installation

The offical download of this version of Metric_FF is: https://github.com/tatsubori/Metric-FF.

It is reccomended to use the planner already included in the project folder of this work. It has been slightly modified to make the output compatible with other official versions of Metric.

The planner is already compiled but if necessary recompile it then install the following packages before do that:
'''
sudo apt-get update 
sudo apt-get install flex bison
'''

## Run the project
Open terminator and split it in 3 terminals. 
In the terminals use these commands:\
```
AR_simulator_gui or AR_simulator
AR_pipeline
AR_run
```
If it is necessary to specify the number of robots present, it must be done both at the start of the simulator and the pipeline with the option "n:=NumberOfRobots".

Ex:
```
AR_simulator_gui n:=2
AR_pipeline n:=2
```
