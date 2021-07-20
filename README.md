# InHuS Social Navigation

## Installation (ros-melodic)

### Dependencies

Ros Melodic has to be installed (http://wiki.ros.org/melodic/Installation/Ubuntu).

Here is a list of all the dependencies to get with `sudo apt install [package]` :
```
sudo apt install xterm libsdl-image1.2-dev libsuitesparse-dev ros-melodic-move-base-msgs ros-melodic-pr2-msgs ros-melodic-tf2-sensor-msgs ros-melodic-mbf-msgs ros-melodic-mbf-costmap-core ros-melodic-joint-trajectory-action ros-melodic-costmap-converter ros-melodic-libg2o ros-melodic-pointcloud-to-laserscan ros-melodic-teb-local-planner
```

### Clone and compile InHuS

Once all dependencies installed, clone the workspace repository wherever you want and then compile it using catkin_make :
```
git clone https://github.com/AnthonyFavier/InHuS_Social_Navigation
cd InHuS_Social_Navigation/
catkin_make
```

### MORSE Simulator installation

The Stage simulator is in-build in the package and can be used without any additional actions. However, to be able to start the system with the MORSE simulator you will need a separated installation and thus to follow the next instructions. Despite the fact MORSE is heavier, it gives more precise movements and we recommand to use it for experimenting. But you can skip this step if you only want to use the way lighter Stage simulator for a quick test.

You first need to clone the following repository :
```
git clone https://github.com/sphanit/morse
```
Then go in the downloaded directory to build and install the simulator :
```
cd morse
mkdir build && cd build
cmake ..
sudo make install
```
(Make sure *python-dev* and *python3-dev* are installed)

After that, you can check if the installation and configuration is ok :
```
morse check
```

## Usage

**0. Sourcing**

Don't forget in each shell to source the workspace :
```
source devel/setup.bach
```

**1. Start the simulator**
* With Stage :
```
roslaunch inhus_navigation stage_simulator.launch 
```
* With MORSE (if installed thanks to the instructions above) :
```
roslaunch inhus_navigation morse_simulator.launch 
```
**2. Start InHuS**
* With basic navigation : 
```
roslaunch inhus_navigation inhus_nav.launch
```
* With enhanced navigation (CoHAN)
```
roslaunch inhus_navigation inhus_wo_nav.launch
```
```
roslaunch inhus_navigation cohan_human_agent.launch 
```
**3. Send goals**

Now use the Boss interface to send goals individually to different agents, start scenarios, activate attitudes or endless mode. Predifined goals are specified in the *goals.xml* file.

![boss](https://github.com/AnthonyFavier/images/blob/master/boss.png)

**4. View logs**
Once the system is shut down. Go to *InHuS_Social_Navigation/src/inhus_system/inhus/logs/* and run the following python script to show both the time colored path of agents and graphs of the metrics :
```
python all.py
```
Note that you can add 2 arguments to show only a specified temporal window :
```
python all.py 175 180
```
![graph](https://github.com/AnthonyFavier/images/blob/master/graphs_OO_smb_replan.png)
![path](https://github.com/AnthonyFavier/images/blob/master/paths_OO_smb_replan_new.png)
![legend](https://github.com/AnthonyFavier/images/blob/master/legend.png)

More details about show the log data are avaible in the file *readme.txt* in the same folder.

## InHuS : A generic architecture

InHuS is an architecture thought to be as generic and modular as possible. It's purpose is to control a simulated autonomous human agent which is both reactive and rational. Given a goal, the agent plans and executes a set of actions in order to fulfill its goal. A user interface is provided to easily send goals to both other agents and the human one present in the scene. The interface makes it easy to repeat scenarios and run long term activity. A log manager is also part of the system. It records and computes execution data.
![global archi](https://github.com/AnthonyFavier/images/blob/master/global_archi_grand.png)

## Tailored to navigation

The provided implemented version is tailored to social robot navigation. Thus, plans are composed of positions to reach. Thanks to some unique features the system offers several rational human behaviors. 

![corridor](https://github.com/AnthonyFavier/images/blob/master/nav_hateb.gif)

**1. Navigation Conflict Manager**

One of the unique features is the ability to detect and act over path blockage by other agents. Instead of taking a detour, the agent performs an approach and waits for the path to be cleared. The behavior to follow when a navigation conflict is encountered is described in the Navigation Conflict Manager component, itself in the Human Behavior Model component.

![path blockage](https://github.com/AnthonyFavier/images/blob/master/path_blocked.gif)

**2. Attitudes**

Another unique feature is the ability to activate Attitudes. Attitudes are modes that affect goal decisions and reactions regarding other agents. They can create several very different behaviors and long term activity. Four attitudes are currently implemented, two of them are represented below (Harass and StopAndLook) : 

![harass](https://github.com/AnthonyFavier/images/blob/master/attitude_harass.gif)
![stop and look](https://github.com/AnthonyFavier/images/blob/master/attitude_non_coop_stopLook.gif)


