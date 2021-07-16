# InHuS Social Navigation

## A generic architecture

InHuS is an architecture thought to be as generic and modular as possible. It's purpose is to control a simulated autonomous human agent which is both reactive and rational. Given a goal, the agent plans and executes a set of actions in order to fulfill its goal. A user interface is provided to easily send goals to both other agents and the human one present in the scene. The interface makes it easy to repeat scenarios and run long term activity. A log manager is also part of the system. It records and computes execution data.

![global archi](https://github.com/AnthonyFavier/images/blob/master/global_archi_grand.png)

## Tailored to navigation

The provided implemented version is tailored to social robot navigation. Thus, plans are composed of positions to reach. Thanks to some unique features the system offers several rational human behaviors. One of the unique features is the ability to detect and act over path blockage by other agents.

![corridor](https://github.com/AnthonyFavier/images/blob/master/nav_hateb.gif)

![path blockage](https://github.com/AnthonyFavier/images/blob/master/path_blocked.gif)

![stop and look](https://github.com/AnthonyFavier/images/blob/master/attitude_non_coop_stopLook.gif)

![harass](https://github.com/AnthonyFavier/images/blob/master/attitude_harass.gif)

Dependencies :

- sudo apt-get install python3-rospkg-modules

- pointcloud_to_laserscan
