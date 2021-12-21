#! /usr/bin/env bash

roslaunch inhus_navigation _inhus_nav.launch map_name:=$(rosparam get /map_name)

