#! /usr/bin/env bash

roslaunch inhus_navigation _cohan_nav.launch map_name:=$(rosparam get /map_name) 

