#! /usr/bin/env bash

roslaunch inhus_navigation _inhus_cohan_nav.launch map_name:=$(rosparam get /map_name)

