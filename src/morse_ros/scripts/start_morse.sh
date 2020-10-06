#! /usr/bin/env bash

# add resources of current package to morse
export MORSE_RESOURCE_PATH=$MORSE_RESOURCE_PATH:`rospack find morse_ros`/blender_files

# filter arguments
args=( "$@" )
unset "args[${#args[@]}-1]"
unset "args[${#args[@]}-1]"

# start morse with given scenario and arguments
morse run -g 1280x720 ${args[@]}
