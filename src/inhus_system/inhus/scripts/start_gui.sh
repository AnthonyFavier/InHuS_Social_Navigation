#! /usr/bin/env bash

bokeh serve "$1/gui_server" --show --args $1 $2

