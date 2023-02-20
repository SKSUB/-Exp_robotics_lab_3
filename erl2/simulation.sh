#!/bin/bash

gnome-terminal --tab --title="robot_simulation" -- bash -c "roslaunch erl2 simulation.launch 2</dev/null"
gnome-terminal --tab --title="ARMOR" -- bash -c "sleep 1; rosrun armor execute it.emarolab.armor.ARMORMainService"
gnome-terminal --tab --title="gmapping" -- bash -c "sleep 1; roslaunch erl2 gmapping.launch 2</dev/null"
gnome-terminal --tab --title="nodes" -- bash -c "sleep 7; roslaunch erl2 nodes.launch 2</dev/null"
