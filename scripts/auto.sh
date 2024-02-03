#!/bin/bash

simulate_input() {
    xdotool type "$1"
    xdotool key Return
    sleep 1
}

split_vertical() {
    xdotool key Ctrl+Shift+E
    sleep 1
}

split_horizontal() {
    xdotool key Ctrl+Shift+O
    sleep 1
}

bring_to_front() {
    wmctrl -r :ACTIVE: -e 0,0,0,-1,-1
}

focus_on_window() {
    wmctrl -a "$1"
}

python3 cool.py

terminator_width=$(xdpyinfo | awk '/dimensions:/ {print $2}' | cut -d'x' -f1)
terminator_height=$(expr $(xdpyinfo | awk '/dimensions:/ {print $2}' | cut -d'x' -f2))

terminator --geometry=${terminator_width}x${terminator_height}+0+0 &
bring_to_front
sleep 1

simulate_input "roscore"
split_horizontal
simulate_input "./pub.py"
split_vertical
simulate_input "roslaunch rocket_description gazebo.launch"
sleep 1
focus_on_window "Terminator"
split_vertical
simulate_input "./sundar.py"
focus_on_window "Terminator"
xdotool key Alt+Up
split_vertical
simulate_input "./visual.py"
focus_on_window "Terminator"
