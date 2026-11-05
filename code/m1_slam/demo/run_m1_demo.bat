@echo off
rem One-click M1 demo: opens 3 Ubuntu tabs (PX4+Gazebo, M1 launch, finish/save)
set S=/mnt/c/Users/kisha/PROJECTS/automotive/code/m1_slam/demo
wt -w new new-tab --title "1 PX4+Gazebo" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab1_px4.sh ; new-tab --title "2 M1 launch" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab2_launch.sh ; new-tab --title "3 finish" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab3_finish.sh
