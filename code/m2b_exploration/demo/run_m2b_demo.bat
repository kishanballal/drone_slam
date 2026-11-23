@echo off
rem One-click M2b demo (GPS off): PX4+Gazebo, M2b stack (Nav2), autonomous exploration + map save
set S=/mnt/c/Users/kisha/PROJECTS/automotive/code/m2b_exploration/demo
wt -w new new-tab --title "1 PX4+Gazebo" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab1_px4.sh %1 ; new-tab --title "2 M2b stack" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab2_stack.sh ; new-tab --title "3 explore" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab3_explore.sh
