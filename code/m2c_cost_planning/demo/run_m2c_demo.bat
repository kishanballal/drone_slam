@echo off
rem One-click M2c demo (GPS off): exploration with the cost-aware planner, then moving obstacles + real-time replanning
set S=/mnt/c/Users/kisha/PROJECTS/automotive/code/m2c_cost_planning/demo
wt -w new new-tab --title "1 PX4+Gazebo" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab1_px4.sh %1 ; new-tab --title "2 M2c stack" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab2_stack.sh ; new-tab --title "3 demo" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab3_demo.sh
