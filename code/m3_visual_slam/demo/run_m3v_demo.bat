@echo off
rem One-click M3v demo (GPS off): LiDAR stack + memory-guarded camera + RTAB-Map visual SLAM during exploration
set S=/mnt/c/Users/kisha/PROJECTS/automotive/code/m3_visual_slam/demo
wt -w new new-tab --title "1 setup+PX4" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab1_px4.sh %1 ; new-tab --title "2 M2b stack" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab2_stack.sh ; new-tab --title "3 camera vSLAM" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab3_vslam.sh ; new-tab --title "4 demo" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab4_demo.sh
