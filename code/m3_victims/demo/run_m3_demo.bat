@echo off
rem One-click M3 demo (GPS off): exploration + 3D LiDAR map + victim detection in building_victims
set S=/mnt/c/Users/kisha/PROJECTS/automotive/code/m3_victims/demo
wt -w new new-tab --title "1 setup+PX4" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab1_px4.sh %1 ; new-tab --title "2 M2b stack" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab2_stack.sh ; new-tab --title "3 3D map+victims" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab3_m3.sh ; new-tab --title "4 demo" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab4_demo.sh
