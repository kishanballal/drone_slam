@echo off
rem One-click M4 demo (GPS off): explore, then the drone plans its OWN search route
rem (rooms from its map, fewest stops, shortest order), flies it and pins victims.
rem   run_m4_demo.bat        headless Gazebo (faster)
rem   run_m4_demo.bat gui    with the Gazebo window (for recordings)
set S=/mnt/c/Users/kisha/PROJECTS/automotive/code/m4_search_route/demo
wt -w new new-tab --title "1 setup+PX4" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab1_px4.sh %1 ; new-tab --title "2 M2b stack" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab2_stack.sh ; new-tab --title "3 3D map+victims" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab3_m3.sh ; new-tab --title "4 M4 search route" wsl.exe -d Ubuntu-24.04 -- bash %S%/tab4_demo.sh
