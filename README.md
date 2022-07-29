# Cooperative-object-transport-task-with-multiple-UAVs
This work aims to design two drones able to pick up objects and take them to a new location through a hierarchical control. For each of the two drones, a route search algorithm has been implemented that allows it to avoid obstacles of the environment. The two UAVs must be synchronized because the second UAV takes the object from the place where the first UAV has brought it. The mass of the objects to be picked is not known and it is estimated using an estimator in run time.
## Code Information
The project consists of 3 files:

Trajectory_Mirzaagha_Trinchese.m: This file contains the code for generating a reference trajectory for the two UAVs. In the first lines of code there is a small comment section useful if you want to import a workspace, taking only the essentials with you. It is not strictly necessary for the program to work, it was only necessary for us in the testing phase to be able to work on the same trajectory, however, scaling the times for the appropriate tests. Possibly, the necessary to save in the workspace in order to perform the operation is:
 PATH_TL, PATH_TL2, TOTAL_COORD_PATH, TOTAL_COORD_PATH2, TOTAL_COORD_PATHenu, TOTAL_COORD_PATH2enu.

Final_Project_Mirzaagha_Trinchese.slx: In this file there is the simulink schema for the simulation but it is necessary to underline two things:
-the 3D visualization of the uavs does not happen in this file: The simulation is already heavy on its own and if you add the 3D visualization too the situation gets worse. For this reason, the data that would normally be sent to the UAV blocks are instead sent to a "To workspace" to be used in the other simulink file. If you want to see the 3D animation in this file, just uncomment the UAV toolbox blocks in this file.
-If you want to change the order of the estimator just comment the old estimator, uncomment the new one and reconnect in the sum blocks incestimr2 instead of incestim or vice versa. The blocks sum to obtain the total estimated final mass are two, one for each uav.

VIDEO_SIM.slx
Here, the data saved with the "To Workspace" in the other simulink file is used. In this scheme there are only the blocks necessary for the visualization, so by running this scheme the animation is decidedly more fluid and rapid.

STEP:
1) run the matlab code
2) run Final_Project_Mirzaagha_Trinchese.slx
3) (if you want to see the 3d animation) run VIDEO_SIM.slx or uncomment the UAV toolbox blocks of the Final_Project_Mirzaagha_Trinchese.slx file
## Collaboration
This work was carried out in cooperation with the engineer Marco Trinchese
