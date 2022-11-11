# NBV_Simulation_GMC
This is the nbv simulation system which can be compiled by Visual Studio 2019 with c++ 14 and run on Windows 10.
## Installion
These libraries need to be installed: opencv 4.4.0, PCL 1.9.1, Eigen 3.3.9, OctoMap 1.9.6, Gurobi 9.1.1. Note that Gurobi is only free for academic use.
## Note
For other system (such as Ubuntu), please check the file read/write or multithreading functions in the codes.
<br>
Change "const static size_t maxSize = 100000;" to "const static size_t maxSize = 1000" in file OcTreeKey.h, so that the code will run faster.
## Quick Start
Keep DefaultConfiguration.yaml in the correct path, and then run compiled program of main.cpp.
<br>
Input "0" in the Console.
## Usage
The mode of the system should be input in the Console.
### Mode 0 
The system will read the configuration in the yaml and run single case. You may change name_of_pcd, method_of_IG, sampling_method_of_viewspace, and first_view_id. For method_of_IG, MCMF is 0, OA is 1, UV is 2, RSE is 3, APORA is 4, Kr is 5, NBVNET is 6, PCNBV is 7 (not support), SCO is 8, OurBasic is 9, GMC is 10. For sampling_method_of_viewspace, 0 is random, 1 is uniform. For first_view_id, the initial view index is ranging from 0 to 1591.
### Mode 1
Give the object names in the Console (-1 to break input). Then give the tested method_of_IG in the Console. It will run with 5 initial views.
### Mode 2
Give the object names in the Console (-1 to break input). You will run the ablation studies. The saving path needs to be modifed in the share_data.hpp (save_path += '\_' + to_string(visble_rate); save_path += '\_' + to_string(move_rate);).
### Mode 3
Give the object names in the Console (-1 to break input). The maximum visible coveage will be computed for the certain view space of each object.
