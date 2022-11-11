# GMC-based-NBV
This is a repository for supporting our paper "A Global Generalized Maximum Coverage-based Solution to the Non-model-based View Planning Problem for Object Reconstruction" (DOI: Coming soon).
<br>
<br>
This GMC method is a somehow imporved version of our previous work Max-Flow-based method: "A Global Max-flow-based Multi-resolution Next-best-view Method for Reconstruction of 3D Unknown Objects" (doi: 10.1109/LRA.2021.3132430, Codes: https://github.com/psc0628/NBV-Simulation).
## File structure
3d_models 
The repository has five folders and two files.
<br>
3d_models contians an example of input object 3D model (a pcd file sampled by a ply file). 
<br>
GMC contains an example of output results (point clouds, girds and indicators) with viewspaces and the maximum visible coverage.
<br>
NBV_Simulation_GMC contains c++ codes of NBV planning simulation system for object reconstruction.
<br>
nbv-net-sphere contains python codes of our re-trained NBV-Net network and its interface. 
<br>
DefaultConfiguration.yaml contains the configurations for c++ codes.
<br>
pack.3.1592.txt is the uniform sampled view space by Shpere Codes (http://neilsloane.com/packings/).
## Questions
Please contact 18210240033@fudan.edu.cn or 425145649@qq.com
