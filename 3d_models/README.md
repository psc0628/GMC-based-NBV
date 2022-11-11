# Input Object 3D models
Sample your 3d object model from *.obj or *.ply to *.pcd, and there is an example in directory 3d_models. For the sampling method, please follow https://github.com/PointCloudLibrary/pcl/blob/master/tools/mesh_sampling.cpp, and run with "./pcl_mesh_sampling.exe *.ply *.pcd -n_samples 100000 -leaf_size 0.5 -no_vis_result" or -leaf_size 0.0005, depending on the model.
