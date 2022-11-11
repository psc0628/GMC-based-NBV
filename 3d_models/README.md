# Input Object 3D models
Sample your 3d object model from *.obj or *.ply to *.pcd before put it into the NBV system.
<br>
For the sampling method, please follow the PCL mesh_sampling (https://github.com/PointCloudLibrary/pcl/blob/master/tools/mesh_sampling.cpp).
<br>
Run with "./pcl_mesh_sampling.exe *.ply *.pcd -n_samples 100000 -leaf_size 0.5 -no_vis_result" or -leaf_size 0.0005, depending on the model.
<br>
Make sure there are enough points over 10000.
<br>
This sampling will result in the holes in object surfaces, you can samller the size of the object to make surfaces closes.
