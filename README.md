# GMC-based-NBV
This is our method with a NBV simulation system, supporting our paper "A global Generalized Maximum Coverage-based Solution to Non-model-based View Planning Problem for Object Reconstruction". Codes are coming soon.
## Installion
For our nbv_simulation c++ code, these libraries need to be installed: opencv 4.4.0, PCL 1.9.1, Eigen 3.3.9, OctoMap 1.9.6 and Gurobi 9.1.1.
For nbv_net, please follow https://github.com/irvingvasquez/nbv-net.
We tested our codes on Windows 10. For other system, please check the file read/write or multithreading functions in the codes.
## Note
Change "const static size_t maxSize = 100000;" to "const static size_t maxSize = 1000" in file OcTreeKey.h, so that the code will run faster.
## Questions
Please contact 18210240033@fudan.edu.cn
