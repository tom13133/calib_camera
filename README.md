# ROS package to estimate the transformation between 3D-2D  correspondences.

The package is used to compute the transformation between 3D-2D correspondences (3D-sensor to camera).  

## Dependencies
1. Eigen3
2. Ceres Solver
3. Sophus

* Installation of Ceres Solver and Sophus: https://hackmd.io/LlT0q5UOQW6YiXNsuGVY4g
## Nodes
1. Node **camera_3D_2D_calib_node**

## 1. Node camera_3D_2D_calib_node
This node is used to estimate the 6 degree of freedom (DoF) transformation from 3D-2D correspondences (3D-sensor to camera).  

### (a) Dataset
This node reads dataset containnig 3D-2D correspondences.  
The dataset shall follow the format as follow:  

1. path:  
Its path is ```~/calib_3d/dataset/correspondences.csv```

2. format:
#### data.csv
> x, y, z, u, v  
> -3.67334, 9.61517, 0.626889, 766, 250  
> -3.05844, 9.89671, -0.582504, 809, 336  
> -4.20264, 9.07678, -0.535, 714, 329  
> ...  

Each correspondence includes (x, y, z, u, v),  
where **(x, y, z)** is the 3D point, and **(u, v)** is its corresponding point on image.  
Notice that the first line in **data.csv** will be ignored.  

### (b) Setup
Before start running the calibration module, there is one configuration file that need to be specfied.  
We use ROS Parameter Server to set the configurations.  
1. path:  
Its path is ```~/calib_camera/config/default.yaml```  

2. format:
#### default.yaml
> initial_guess_camera_TO_3D_sensor:  
>   translation: [0, 0, 0]  
>   rotation: [20,0,-90]  
> intrinsic_parameters:  
>   matrix: [698.936, 0.0, 641.868, 0.0, 698.936, 385.788, 0.0, 0.0, 1.0]  
> distortion_parameters:  
>   k: [-0.171466, 0.0246144]  

where parameter **initial_guess_camera_TO_3D_sensor** is needed to be specified, which contains translation (x, y, z) and rotation (yaw, pitch, roll),  
**intrinsic_parameters/matrix** is the intrinsic matrix.  
**distortion_parameters/k** determines distortion paramters.  

### (c) Getting Started.
```
roslaunch calib_camera camera_3D_2D_calib.launch
```


* Result:  
Red: source points  
Green: target points  
White: result points (transformed source points)  
<img src="https://github.com/tom13133/calib_camera/blob/master/images/result.png" width="800">
