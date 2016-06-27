# Benchmarking the C++ and Matlab implementations of Drake

Ignoring the hard-coded URDF name, here are the results:

## Matlab
```bash
Elapsed time is 6.406154 seconds.
RigidBodySystem has 64 states and 32 inputs
Elapsed time is 3.210128 seconds.
```

## C++
```bash
$ benchmark_dynamics 
welding left_camera_optical_frame_joint because it has no inertia beneath it
welding right_camera_optical_frame_joint because it has no inertia beneath it
Time elapsed: 0.070279
RigidBodySystem has 64 states and 32 inputs
Time elapsed: 5.10138
```
