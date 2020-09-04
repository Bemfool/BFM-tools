# A C++ Impl Tool for Basel Face Model

A simple tool to use Basel Face Model (BFM) in C++. 

## Depend on:

* Basel Face Model (version>=2017, because we need expression information);
* Eigen3
* [HDF5](https://www.hdfgroup.org/downloads/hdf5) (version=1.12);
* [Dlib](http://dlib.net/) (version=19.21);
* [OpenCV](https://github.com/opencv/opencv) with [opencv_contrib](https://github.com/opencv/opencv_contrib) (version<=3.4.10, because OpenCV4 remove some basic data structure like CvMat)

*Note: You could use your own dataset whose format is similar with Basel Face Model, and you should change corresponding parameters.*

## How to use:

See `example/ex_gen_faces.cpp`. You need know information about your dataset as follows:

```
/path/to/basel_face_model.h5
n_vertice n_face n_id_pc n_expr_pc
fx fy cx cy
/h5_path/to/shape_mu /h5_path/to/shape_ev /h5_path/to/shape_pc
/h5_path/to/tex_mu /h5_path/to/tex_ev /h5_path/to/tex_pc
/h5_path/to/expr_mu /h5_path/to/expr_ev /h5_path/to/expr_pc
/h5_path/to_triangle_lists
n_landmark /path/to/landmark.anl
```

If landmark is not needed, use `0` to replace `n_landmark /path/to/landmark.anl`  .

If needed, `landmark.anl` format is as follow:

```
landmark_idx[0] 
landmark_idx[1]
...
landmark_idx[n_landmark-1]
```

