# Efficient Divide-And-Conquer Ray Tracing using Ray Sampling

This project was a final project for INF584 - Image Synthesis course at Ecole Polytechnique de Paris. This project is implementation of the paper by K.Nabata [Efficient Divide-And-Conquer Ray Tracing using Ray Sampling](http://nishitalab.org/user/UEI/publication/Nabata_HPG2013.pdf)


Below are instruction on Windows. We assume CMake to be installed. On MS Windows, once CMake is executed, one can open the solution (.sln) file located in the build directory.

## Compiling

```
mkdir build
cd build
cmake ..
```

## Running 

```
MyRayTracer.sln
```
After the previous line, run in release mode in Visual Studio.

This will output a file named "output.ppm" by default, that can be visualized using InfraView on windows, Feh on linux or GIMP on any platform.

## Results
The results obtained with my implementation of EDACRT are as following:


![alt text](https://github.com/iradab/Efficient-Divide-And-Conquer-Ray-Tracing/blob/main/EDACRT.png)

In order to evaluate the speed up obtained by the EDACRT, the results were compared with naive ray tracing:

* The first image (with 518.400 rays and 1200 triangles)

was ray traced in 33 s with naïve ray tracing VS 17 s with EDACRT. 

By increasing the 
number of rays per pixel from 4 to 8, one would obtain more than a million rays – which would take 57 s
with naïve approach and 26 seconds with EDACRT for the same number of primitives.

* The second image (with 518.400 rays and 29983 triangles)

was ray traced in  526 s with naïve ray tracing VS 172s with EDACRT. 

By results obtained with several tests, it is possible to observe that by increasing the number of rays and 
primitives, the speedup from naïve ray tracing to EDACRT increases. Since in EDACRT the data 
structure is not stored, the memory usage of the algorithm is not of main interest to make a comparison.


## Authors

* **Irada Bunyatova**     [iradab](https://github.com/iradab)
