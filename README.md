# Extended Kalman Filter Submission

## Objective
Implement an estimator to track the state <img src="https://render.githubusercontent.com/render/math?math=\mathbf{x}"> of a driving vehicle based on lidar and radar data. State <img src="https://render.githubusercontent.com/render/math?math=\mathbf{x}"> is a vector containing 2D position and 2D velocity:

<img src="https://render.githubusercontent.com/render/math?math=\mathbf{x}=(x, y, v_{x}, v_{y})^{T}">

The given lidar data <img src="https://render.githubusercontent.com/render/math?math=\mathbf{d}_{Lidar}"> are estimates of position in cartesian coordinates:

<img src="https://render.githubusercontent.com/render/math?math=\mathbf{d}_{Lidar}=(x, y)^{T}">

The radar data <img src="https://render.githubusercontent.com/render/math?math=\mathbf{d}_{Radar}"> contain positional information in polar coordinates as well as the radial velocity:
<img src="https://render.githubusercontent.com/render/math?math=\mathbf{d}_{Radar} = (\rho, \varphi, \dot{\rho})^{T}">
The provided suggestion to solve the tracking challenge was to evaluate lidar data with a Kalman Filter and the radar data with an extended Kalman Filter, since the transformation of polar to cartesian coordinates is non-linear (this will be described in the Implementation Details - Initialization section). The combination of both is referred to as Fusion EKF within this project.
## Implementation Details
### Initialization
To initialize the Fusion EKF the first measurement of either radar or data are used.
#### Lidar
In case lidar sends the first measurement <img src="https://render.githubusercontent.com/render/math?math=\mathbf{x}"> can easily be initialized by directly assigning it to <img src="https://render.githubusercontent.com/render/math?math=\mathbf{d}_{Lidar}">

<img src="https://render.githubusercontent.com/render/math?math=(x_{0}, y_{0})^{T}=\mathbf{d}_{Lidar, 0}">

About the initial velocity nothing can be said by only looking at the first lidar measurement, so it was initialized with 0: 

<img src="https://render.githubusercontent.com/render/math?math=(v_{x}, v_{y})^{T}=(0, 0)^{T}">

The initial state covariance matrix <img src="https://render.githubusercontent.com/render/math?math=\mathbf{P}"> was chosen to be a diagonal matrix, where the first element on the diagonal describes the variance of <img src="https://render.githubusercontent.com/render/math?math=x_{0}">, the second one the variance of <img src="https://render.githubusercontent.com/render/math?math=y_{0}">, etc. For the first two elements on the diagonal I chose the provided values <img src="https://render.githubusercontent.com/render/math?math=\sigma^{2}_{x}"> and <img src="https://render.githubusercontent.com/render/math?math=\sigma^{2}_{y}"> of the lidar covariance matrix. The remaining two, which describe the uncertainty of the velocity, I set to 100, because the current speed can only be guessed.  
#### Radar
With radar data things are a bit different. As mentioned before, the relation between polar and cartesian coordinates is non-linear:

<img src="https://render.githubusercontent.com/render/math?math=x_{0} = \rho_{0} \cdot cos(\varphi_{0})">

<img src="https://render.githubusercontent.com/render/math?math=y_{0} = \rho_{0} \cdot sin(\varphi_{0})">

Again, even though radar provides a velocity measure, namely speed in <img src="https://render.githubusercontent.com/render/math?math=\rho"> direction, that is not enough to get an accurate measure of <img src="https://render.githubusercontent.com/render/math?math=v_{x}, v_{y}">. I tried to use <img src="https://render.githubusercontent.com/render/math?math=\dot\rho_{0}"> to initialize <img src="https://render.githubusercontent.com/render/math?math=v_{x,0}, v_{y,0}">, but that approach yielded bad results. So for initial velocity estimation I did the same as with the lidar data.
Similarly to lidar I initialized <img src="https://render.githubusercontent.com/render/math?math=\mathbf{P}"> with the provided <img src="https://render.githubusercontent.com/render/math?math=\sigma^{2}_{\rho}"> for the positional variances and used 100 for the velocity variances.

### Predict and Update
I won't go in to detail here, what exactly I did, because this is basically using formulas. However I want to describe here what I did different from the Udacity class suggestions.
##### Implementation Order
 Instead of implementing everything at once, I first only implemented the formula for lidar. First of all, because the initialization step was more likely to be accurate for lidar, and second of all, because implementing a standard Kalman Filter is easier than an Extended Kalman Filter. So to get first results, I finished all steps for lidar, started the simulator and achieved my first mile stone by seeing, that the results were already very good with only lidar data. "Very good", only referring to the look, not on the RMSE.
 
 ##### Performance boosting
 I noticed that Udacity's code template for this project had a lot of heap allocation during run time. To avoid copying heap objects every cycle, I modified the Update and UpdateEKF methods, by adding the <img src="https://render.githubusercontent.com/render/math?math=\mathbf{H}"> and <img src="https://render.githubusercontent.com/render/math?math=\mathbf{R}"> matrices to the arguments list of these methods, and passing them by const reference. 
 Another improvement I made was due to the fact, that the matrices used are constant in their size, so saving them on the heap is unnecessary. With template notation it is possible to initialize constant size matrices, which are then allocated on the stack. I could have done this with the whole project code, but this would have been a bit tedious. However, Update and UpdateEKF methods show that it is possible.
 
 ## Fusion EKF validation
 The precision goal is:
 
<img src="https://render.githubusercontent.com/render/math?math=RMSE((x, y, v_{x}, v_{y})) \le (.11, .11, 0.52, 0.52)">

for the last point the combined evaluation of both lidar and radar data yields

<img src="https://render.githubusercontent.com/render/math?math=v_{x}:0.4940,v_{y}:0.4543,x:0.1067,y:0.0928">

which is quite the improvement compaired to lidar only:

<img src="https://render.githubusercontent.com/render/math?math=v_{x}:0.6625,v_{y}:0.5266,x:0.1477,y:0.1149">

Especially interesting is the development of <img src="https://render.githubusercontent.com/render/math?math=v_x">, because it takes quite long to get below the conversion goal, even though I tried covariances of 10000.

 
