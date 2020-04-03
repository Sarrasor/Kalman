# Kalman Filters
## Task Description
The task is to track a pedestrian using LIDAR and RADAR sensors. Each of the sensor has its strengths and weaknesses. We will use Kalman Filter to fuse data from both.

Our sensor team did all necessary filtering and tracking of LIDAR point clouds and RADAR data, and provides us with x-y position from LIDAR and rho-phi-rho dot data from RADAR.

### LIDAR
**LI**ght **D**etection **A**nd **R**anging

### RADAR
**RA**dio **D**etection **A**nd **R**anging

## Math

### System
We have a discrete linear system of the following form:

$\begin{cases}x_{i+1} = Ax_{i} + Bu_{i} + w \\ y_{i} = Cx_{i} + v\end{cases}$

where:
$x_{i}$ - state vector of the system at time $i$ \
$u_{i}$ - input(also called control) vector at time $i$ \
$y_{i}$ - output vector at time $i$. Usually represents quantities we can measure \
$A$ - state transition matrix. How $x$ evolves over time\
$B$ - input matrix. Represents how $u_{i}$ influences on $x_{i}$ \
$C$ - output matrix(also called measurement matrix). Represesents the relation between $y_{i}$ and $x_{i}$ \
$w$ - process noise. Random variable with covariane matrix $W$ \
$v$ - measurement noise. Random variable with covariance matrix $V$  

Since the real state $x$ of the system is not available, we will try to estimate it with $\hat x$

In our case, we want to track a pedestian. Assume we are sitting in a self-driving car and interested in the location of the pedestian on x-y plane. We do not know anything about $u$(where and when pedestrian wants to go), because we are not all-seeing wizards. We will incude $u$ into our process noise $w$. Given this information, we represent state vector $\hat x$ and input vector $u$ as:

$\hat x= \begin{bmatrix} p_x \\ p_y \\ v_x \\ v_y \end{bmatrix}$

$u = 0$

We will use constant velocity kinematic model:

$\begin{cases}p_x = p_x + dtv_x \\ p_y = p_y + dtv_y \\ v_x = v_x \\ v_y = v_y\end{cases}$

Using the model we define  state transition and input matrices as follows:

$A = \begin{bmatrix} 1 & 0 & dt & 0  \\ 0 & 1 & 0 & dt \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1\end{bmatrix}$

$B = 0$ 

We have two sensors, thus two models of measurement.

$y_L = \begin{bmatrix} p_x \\ p_y\end{bmatrix}$

$y_R = \begin{bmatrix} \rho \\ \phi \\ \dot \rho\end{bmatrix}$

Given $\hat x$, $y_L$, and $y_L = C_L\hat x$, we can define $C_L$:

$C_L = \begin{bmatrix} 1 & 0 & 0 & 0  \\ 0 & 1 & 0 & 0\end{bmatrix}$

The problem is, we cannot define the $C_R$ as a matrix. It is a nonlinear vector function of $x$. Why? How to derive? Check the **Appendix** section. Nonlinearity alert! We will see how to deal with nonlinearities later.

$C_R(\hat x') = \begin{bmatrix} \sqrt{p_x'^2+p_y'^2}  \\ \arctan {(\frac{p_y'}{p_x'})}\\ \frac{p_x'v_x'+p_y'v_y'}{\sqrt{p_x'^2+p_y'^2}} \end{bmatrix}$

> **Tip**: To find $C$ for each sensor firstly think about dimentions of $C$.
>  $y = Cx$ should help.

### What are $w$ and $v$
$w$ - process noise. Includes everything that our model does not take into account. Friction, sudden wind, slipping, etc.  Often represented as a multivariate Gaussian distribution with zero mean and covariance matrix $W$. 

$v$ - measurement noise. Our sensors are not ideal, measurements contain some noise that we have to deal with. Often represented as a multivariate Gaussian distribution with zero mean and covariance matrix $V$. Sensor manufacturer should provide $V$ for you, or you can calculate it yourself. 

In our case:

$W = \begin{bmatrix} \frac{dt^4}{4}\sigma_{ax}^2 & 0 & \frac{dt^3}{2}\sigma_{ax}^2   & 0  \\ 0 & \frac{dt^4}{4}\sigma_{ay}^2 & 0 & \frac{dt^3}{2}\sigma_{ay}^2 \\  \frac{dt^3}{2}\sigma_{ax}^2 & 0 & dt^2\sigma_{ax}^2 & 0 \\ 0 &  \frac{dt^3}{2}\sigma_{ay}^2 & 0 & dt^2\sigma_{ay}^2 \end{bmatrix}$

Check **Appendix** for derivation of W

$V_L = \begin{bmatrix} 0.0225 & 0  \\ 0 & 0.0225\end{bmatrix}$

$V_R =\begin{bmatrix} 0.09 & 0  & 0 \\ 0 & 0.0009 & 0 \\ 0 & 0 & 0.09\end{bmatrix}$  

### Kalman filter
We estimate $\hat x$ in two steps:
1) Dynamic update (also called Prediction)
$\hat x_{i|i-1}$ - is the i-th step estimate after dynamics update; also called an a priori estimate.
2) Sensor update (also called Measurement update)
$\hat x_{i|i}$ - is the estimate after sensor update; also called an a posteriori estimate

From now on I will use sligtly different notation for $\hat x_{i|i}$ and $\hat x_{i|i-1}$. Since we always use previous estimated state to predict the next one, $i$ notation is redundant. So, $\hat x_{i|i}$ becomes $\hat x$, and $\hat x_{i|i-1}$ becomes $\hat x'$

With new notation Kalman filter equations become:
1) Prediction
$\hat x' = A\hat x + Bu$
$P' = APA^{T}+W$
2) Measurement
$Y = CP'C^{T} + V$
$K = P'C^{T}Y^{-1}$
$\hat x = \hat x' + K(y - C\hat x')$
$P = (I - KC)P'$

In our situation we will use Kalman Filter to track 2D position and velocity of a pedestrian using only noisy LIDAR measurements. The cool thing is - Kalman Filter will handle noise. Another cool thing - we can estimate velocity of the pedestrial by only measuring her position.

You can check the `KF` folder to run Kalman Filter for LIDAR measurements.

### Extended Kalman filter
$x$ and $y$  are functions in general: $x' = f(x, u)$, $y=g(x')$ .  Extended Kalman filter is used in case of nonlinearities in $f$ or $g$. This means, we cannot represent state transition and input matrces as matrices. Why nonlinear is bad for us? Check **Appendix**. What we can do, is we can use Taylor expansion to approximate $f$ and $g$.

> **Taylor expansion reminder**
Assume we have scalar function f(x) and we want to approximate it with a linear function at point $\mu$. The Taylor expansion will look like this:
>
>$f(x) \approx f(\mu) + \frac{\partial f(\mu)}{\partial x}(x-\mu)$
> 
> We can also expand this definition to vectors:
> 
> $T(x) = f(\mu) + (x-\mu)^{T}Df(\mu) + (x-\mu)^{T}D^2f(\mu)(x-\mu) + ...$
> 
>  $Df(\mu)$ is called the Jacobian matrix and $D^2f(\mu)$ is called the Hessian matrix. They represent first order and second order derivatives of multi-dimensional equations. A full Taylor series expansion would include higher order terms as well for the third order derivatives, fourth order derivatives, and so on.

Updated Kalman filter equations look like this:
1) Prediction
$\hat x' = f(\hat x, u)$
$P' = A_{j}PA_{j}^{T}+W$
2) Measurement
$Y = C_{j}P'C_{j}^{T} + V$
$K = P'C_{j}^{T}Y^{-1}$
$\hat x = \hat x' + K(y - g(\hat x'))$
$P = (I - KC_{j})P'$

In our case $f(\hat x, u)$ is linear, and can be represented with $A$ matrix. So, $A_{j} = A$. 

$g(\hat x')$ is more interesting. To derive a linear approximation for the RADAR's $g(\hat x')$ function, we will only keep the expansion up to the Jacobian matrix $Df(\mu)$. We will ignore the Hessian matrix $D^2f(\mu)$ and other higher order terms. Assuming $(x - \mu)(x−\mu)$ is small, $(x - \mu)^2$ or the multi-dimensional equivalent $(x-\mu)^T (x - \mu)$ will be even smaller; the extended Kalman filter we'll be using assumes that higher order terms beyond the Jacobian are negligible.

In our case we have to calculate this expression:

$C_j = \frac{\partial C_R(\hat x)}{\partial \hat x}$

Ok, we need to calculate some partial derivatives. Here is the result:

$C_j = \begin{bmatrix} \frac{p_x}{\sqrt{p_x^2+p_y^2} } & \frac{p_y}{\sqrt{p_x^2+p_y^2} } & 0 & 0  \\- \frac{p_y}{p_x^2+p_y^2 } & \frac{p_x}{p_x^2+p_y^2 } & 0 & 0 \\ \frac{p_y(v_xp_y-v_yp_x)}{(p_x^2+p_y^2)^{3/2}} & \frac{p_x(v_yp_x-v_xp_y)}{(p_x^2+p_y^2)^{3/2}} & \frac{p_x}{\sqrt{p_x^2+p_y^2} } & \frac{p_y}{\sqrt{p_x^2+p_y^2}}\end{bmatrix}$

 Check **Appendix** for calculations.

Now we will integrate RADAR sensor to our previous Kalman Filter with the use of Extended Kalman technique.

You can check the `EKF` folder to run Extended Kalman Filter for LIDAR and RADAR measurements fusion.

### Unscented Kalman filter
**TODO**

## Appendix
### Deriving $C_R$
Ok, we have the following situation:

### Deriving $W$
Since we don not know the acceleration

### Evil nonlinearities

### Calculating Jacobian matrix for RADAR

## References
Idea and parts of code are taken from [CarND-Extended-Kalman-Filter-Project](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) \
Simulator can be found here [self-driving-car-sim](https://github.com/udacity/self-driving-car-sim) \
Notation is taken from [Linear-Control-Slides-Spring-2020](https://github.com/SergeiSa/Linear-Control-Slides-Spring-2020) \
Written with [StackEdit](https://stackedit.io/)