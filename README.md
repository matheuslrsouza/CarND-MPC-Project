# Model Predictive Control

MPC has the ability to anticipate future events and can take control actions accordingly.
The main advantage of MPC is the fact that it allows the current timeslot to be optimized, while keeping future timeslots in account. This is achieved by optimizing a finite time-horizon, but only implementing the current timeslot and then optimizing again, repeatedly.
[Retrived from wikipedia](https://en.wikipedia.org/wiki/Model_predictive_control)

## The problem

The main goal of this project is to make the car complete a lap without leaving the track. For this we have to control the 2 actuators of the vehicle (acceleration and steering angle) to make the car follow a trajectory.

<img src='/media/mpc-vid2.gif'>

*The yellow is the trajectory and the green line represents the x and y coordinates of the MPC trajectory. Retrieved from Udacity*

## Model

### State

In order to predict the best action that the vehicle should perform in the next step (`t + 1`), we need to store the current state of the vehicle, we do this with the following elements

* **x:** The x-point of the vehicle in its own coordinate system. Since the vehicle is always at the origin of your system, x will always be equal to 0

* **y:** The y-point of the vehicle in its own coordinate system. Since the vehicle is always at the origin of your system, y will always be equal to 0

* **psi:** The orientation of the vehicle in its own coordinate system. In the same way as x and y, it will always be zero, because the system always starts from its orientation

* **v:** The actual velocity of the vehicle, received by the simulator at each iteration

* **cte:** The actual cross track error. It is always the distance on the y-axis between where the vehicle is and where it should be, according to the trajectory. To calculate this, the coefficients of the trajectory polynomial are used to know where the car should be
```c++
polyeval(coeffs, x)
```
Since the vehicle y is always 0, no subtraction is required

* **epsi:** Error in the direction of the vehicle compared to the trajectory. To calculate this difference, we first calculate the derivative of the polynomial to obtain the slope at point x. Then we find the angle at this point using the arctangent.
**ex:**
```c++
double derivative_poly = 3 * coeffs[3] * pow(x, 2) + 2 * coeffs[2] * x + coeffs[1];
epsi = pis - atan(derivative_poly);
```
Because x is always equal to 0, we can neglect the terms involving x, in the same way we do with the psi that always is 0
```c++
double derivative_poly = coeffs[1];
epsi = -atan(derivative_poly);
```

<img src='/media/vehicle_coord.png' width="200px">

*Vehicle coordnate system: Retrived from Udacity*

### Actuators

The actuators are the inputs we send to the simulator to be able to make our vehicle move

* **delta** Angle in radians, that the vehicle should be steered. The maximum and minimum that the vehicle can steer is 25°, being `-0.436332` and` 0.436332` in radians

* **a** It allows to accelerate or to brake our vehicle, being that the maximum and minimum allowed values are `-1` and` 1`. Negative values mean brake

### Update equations

Below are the equations used to calculate the next state

```c++
for (int t = 1; t < N; t++) {
  ...
  AD<double> x_2 = x0 * x0;
  AD<double> x_3 = x_2 * x0;      
  AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x_2 + coeffs[3] * x_3;
  AD<double> psi_dest = CppAD::atan(3 * coeffs[3] * x_2 + 2 * coeffs[2] * x0 + coeffs[1]);

  fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt); //x
  fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt); //y
  fg[1 + psi_start + t] = psi1 - (psi0 - (v0/Lf) * delta0 * dt); //psi
  fg[1 + v_start + t] = v1 - (v0 + a0 * dt); //v
  fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt)); //cte
  fg[1 + epsi_start + t] = epsi1 - ((psi0 - psi_dest) - (v0/Lf) * delta0 * dt); //epsi
}
```

The `Lf` is a constant with value `2.67` and this represents the length from front to CoG (center of gravity) of vehicle

Variables with suffix `1`, example` x1`, are variables calculated by Solver and are one step ahead (t + 1) of variables with a suffix `0`. What the solver tries to do is minimize the error according to our passed contraints (see below), in which case it tries to make the above values equal zero. So we calculate the difference between what the solver calculated (`x1`) and what our model calculates (`x0 + v0 * CppAD::cos(psi0) * dt)`), so that it can check which element is contributing more to the error and can minimize it.

### Solver

The minimization of errors is done by the `Ipopt` library, which is responsible for minimizing the error using our model and our passed constraints

```c++
// Lower and upper limits for the constraints
Dvector constraints_lowerbound(n_constraints);
Dvector constraints_upperbound(n_constraints);
for (int i = 0; i < n_constraints; i++) {
  constraints_lowerbound[i] = 0;
  constraints_upperbound[i] = 0;
}
```

This is how we report to the solver, how we want the result of the equations to be, so that it will be able to calculate the best values for that to happen, in which case trying to get the results of the equations to be equal 0.

The code below describes how we invoke `MPC::Solver` to get the path it has defined

```c++
vector<double> vars = mpc.Solve(state, coeffs);
```

After that the variables are used to draw the predicted path (green line) and to perform the actuators

```c++
vector<double> vars = mpc.Solve(state, coeffs);
double steer_value = vars[0];
json msgJson;
msgJson["steering_angle"] = steer_value / (deg2rad(25) * Lf);
msgJson["throttle"] = vars[1];

//Display the MPC predicted trajectory 
vector<double> mpc_x_vals;
vector<double> mpc_y_vals;

for (int i = 2; i < vars.size(); i++) {
  double point = vars[i];
  if (i % 2 == 0) {
    mpc_x_vals.push_back(point);
  } else {
    mpc_y_vals.push_back(point);
  }
}
```
We only use the first item of the trajectory to perform the actuators, in the next step the whole process is repeated and recalculated

### Timestep Length and Elapsed Duration (N & dt)

The final values chosen for `N` and `dt` were `10` and `1.3`. I think 10 meters ahead of the vehicle is a good value for which a route should be considered, a higher value than this can cause a bad performance in the solver and does not add value to the result. As for `dt` I tried different values, like 2.0 and 0.05, but none of these values caused the vehicle to complete a lap.

### Polynomial Fitting and MPC Preprocessing

To calculate the two errors that we store (cte and epsi), we need the trajectory to know how far we are and how wrong the orientation is. The way to do this is to use the polynomial of the points of the trajectory, but before we calculate the coefficients, it is necessary to first transform the points of the trajectory to the coordinate system of the vehicle

```c++
//rotates counterclockwise 90 degree, to convert from polar navigation
double psi_rotated = psi_unity - (pi() / 2);

Eigen::VectorXd xvals(ptsx.size());
Eigen::VectorXd yvals(ptsx.size());

for (int i = 0; i < ptsx.size(); i++) {
  //difference between the points and the vehicle position, in order to shift the 
  //trajectory to the right place (center of track)
  double delta_x = ptsx[i] - px;
  double delta_y = ptsy[i] - py;

  //tranformation from map system to vehicle coordinate system
  xvals(i) = cos(psi_rotated) * delta_x - sin(psi_rotated) * delta_y;
  yvals(i) = sin(psi_rotated) * delta_x + cos(psi_rotated) * delta_y;
}
```
[Coordinate Transformation](https://www.miniphysics.com/coordinate-transformation-under-rotation.html)


After that the coefficients can already be calculated

```c++
auto coeffs = polyfit(xvals, yvals, 3);
```

The line of the trajectory is presented on the basis of the calculated coefficients

```c++
for (int x = 1; x < 20; x++) {
  next_x_vals.push_back(x * 2.5);
  next_y_vals.push_back(polyeval(coeffs, x * 2.5));
}
```

### Complete lap

Below is a video of the car in a complete lap

[![complete lap](https://img.youtube.com/vi/IReRAXvSulc/0.jpg)](https://youtu.be/IReRAXvSulc)