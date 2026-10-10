# Mathematical Foundations: Autonomous Vehicle Navigation

This document outlines the core mathematical models governing the AI-Based Autonomous Navigation System.

---

## 1. Kinematic Bicycle Model (Non-Holonomic Motion)

For ground vehicles operating at modest speeds without lateral tire slip, the **kinematic bicycle model** provides a rigorous and physically accurate model of planar motion. The two wheels of each axle are collapsed into single central virtual wheels.

### State Equations
Given the state vector $\mathbf{x} = [x, y, \theta, v]^T$, where:
- $(x, y)$: Rear axle center position in world frame (meters)
- $\theta$: Vehicle orientation / yaw angle relative to the X-axis (radians)
- $v$: Longitudinal linear speed (m/s)
- $\delta$: Front wheel steering angle (radians)
- $L$: Vehicle wheelbase (distance between front and rear axles)
- $a$: Commanded longitudinal acceleration (m/s$^2$)

The continuous-time kinematics are:
$$\dot{x} = v \cos(\theta)$$
$$\dot{y} = v \sin(\theta)$$
$$\dot{\theta} = \frac{v}{L} \tan(\delta)$$
$$\dot{v} = a$$

### Discrete-Time Integration (Forward Euler)
For simulation step $\Delta t$:
$$x_{k+1} = x_k + v_k \cos(\theta_k) \Delta t$$
$$y_{k+1} = y_k + v_k \sin(\theta_k) \Delta t$$
$$\theta_{k+1} = \theta_k + \frac{v_k}{L} \tan(\delta_k) \Delta t$$
$$v_{k+1} = \text{clamp}(v_k + a_k \Delta t, 0, v_{\max})$$

---

## 2. Global Path Planning: Inflated $A^*$ Algorithm

The $A^*$ algorithm searches for an optimal path from node $s$ to node $g$ on an 8-connected grid by minimizing the total evaluation function:
$$f(n) = g(n) + h(n)$$

Where:
- $g(n)$: Exact cumulative cost from start node $s$ to current node $n$.
- $h(n)$: Admissible heuristic estimating remaining cost from $n$ to goal $g$.

### Heuristic Function (Euclidean Metric)
$$h(n) = \sqrt{(x_n - x_g)^2 + (y_n - y_g)^2}$$

Because $h(n) \le d(n, g)$ (straight-line distance never overestimates true travel distance), the heuristic is strictly **admissible** and **consistent**, guaranteeing that $A^*$ returns an optimal path.

### Obstacle Costmap Inflation
To ensure the vehicle's physical footprint does not collide with obstacles, all obstacle boundaries are expanded by an inflation radius $R_{inf}$:
$$R_{inf} \ge \sqrt{(L/2)^2 + (W/2)^2} + \epsilon_{safety}$$

---

## 3. Lateral Path Tracking: Pure Pursuit Geometry

The Pure Pursuit controller calculates the steering angle required to chase a moving target point along the planned trajectory at a lookahead distance $L_d$.

### Geometric Derivation
Let $\alpha$ be the angle between the vehicle's heading vector and the line of sight to the lookahead waypoint $(x_t, y_t)$:
$$\alpha = \text{atan2}(y_t - y, x_t - x) - \theta$$

By the Law of Sines on the resulting isosceles triangle:
$$\frac{L_d}{\sin(2\alpha)} = \frac{R}{\sin(\frac{\pi}{2} - \alpha)} = \frac{R}{\cos(\alpha)}$$
$$2 R \sin(\alpha) = L_d \implies R = \frac{L_d}{2 \sin(\alpha)}$$

The path curvature $\kappa = \frac{1}{R}$ is:
$$\kappa = \frac{2 \sin(\alpha)}{L_d}$$

Relating curvature to steering angle $\delta$ via the bicycle model ($\tan(\delta) = \frac{L}{R} = L \kappa$):
$$\delta = \arctan\left(\frac{2 L \sin(\alpha)}{L_d}\right)$$

---

## 4. Longitudinal Speed Control: PID Velocity Profiling

The longitudinal controller regulates throttle and braking acceleration commands $a(t)$ to achieve desired cruise velocity $v_{target}$:

$$e(t) = v_{target}(t) - v(t)$$
$$a(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

With anti-windup clamping applied to the integral accumulator to prevent overshoot during sustained acceleration.
