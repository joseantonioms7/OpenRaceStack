# State Estimation Mathematical Formulations

ApexPathAI incorporates two complementary state estimators:
1. **Planar 5-State CTRV EKF** (`LocalizationEKF`)
2. **3D 15-DOF Error-State Kalman Filter on $\text{SE}(3) \times \mathbb{R}^6$** (`ErrorStateKalmanFilter3D`)

---

## 1. Planar CTRV Extended Kalman Filter

### State Vector
The state vector is defined in the vehicle navigation plane:
$$x = \begin{bmatrix} p_x & p_y & \theta & v & \omega \end{bmatrix}^T \in \mathbb{R}^2 \times S^1 \times \mathbb{R}^2$$

Where:
- $p_x, p_y$: Global Cartesian position (meters)
- $\theta$: Heading / yaw angle normalized to $[-\pi, \pi)$ (radians)
- $v$: Longitudinal linear velocity (m/s)
- $\omega$: Yaw rate (rad/s)

### CTRV Motion Model Integration
For non-zero yaw rate ($|\omega| > 10^{-5}\,\text{rad/s}$), circular-arc integration is used:

$$\begin{aligned}
p_x(t + \Delta t) &= p_x(t) + \frac{v}{\omega} \Big(\sin(\theta + \omega \Delta t) - \sin(\theta)\Big) \\
p_y(t + \Delta t) &= p_y(t) + \frac{v}{\omega} \Big(-\cos(\theta + \omega \Delta t) + \cos(\theta)\Big) \\
\theta(t + \Delta t) &= \text{wrap}\Big(\theta(t) + \omega \Delta t\Big) \\
v(t + \Delta t) &= v(t) \\
\omega(t + \Delta t) &= \omega(t)
\end{aligned}$$

For $|\omega| \le 10^{-5}\,\text{rad/s}$, a second-order midpoint Taylor expansion prevents division by zero:
$$\begin{aligned}
\theta_{\text{mid}} &= \theta + \frac{1}{2} \omega \Delta t \\
p_x(t + \Delta t) &= p_x(t) + v \cos(\theta_{\text{mid}}) \Delta t \\
p_y(t + \Delta t) &= p_y(t) + v \sin(\theta_{\text{mid}}) \Delta t
\end{aligned}$$

### Continuous-to-Discrete Process Noise Projection
Process noise accounts for longitudinal acceleration noise $\sigma_a^2$ and angular acceleration noise $\sigma_\alpha^2$:
$$Q(\theta, \omega, \Delta t) = \Gamma \, Q_u \, \Gamma^T + Q_{\text{base}} \frac{\Delta t}{\Delta t_{\text{ref}}}$$

$$\Gamma = \begin{bmatrix}
\frac{1}{2} \Delta t^2 \cos(\theta_{\text{mid}}) & 0 \\
\frac{1}{2} \Delta t^2 \sin(\theta_{\text{mid}}) & 0 \\
0 & \frac{1}{2} \Delta t^2 \\
\Delta t & 0 \\
0 & \Delta t
\end{bmatrix}, \quad
Q_u = \begin{bmatrix} \sigma_a^2 & 0 \\ 0 & \sigma_\alpha^2 \end{bmatrix}$$

### Joseph Stabilized Measurement Update
To guarantee that the updated error covariance remains symmetric positive semi-definite ($P = P^T \ge 0$):
$$P_{k|k} = (I - K_k H_k) P_{k|k-1} (I - K_k H_k)^T + K_k R_k K_k^T$$

---

## 2. 15-DOF Error-State Kalman Filter on $\text{SE}(3) \times \mathbb{R}^6$

### Nominal and Error States
- **Nominal State (16-D):**
  $$x_{\text{nom}} = \begin{bmatrix} p_{3\times 1} & v_{3\times 1} & q_{4\times 1} & b_{a, 3\times 1} & b_{\omega, 3\times 1} \end{bmatrix}^T$$
  Where $q$ is a unit quaternion representing attitude on $\text{SO}(3)$.
- **Error State (15-DOF):**
  $$\delta x = \begin{bmatrix} \delta p & \delta v & \delta \theta & \delta b_a & \delta b_\omega \end{bmatrix}^T \in \mathbb{R}^{15}$$
  Where $\delta \theta \in \mathfrak{so}(3)$ is the rotation vector in the Lie algebra tangent space.

### Strapdown Inertial Mechanization
Given accelerometer measurement $a_m$ and gyroscope measurement $\omega_m$:
$$a_b = a_m - b_a, \quad \omega_b = \omega_m - b_\omega$$
$$a_g = R(q) a_b + g_{\text{ENU}}$$
$$\dot{p} = v, \quad \dot{v} = a_g, \quad \dot{q} = \frac{1}{2} q \otimes \begin{bmatrix} 0 \\ \omega_b \end{bmatrix}$$

### Manifold State Injection & Error Reset
When a measurement update produces error state $\delta x$:
$$\begin{aligned}
p &\leftarrow p + \delta p \\
v &\leftarrow v + \delta v \\
q &\leftarrow q \otimes \text{Exp}(\delta \theta) \\
b_a &\leftarrow b_a + \delta b_a \\
b_\omega &\leftarrow b_\omega + \delta b_\omega
\end{aligned}$$
$$\delta x \leftarrow 0$$
