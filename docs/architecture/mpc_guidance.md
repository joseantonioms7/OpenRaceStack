# Path Guidance & Kinematic Control

ApexPathAI employs a Frenet-frame kinematic guidance system with optimal raceline curvature feedforward and predictive horizon rollout.

---

## 1. Frenet Tracking Error Kinematics

Given vehicle Cartesian position $(x, y)$ and heading $\theta$, the vehicle is orthogonally projected onto the nearest racetrack centerline or raceline waypoint $(x_{\text{ref}}, y_{\text{ref}})$ with tangent orientation $\psi_{\text{ref}}$:

$$\Delta x = x - x_{\text{ref}}, \quad \Delta y = y - y_{\text{ref}}$$

### Cross-Track Error ($e_y$)
$$e_y = -\sin(\psi_{\text{ref}}) \Delta x + \cos(\psi_{\text{ref}}) \Delta y$$
- $e_y > 0$: Vehicle is to the left of the reference raceline.
- $e_y < 0$: Vehicle is to the right of the reference raceline.

### Heading Orientation Error ($e_\psi$)
$$e_\psi = \text{wrap}(\theta - \psi_{\text{ref}})$$

---

## 2. Steering Command Synthesis

### Curvature Feedforward ($\delta_{\text{ff}}$)
To negotiate a turn of instantaneous reference curvature $\kappa_{\text{ref}}$ with vehicle wheelbase $L$:
$$\delta_{\text{ff}} = \arctan(L \cdot \kappa_{\text{ref}})$$

### Feedback Correction ($\delta_{\text{fb}}$)
Cross-track error is nullified using Frenet guidance:
$$\psi_{\text{desired}} = -\arctan\left(\frac{k_{\text{guidance}} \cdot e_y}{\max(v, 1.0)}\right)$$
$$e_{\psi, \text{effective}} = \text{wrap}(e_\psi - \psi_{\text{desired}})$$
$$\delta_{\text{fb}} = -k_p \cdot e_y - k_d \cdot e_{\psi, \text{effective}}$$

### Slew Rate & Saturation Envelopes
$$\delta_{\text{raw}} = \delta_{\text{ff}} + \delta_{\text{fb}}$$
$$\Delta \delta = \text{clamp}(\delta_{\text{raw}} - \delta_{\text{prev}}, -\dot{\delta}_{\max} \Delta t, \dot{\delta}_{\max} \Delta t)$$
$$\delta_{\text{cmd}} = \text{clamp}(\delta_{\text{prev}} + \Delta \delta, -\delta_{\max}, \delta_{\max})$$

---

## 3. Horizon Preview Rollout

Over the prediction horizon $N_p$ (15 steps at $\Delta t = 0.05\,\text{s}$):
$$\begin{aligned}
p_x(k+1) &= p_x(k) + v(k) \cos(\theta(k)) \Delta t \\
p_y(k+1) &= p_y(k) + v(k) \sin(\theta(k)) \Delta t \\
\theta(k+1) &= \text{wrap}\left(\theta(k) + \frac{v(k)}{L} \tan(\delta_{\text{cmd}}) \Delta t\right) \\
v(k+1) &= \max(0.0, v(k) + a_{\text{cmd}} \Delta t)
\end{aligned}$$
This forward projection provides trajectory preview vectors for collision checking and telemetry visualization.
