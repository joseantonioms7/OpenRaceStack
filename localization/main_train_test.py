import matplotlib.pyplot as plt
import numpy as np

try:
    from Localization.core.ekf_engine import LocalizationEKF, wrap_angle
    from Localization.utils.metrics import (
        calculate_advanced_metrics,
        generate_multi_track_report,
        generate_performance_report,
    )
    from Localization.utils.track_loader import TrackLoader
except ImportError:
    from core.ekf_engine import LocalizationEKF, wrap_angle
    from utils.metrics import (
        calculate_advanced_metrics,
        generate_multi_track_report,
        generate_performance_report,
    )
    from utils.track_loader import TrackLoader


def _plot_track_results(track_true, noisy_measurements, ekf_estimates, title="Test EKF en Circuito F1"):
    plt.figure(figsize=(12, 8))
    plt.plot(track_true[:, 0], track_true[:, 1], 'g-', linewidth=2, label='Trayectoria Real (F1 Track)')
    plt.scatter(noisy_measurements[:, 0], noisy_measurements[:, 1], c='red', s=10, alpha=0.3, label='Odometría/GPS Ruidoso')
    plt.plot(ekf_estimates[:, 0], ekf_estimates[:, 1], 'b-', linewidth=2, label='Estimación LocalizationEKF')
    plt.title(title)
    plt.xlabel("Coordenada X (m)")
    plt.ylabel("Coordenada Y (m)")
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()


def simulate_single_circuit(ruta_dataset, show_plots=False, seed=42, verbose=True):
    """
    Ejecuta la simulación y filtrado EKF sobre un circuito individual.
    """
    rng = np.random.default_rng(seed)

    loader = TrackLoader(ruta_dataset)
    track = loader.load_track(verbose=verbose)

    if track is None:
        raise RuntimeError(f"No se pudo cargar el circuito desde '{ruta_dataset}'.")

    track_name = loader.track_name or ruta_dataset
    n_points = len(track)
    n_steps = n_points - 1
    dt = 0.1

    # Pre-calcular cinemática real CTRV en lazo cerrado sin desfase de un paso
    diffs = np.diff(track, axis=0)
    dx_all = diffs[:, 0]
    dy_all = diffs[:, 1]
    chord_len = np.hypot(dx_all, dy_all)
    psi_chord = np.arctan2(dy_all, dx_all)

    # Tangentes continuas en cada vértice (con cierre periódico en la costura del circuito)
    psi_seam = float(np.arctan2(track[0, 1] - track[-1, 1], track[0, 0] - track[-1, 0]))
    psi_in = np.empty(n_points, dtype=np.float64)
    psi_out = np.empty(n_points, dtype=np.float64)
    psi_in[0] = psi_seam
    psi_in[1:] = psi_chord
    psi_out[:-1] = psi_chord
    psi_out[-1] = psi_seam
    theta_wp = wrap_angle(psi_in + 0.5 * wrap_angle(psi_out - psi_in))

    theta_true_all = theta_wp[1:]
    omega_step = wrap_angle(np.diff(theta_wp)) / dt
    half_dtheta = 0.5 * omega_step * dt
    sinc_half = np.where(
        np.abs(half_dtheta) > 1e-6, np.sin(half_dtheta) / half_dtheta, 1.0
    )
    v_step = (chord_len / dt) / sinc_half

    # Alinear velocidad y tasa de guiñada medidas en t_{i+1} con la transición siguiente
    v_true_all = np.empty(n_steps, dtype=np.float64)
    v_true_all[:-1] = v_step[1:]
    v_true_all[-1] = v_step[0]
    omega_true_all = np.empty(n_steps, dtype=np.float64)
    omega_true_all[:-1] = omega_step[1:]
    omega_true_all[-1] = omega_step[0]

    # Estado inicial físicamente consistente [x, y, theta, v, omega] en t_0
    x0 = np.array(
        [
            [track[0, 0]],
            [track[0, 1]],
            [float(theta_wp[0])],
            [float(v_step[0])],
            [float(omega_step[0])],
        ],
        dtype=np.float64,
    )
    P0 = np.diag([0.1, 0.1, 0.01, 0.05, 0.05])

    sigma_gps = 1.5
    sigma_heading = 0.05
    sigma_v_odom = 0.08
    sigma_omega_imu = 0.02

    # Q calibrado con proyección de punto medio para consistencia 5-DOF en los 23 circuitos
    Q_diag = [0.00015, 0.00015, 0.00015, 0.0015, 0.28]
    R_gps_diag = [sigma_gps**2, sigma_gps**2]
    R_pose = np.diag([sigma_gps**2, sigma_gps**2, sigma_heading**2])
    R_twist = np.diag([sigma_v_odom**2, sigma_omega_imu**2])

    ekf = LocalizationEKF(
        dt=dt,
        Q_diag=Q_diag,
        R_diag=R_gps_diag,
        sigma_a=0.14,
        sigma_alpha=1.45,
        use_dynamic_Q=True,
        gate_threshold=16.266,  # Umbral chi-cuadrado 99.9% para 3 DOF
    )
    ekf.initialize_state(x0, P0=P0, timestamp=0.0)

    # Pre-alojar buffers contiguos de historial
    true_history = np.ascontiguousarray(track[1:, :2], dtype=np.float64)
    gps_noise = rng.normal(0.0, sigma_gps, size=(n_steps, 2))
    noisy_history = true_history + gps_noise
    heading_noise = rng.normal(0.0, sigma_heading, size=n_steps)
    twist_noise = np.column_stack(
        (
            rng.normal(0.0, sigma_v_odom, size=n_steps),
            rng.normal(0.0, sigma_omega_imu, size=n_steps),
        )
    )

    ekf_history = np.empty((n_steps, 2), dtype=np.float64)
    v_ekf_history = np.empty(n_steps, dtype=np.float64)
    theta_ekf_history = np.empty(n_steps, dtype=np.float64)
    omega_ekf_history = np.empty(n_steps, dtype=np.float64)
    nees_history = np.empty(n_steps, dtype=np.float64)
    nis_history = np.empty(n_steps, dtype=np.float64)
    nis_twist_history = np.empty(n_steps, dtype=np.float64)
    cov_trace_history = np.empty(n_steps, dtype=np.float64)

    z_pose_buf = np.empty((3, 1), dtype=np.float64)
    z_twist_buf = np.empty((2, 1), dtype=np.float64)
    err_x_buf = np.empty((5, 1), dtype=np.float64)

    has_widths = loader.track_widths is not None
    if verbose:
        print(f"Simulando vuelta en {track_name} y estimando estados ocultos...")

    for i in range(n_steps):
        # Paso 1: Predicción cinemática CTRV exacta
        ekf.predict(dt)

        # Paso 2: Fusión propioceptiva (Odometría de rueda + IMU yaw rate)
        z_twist_buf[0, 0] = v_true_all[i] + twist_noise[i, 0]
        z_twist_buf[1, 0] = omega_true_all[i] + twist_noise[i, 1]
        ekf.update_imu_odom(z_twist_buf, R_prop=R_twist, gate_threshold=25.0)
        nis_twist_history[i] = ekf.channel_nis.get("imu_odom", ekf.last_nis)

        # Paso 3: Fusión exteroceptiva (Posición GPS + Rumbo con compuerta Mahalanobis chi2)
        z_pose_buf[0, 0] = noisy_history[i, 0]
        z_pose_buf[1, 0] = noisy_history[i, 1]
        z_pose_buf[2, 0] = float(wrap_angle(theta_true_all[i] + heading_noise[i]))
        ekf.update_pose(z_pose_buf, R_pose=R_pose, gate_threshold=16.266)
        nis_history[i] = ekf.channel_nis.get("pose", ekf.last_nis)

        # Paso 4: Restricción de corredor map-matched (sin fuga de ground-truth) y actualización Joseph
        if has_widths:
            ekf.constrain_to_corridor(track, w_right=loader.track_widths)

        # Guardar estimaciones en buffers pre-alojados
        ekf_history[i, 0] = ekf.x[0, 0]
        ekf_history[i, 1] = ekf.x[1, 0]
        theta_ekf_history[i] = ekf.x[2, 0]
        v_ekf_history[i] = ekf.x[3, 0]
        omega_ekf_history[i] = ekf.x[4, 0]
        cov_trace_history[i] = float(np.trace(ekf.P))

        # Calcular NEES (Normalized Estimation Error Squared) sobre el estado 5D
        err_x_buf[0, 0] = true_history[i, 0] - ekf.x[0, 0]
        err_x_buf[1, 0] = true_history[i, 1] - ekf.x[1, 0]
        err_x_buf[2, 0] = float(wrap_angle(theta_true_all[i] - ekf.x[2, 0]))
        err_x_buf[3, 0] = v_true_all[i] - ekf.x[3, 0]
        err_x_buf[4, 0] = omega_true_all[i] - ekf.x[4, 0]
        nees_history[i] = float((err_x_buf.T @ np.linalg.solve(ekf.P, err_x_buf))[0, 0])

    metrics = calculate_advanced_metrics(
        true_history,
        noisy_history,
        ekf_history,
        v_true_all,
        v_ekf_history,
        theta_true_hist=theta_true_all,
        theta_ekf_hist=theta_ekf_history,
        nees_hist=nees_history,
        nis_hist=nis_history,
        nis_twist_hist=nis_twist_history,
        cov_trace_hist=cov_trace_history,
        omega_ekf_hist=omega_ekf_history,
        track_widths=loader.track_widths,
        dt=dt,
    )

    if verbose:
        generate_performance_report(metrics, track_name=track_name)

    if show_plots:
        dist_errors = metrics["dist_errors"]
        v_errors = metrics["v_errors"]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

        ax1.plot(dist_errors, color="red", label="Error de Posición (m)")
        ax1.axhline(
            y=metrics["rmse_ekf"], color="blue", linestyle="--", label="RMSE Posición"
        )
        ax1.set_title(f"Diagnóstico del Filtro ({track_name}): Error de Posición")
        ax1.set_ylabel("Metros")
        ax1.legend()
        ax1.grid(True)

        ax2.plot(v_errors, color="orange", label="Error de Velocidad Estimada (m/s)")
        ax2.axhline(
            y=metrics["rmse_v"], color="purple", linestyle="--", label="RMSE Velocidad"
        )
        ax2.set_title(
            f"Diagnóstico del Filtro ({track_name}): Estimación de Estado Oculto (Velocidad)"
        )
        ax2.set_xlabel("Punto de la Trayectoria (Waypoint)")
        ax2.set_ylabel("Metros / Segundo")
        ax2.legend()
        ax2.grid(True)

        plt.tight_layout()
        plt.show()

        _plot_track_results(
            true_history,
            noisy_history,
            ekf_history,
            title=f"Test EKF en Circuito {track_name}",
        )

    return metrics


def simulate_and_filter(show_plots=True, seed=42, run_all_circuits=True):
    """
    Evalúa el pipeline de localización sobre todos los circuitos disponibles en
    el directorio datasets/ (o sobre un circuito específico si se desactiva run_all_circuits).
    """
    if not run_all_circuits:
        return simulate_single_circuit(
            "datasets/Monza/Monza_centerline.csv",
            show_plots=show_plots,
            seed=seed,
            verbose=True,
        )

    tracks = TrackLoader.discover_all_tracks("datasets")
    if not tracks:
        raise RuntimeError("No se encontraron circuitos en el directorio 'datasets/'.")

    all_metrics = {}
    for idx, (track_name, track_path) in enumerate(tracks):
        # Mostrar gráficas interactivas para el circuito de referencia si show_plots=True
        plot_this = show_plots and (track_name == "Monza" or idx == 0 and len(tracks) == 1)
        m = simulate_single_circuit(
            track_path,
            show_plots=plot_this,
            seed=seed,
            verbose=False,
        )
        all_metrics[track_name] = m

    generate_multi_track_report(all_metrics)

    if show_plots and len(all_metrics) > 1:
        names = list(all_metrics.keys())
        rmse_ekf_vals = [all_metrics[n]["rmse_ekf"] for n in names]
        rmse_lat_vals = [all_metrics[n]["rmse_lat"] for n in names]
        rmse_v_vals = [all_metrics[n]["rmse_v"] for n in names]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
        x_idx = np.arange(len(names))
        width = 0.38

        ax1.bar(x_idx - width / 2, rmse_ekf_vals, width, label="RMSE 2D (m)", color="royalblue")
        ax1.bar(
            x_idx + width / 2, rmse_lat_vals, width, label="RMSE Lateral Frenet (m)", color="seagreen"
        )
        ax1.set_title("Precisión de Localización EKF en los 23 Circuitos (Posición)")
        ax1.set_ylabel("RMSE (m)")
        ax1.legend()
        ax1.grid(True, axis="y", alpha=0.4)

        ax2.bar(x_idx, rmse_v_vals, width=0.5, label="RMSE Velocidad (m/s)", color="darkorange")
        ax2.set_title("Precisión de Estimación de Estado Oculto (Velocidad) en los 23 Circuitos")
        ax2.set_ylabel("RMSE (m/s)")
        ax2.set_xticks(x_idx)
        ax2.set_xticklabels(names, rotation=45, ha="right")
        ax2.legend()
        ax2.grid(True, axis="y", alpha=0.4)

        plt.tight_layout()
        plt.show()

    return all_metrics


if __name__ == "__main__":
    simulate_and_filter()
