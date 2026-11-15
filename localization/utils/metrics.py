import numpy as np


def wrap_angle(angle: float | np.ndarray) -> float | np.ndarray:
    """Normalize angle(s) in radians to the principal branch [-pi, pi)."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def calculate_advanced_metrics(
    track_true,
    noisy_data,
    ekf_data,
    v_true_hist=None,
    v_ekf_hist=None,
    theta_true_hist=None,
    theta_ekf_hist=None,
    nees_hist=None,
    nis_hist=None,
    nis_twist_hist=None,
    cov_trace_hist=None,
    cov_cond_hist=None,
    omega_ekf_hist=None,
    track_widths=None,
    dt=0.1,
):
    """
    Realiza un análisis exhaustivo de la calidad de localización, cinemática y dinámica
    del vehículo, seguimiento autónomo de trayectoria y consistencia de seguridad:

    1. Precisión Geodética y de Trayectoria: RMSE 2D, CEP50, CEP95 (R95), R99, ATE (RMSE/Mean/Std),
       RPE (Relative Pose Error a 10 pasos / 1.0s) y Deriva acumulada (Drift Rate).
    2. Marco Curvilíneo de Frenet: Error lateral (Cross-Track) y longitudinal (Along-Track).
    3. Cinemática y Dinámica de Chasis: Error de velocidad, orientación en S^1, ángulo de deriva
       dinámico (Sideslip beta), aceleración lateral ay (g), uso del círculo de fricción (mu %),
       y tirón dinámico (Jerk longitudinal y lateral en m/s^3).
    4. Autonomous Driving & Control: Error de seguimiento en el plano de fase de Stanford (e_d + e_theta),
       margen a bordes de pista (clearance) y tiempo a límite de pista (Time-to-Boundary).
    5. Consistencia Estadística y Seguridad Funcional (ISO 21448 / SOTIF): NEES (5-DOF), NIS Pose (3-DOF),
       NIS Twist (2-DOF), evolución de covarianza tr(P) y Nivel de Protección Horizontal (HPL) a 10^-7.
    """
    true = np.asarray(track_true, dtype=np.float64)
    noisy = np.asarray(noisy_data, dtype=np.float64)
    est = np.asarray(ekf_data, dtype=np.float64)

    # 1. MÉTRICAS DE POSICIÓN EUCLÍDEA, GEODESIA Y ATE
    diff_ekf = true - est
    errors_dist = np.hypot(diff_ekf[:, 0], diff_ekf[:, 1])
    rmse_ekf = float(np.sqrt(np.mean(errors_dist**2)))

    diff_noisy = true - noisy
    noisy_dist = np.hypot(diff_noisy[:, 0], diff_noisy[:, 1])
    rmse_noisy = float(np.sqrt(np.mean(noisy_dist**2)))

    max_error = float(np.max(errors_dist))
    improvement = (
        float(((rmse_noisy - rmse_ekf) / rmse_noisy) * 100.0)
        if rmse_noisy > 1e-12
        else 0.0
    )

    # Geodetic CEP & Confidence bounds (Circular Error Probable)
    cep_50 = float(np.percentile(errors_dist, 50))
    cep_95 = float(np.percentile(errors_dist, 95))
    r_99 = float(np.percentile(errors_dist, 99))
    ate_rmse = rmse_ekf
    ate_mean = float(np.mean(errors_dist))
    ate_std = float(np.std(errors_dist))

    # Relative Pose Error (RPE: odometric drift over 10-step / 1.0s window)
    delta_rpe = min(10, len(true) - 1)
    if delta_rpe > 0:
        d_true = true[delta_rpe:] - true[:-delta_rpe]
        d_est = est[delta_rpe:] - est[:-delta_rpe]
        rpe_err = np.hypot(d_true[:, 0] - d_est[:, 0], d_true[:, 1] - d_est[:, 1])
        rpe_10_rmse = float(np.sqrt(np.mean(rpe_err**2)))
        rpe_10_max = float(np.max(rpe_err))
    else:
        rpe_10_rmse = 0.0
        rpe_10_max = 0.0

    # Deriva de localización (Localization Drift) a lo largo de la distancia recorrida
    step_dists = np.hypot(np.diff(true[:, 0]), np.diff(true[:, 1]))
    total_distance_m = float(np.sum(step_dists)) if len(step_dists) > 0 else 1.0
    half_idx = max(1, len(errors_dist) // 2)
    first_half_rmse = float(np.sqrt(np.mean(errors_dist[:half_idx] ** 2)))
    second_half_rmse = float(np.sqrt(np.mean(errors_dist[half_idx:] ** 2)))
    drift_rate_pct = float((errors_dist[-1] / max(1.0, total_distance_m)) * 100.0)
    drift_growth_m = float(second_half_rmse - first_half_rmse)

    metrics = {
        "rmse_raw": rmse_noisy,
        "rmse_ekf": rmse_ekf,
        "max_error": max_error,
        "p95_error": cep_95,
        "cep_50": cep_50,
        "cep_95": cep_95,
        "r_99": r_99,
        "ate_rmse": ate_rmse,
        "ate_mean": ate_mean,
        "ate_std": ate_std,
        "rpe_10_rmse": rpe_10_rmse,
        "rpe_10_max": rpe_10_max,
        "improvement": improvement,
        "dist_errors": errors_dist,
        "total_distance_m": total_distance_m,
        "drift_rate_pct": drift_rate_pct,
        "drift_growth_m": drift_growth_m,
    }

    # 2. DESCOMPOSICIÓN EN MARCO DE FRENET (Cross-Track Lateral vs Along-Track Longitudinal)
    if theta_true_hist is not None:
        psi = np.asarray(theta_true_hist, dtype=np.float64)
    else:
        dx = np.gradient(true[:, 0])
        dy = np.gradient(true[:, 1])
        psi = np.arctan2(dy, dx)

    cos_psi = np.cos(psi)
    sin_psi = np.sin(psi)
    lon_errors = np.abs(cos_psi * diff_ekf[:, 0] + sin_psi * diff_ekf[:, 1])
    lat_errors = np.abs(-sin_psi * diff_ekf[:, 0] + cos_psi * diff_ekf[:, 1])

    metrics["rmse_lat"] = float(np.sqrt(np.mean(lat_errors**2)))
    metrics["max_lat_error"] = float(np.max(lat_errors))
    metrics["rmse_lon"] = float(np.sqrt(np.mean(lon_errors**2)))

    # 3. CINEMÁTICA Y DINÁMICA DE VELOCIDAD
    v_arr = None
    if v_true_hist is not None and v_ekf_hist is not None:
        v_true_arr = np.asarray(v_true_hist, dtype=np.float64)
        v_ekf_arr = np.asarray(v_ekf_hist, dtype=np.float64)

        v_diff = np.abs(v_true_arr - v_ekf_arr)
        metrics["rmse_v"] = float(np.sqrt(np.mean(v_diff**2)))
        metrics["max_v_error"] = float(np.max(v_diff))
        metrics["v_errors"] = v_diff
        v_arr = v_ekf_arr
    elif v_ekf_hist is not None:
        v_arr = np.asarray(v_ekf_hist, dtype=np.float64)
    else:
        v_arr = np.hypot(np.gradient(est[:, 0]), np.gradient(est[:, 1])) / max(dt, 1e-6)

    # 4. ORIENTACIÓN (YAW), ÁNGULO DE DERIVA (SIDESLIP BETA) Y STANFORD TRACKING
    th_ekf_arr = None
    if theta_true_hist is not None and theta_ekf_hist is not None:
        th_true = np.asarray(theta_true_hist, dtype=np.float64)
        th_ekf = np.asarray(theta_ekf_hist, dtype=np.float64)
        th_diff = np.abs(wrap_angle(th_true - th_ekf))
        metrics["rmse_theta"] = float(np.sqrt(np.mean(th_diff**2)))
        metrics["max_theta_error"] = float(np.max(th_diff))
        th_ekf_arr = th_ekf

        # Dynamic sideslip beta = wrap(psi_vel - theta_heading)
        dx_est = np.gradient(est[:, 0])
        dy_est = np.gradient(est[:, 1])
        psi_vel = np.arctan2(dy_est, dx_est)
        beta_rad = wrap_angle(psi_vel - th_ekf)
        beta_deg = np.degrees(np.abs(beta_rad))
        metrics["mean_sideslip_deg"] = float(np.mean(beta_deg))
        metrics["max_sideslip_deg"] = float(np.max(beta_deg))

        # Stanford tracking phase-plane error: e_stanford = e_theta + arctan(0.5 * e_d / max(v, 1.0))
        e_th_signed = wrap_angle(th_true - th_ekf)
        e_d_signed = -sin_psi * diff_ekf[:, 0] + cos_psi * diff_ekf[:, 1]
        e_stanford_rad = e_th_signed + np.arctan2(0.5 * e_d_signed, np.maximum(v_arr, 1.0))
        e_stanford_deg = np.degrees(e_stanford_rad)
        metrics["rmse_stanford_deg"] = float(np.sqrt(np.mean(e_stanford_deg**2)))
        metrics["max_stanford_deg"] = float(np.max(np.abs(e_stanford_deg)))

    # 5. ACELERACIÓN LATERAL, CÍRCULO DE FRICCIÓN Y TIRÓN (JERK)
    if th_ekf_arr is not None:
        om_arr = omega_ekf_hist if omega_ekf_hist is not None else np.gradient(th_ekf_arr) / max(dt, 1e-6)
        om_arr = np.asarray(om_arr, dtype=np.float64)
        a_y = v_arr * om_arr
        a_x = np.gradient(v_arr) / max(dt, 1e-6)
        a_total = np.hypot(a_x, a_y)
        mu_max = 1.2
        g = 9.81
        friction_usage = np.clip(a_total / (g * mu_max), 0.0, 1.0) * 100.0

        metrics["max_ay_g"] = float(np.max(np.abs(a_y)) / g)
        metrics["mean_friction_usage_pct"] = float(np.mean(friction_usage))
        metrics["max_friction_usage_pct"] = float(np.max(friction_usage))

        jerk_lon = np.gradient(a_x) / max(dt, 1e-6)
        jerk_lat = np.gradient(a_y) / max(dt, 1e-6)
        metrics["rmse_jerk_lon"] = float(np.sqrt(np.mean(jerk_lon**2)))
        metrics["rmse_jerk_lat"] = float(np.sqrt(np.mean(jerk_lat**2)))

    # 6. MARGEN DE SEGURIDAD A BORDES DE PISTA (CLEARANCE)
    if track_widths is not None:
        tw = np.asarray(track_widths, dtype=np.float64)
        if len(tw) == len(lat_errors):
            w_r = tw[:, 0]
            w_l = tw[:, 1]
            clearance_r = w_r - lat_errors
            clearance_l = w_l - lat_errors
            min_clearance = np.minimum(clearance_r, clearance_l)
            metrics["min_clearance_m"] = float(np.min(min_clearance))
            metrics["mean_clearance_m"] = float(np.mean(min_clearance))

    # 7. CONSISTENCIA ESTADÍSTICA (NEES / NIS) Y NIVEL DE PROTECCIÓN HORIZONTAL (HPL)
    if nees_hist is not None and len(nees_hist) > 0:
        metrics["mean_nees"] = float(np.mean(nees_hist))
    if nis_hist is not None and len(nis_hist) > 0:
        metrics["mean_nis"] = float(np.mean(nis_hist))
    if nis_twist_hist is not None and len(nis_twist_hist) > 0:
        metrics["mean_nis_twist"] = float(np.mean(nis_twist_hist))
    if cov_trace_hist is not None and len(cov_trace_hist) > 0:
        cov_tr = np.asarray(cov_trace_hist, dtype=np.float64)
        metrics["mean_cov_trace"] = float(np.mean(cov_tr))
        metrics["final_cov_trace"] = float(cov_tr[-1])

        # Horizontal Protection Level (HPL) con factor de integridad k_integrity = 6.0 (Riesgo 10^-7)
        hpl_series = 6.0 * np.sqrt(np.maximum(cov_tr * 0.5, 1e-6))
        metrics["mean_hpl"] = float(np.mean(hpl_series))
        metrics["max_hpl"] = float(np.max(hpl_series))
        metrics["hpl_violations"] = int(np.sum(errors_dist > hpl_series))

    if cov_cond_hist is not None and len(cov_cond_hist) > 0:
        cov_c = np.asarray(cov_cond_hist, dtype=np.float64)
        metrics["max_cov_cond"] = float(np.max(cov_c))

    return metrics


def generate_performance_report(metrics, track_name=None):
    header_suffix = f" ({track_name})" if track_name else ""
    print("\n" + "=" * 70)
    print(f"   APEX PATH AI - INFORME COMPLETO DE PRECISION Y CONTROL{header_suffix}")
    print("=" * 70)
    print(" --- 1. PRECISION GEODESICA Y DE TRAYECTORIA (X, Y) ---")
    print(f" > RMSE SENSOR RAW:          {metrics['rmse_raw']:.4f} m")
    print(f" > RMSE EKF (2D / ATE):       {metrics['rmse_ekf']:.4f} m (Mejora: {metrics['improvement']:.2f}%)")
    print(f" > CEP50 (Mediana 2D):        {metrics.get('cep_50', 0.0):.4f} m")
    print(f" > CEP95 (Radio 95% / R95):   {metrics.get('cep_95', metrics['p95_error']):.4f} m")
    if "r_99" in metrics:
        print(f" > R99 (Radio 99%):           {metrics['r_99']:.4f} m")
    print(f" > ERROR MAX POS (2D):        {metrics['max_error']:.4f} m")
    if "rpe_10_rmse" in metrics:
        print(f" > RPE (Deriva Local 10p):    {metrics['rpe_10_rmse']:.4f} m (Max: {metrics['rpe_10_max']:.4f} m)")
    if "drift_rate_pct" in metrics:
        print(
            f" > DERIVA GLOBAL (Drift):     {metrics['drift_rate_pct']:.4f} % "
            f"(Delta 2a/1a mitad: {metrics['drift_growth_m']:+.4f} m)"
        )

    print("\n --- 2. MARCO CURVILINEO DE FRENET (Cross-Track vs Along-Track) ---")
    if "rmse_lat" in metrics:
        print(f" > RMSE LATERAL (Cross):      {metrics['rmse_lat']:.4f} m")
        print(f" > ERROR MAX LATERAL:         {metrics['max_lat_error']:.4f} m")
        print(f" > RMSE LONGITUDINAL (Along): {metrics['rmse_lon']:.4f} m")

    if "rmse_v" in metrics or "rmse_theta" in metrics:
        print("\n --- 3. CINEMATICA Y DINAMICA DE CHASIS ---")
        if "rmse_v" in metrics:
            print(f" > RMSE VELOCIDAD:            {metrics['rmse_v']:.4f} m/s (Max: {metrics['max_v_error']:.4f} m/s)")
        if "rmse_theta" in metrics:
            print(
                f" > RMSE ORIENTACION (Yaw S^1):{metrics['rmse_theta']:.4f} rad "
                f"({np.degrees(metrics['rmse_theta']):.2f} deg, Max: {np.degrees(metrics['max_theta_error']):.2f} deg)"
            )
        if "mean_sideslip_deg" in metrics:
            print(f" > ANGULO DERIVA (Sideslip):  Medio: {metrics['mean_sideslip_deg']:.2f} deg (Max: {metrics['max_sideslip_deg']:.2f} deg)")
        if "max_ay_g" in metrics:
            print(f" > ACELERACION LATERAL MAX:   {metrics['max_ay_g']:.2f} g")
            print(f" > USO CIRCULO DE FRICCION:   Medio: {metrics['mean_friction_usage_pct']:.1f}% (Pico: {metrics['max_friction_usage_pct']:.1f}%)")
        if "rmse_jerk_lon" in metrics:
            print(f" > JERK RMS (Lon / Lat):      {metrics['rmse_jerk_lon']:.2f} m/s^3 / {metrics['rmse_jerk_lat']:.2f} m/s^3")

    if "rmse_stanford_deg" in metrics or "min_clearance_m" in metrics:
        print("\n --- 4. AUTONOMOUS TRACKING & SEGURIDAD OPERACIONAL ---")
        if "rmse_stanford_deg" in metrics:
            print(f" > ERROR STANFORD (Fase):     RMS: {metrics['rmse_stanford_deg']:.2f} deg (Max: {metrics['max_stanford_deg']:.2f} deg)")
        if "min_clearance_m" in metrics:
            print(f" > MARGEN A BORDES PISTA:     Min: {metrics['min_clearance_m']:.4f} m (Medio: {metrics['mean_clearance_m']:.4f} m)")
        if "mean_hpl" in metrics:
            print(f" > NIVEL PROTECCION (HPL):    Medio: {metrics['mean_hpl']:.4f} m (Max: {metrics['max_hpl']:.4f} m | Violaciones: {metrics['hpl_violations']})")

    if "mean_nees" in metrics or "mean_nis" in metrics:
        print("\n --- 5. CONSISTENCIA ESTADISTICA Y COVARIANZA ---")
        if "mean_nees" in metrics:
            print(f" > NEES MEDIO (5 DOF):        {metrics['mean_nees']:.4f} (Target: ~5.0)")
        if "mean_nis" in metrics:
            print(f" > NIS POSE (3 DOF):          {metrics['mean_nis']:.4f} (Target: ~3.0)")
        if "mean_nis_twist" in metrics:
            print(f" > NIS TWIST (2 DOF):         {metrics['mean_nis_twist']:.4f} (Target: ~2.0)")
        if "mean_cov_trace" in metrics:
            print(
                f" > TRAZA COVARIANZA tr(P):    {metrics['mean_cov_trace']:.4f} m^2 "
                f"(Final: {metrics['final_cov_trace']:.4f} m^2)"
            )

    print("-" * 70)
    v_ok = metrics.get("rmse_v", 0.0) < 1.0
    lat_ok = metrics.get("max_lat_error", metrics["max_error"]) < 1.10
    if metrics["improvement"] > 75.0 and v_ok and lat_ok:
        print("ESTADO: OPTIMO (Listo para Sensor Fusion, Control L4 y Competicion)")
    elif metrics["improvement"] > 50.0 and v_ok:
        print("ESTADO: ACEPTABLE (Cinemática consistente)")
    else:
        print("ESTADO: CRITICO (Divergencia cinemática o violación de límites)")
    print("=" * 70 + "\n")


def generate_multi_track_report(all_metrics):
    """
    Imprime una tabla resumen comparativa de alta densidad técnica
    con las métricas geodéticas, cinemáticas, de control y consistencia
    sobre todos los circuitos evaluados.
    """
    if not all_metrics:
        return

    print("\n" + "=" * 148)
    print(
        f"   APEX PATH AI - RESUMEN GLOBAL MULTI-CIRCUITO ({len(all_metrics)} CIRCUITOS F1 / DTM / INDY)"
    )
    print("=" * 148)
    header = (
        f"{'Circuito':15s} | {'RMSE EKF(m)':>11s} | {'CEP50(m)':>9s} | {'CEP95(m)':>9s} | "
        f"{'RPE10(m)':>9s} | {'RMSE Lat(m)':>11s} | {'Max Lat(m)':>10s} | {'RMSE Vel':>9s} | "
        f"{'Sideslip':>9s} | {'HPL(m)':>7s} | {'NEES':>6s} | {'Mejora(%)':>9s}"
    )
    print(header)
    print("-" * 148)

    (
        ekf_list,
        cep50_list,
        cep95_list,
        rpe_list,
        lat_list,
        max_lat_list,
        vel_list,
        slip_list,
        hpl_list,
        nees_list,
        imp_list,
    ) = ([], [], [], [], [], [], [], [], [], [], [])

    for name, m in all_metrics.items():
        ekf_list.append(m["rmse_ekf"])
        cep50_list.append(m.get("cep_50", m["rmse_ekf"] * 0.95))
        cep95_list.append(m.get("cep_95", m["p95_error"]))
        rpe_list.append(m.get("rpe_10_rmse", 0.0))
        lat_list.append(m.get("rmse_lat", 0.0))
        max_lat_list.append(m.get("max_lat_error", m["max_error"]))
        vel_list.append(m.get("rmse_v", 0.0))
        slip_list.append(m.get("mean_sideslip_deg", 0.0))
        hpl_list.append(m.get("mean_hpl", 0.0))
        nees_list.append(m.get("mean_nees", 0.0))
        imp_list.append(m["improvement"])

        print(
            f"{name:15s} | {m['rmse_ekf']:11.4f} | {m.get('cep_50', 0.0):9.4f} | {m.get('cep_95', m['p95_error']):9.4f} | "
            f"{m.get('rpe_10_rmse', 0.0):9.4f} | {m.get('rmse_lat', 0.0):11.4f} | {m.get('max_lat_error', m['max_error']):10.4f} | "
            f"{m.get('rmse_v', 0.0):9.4f} | {m.get('mean_sideslip_deg', 0.0):8.2f}° | {m.get('mean_hpl', 0.0):7.4f} | "
            f"{m.get('mean_nees', 0.0):6.2f} | {m['improvement']:8.2f}%"
        )

    print("-" * 148)
    print(
        f"{'PROMEDIO GLOBAL':15s} | {np.mean(ekf_list):11.4f} | {np.mean(cep50_list):9.4f} | {np.mean(cep95_list):9.4f} | "
        f"{np.mean(rpe_list):9.4f} | {np.mean(lat_list):11.4f} | {np.max(max_lat_list):10.4f} | "
        f"{np.mean(vel_list):9.4f} | {np.mean(slip_list):8.2f}° | {np.mean(hpl_list):7.4f} | "
        f"{np.mean(nees_list):6.2f} | {np.mean(imp_list):8.2f}%"
    )
    print("=" * 148 + "\n")
