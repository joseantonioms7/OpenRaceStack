from pathlib import Path

import numpy as np
import pandas as pd


class TrackLoader:
    def __init__(self, filepath):
        self.filepath = filepath
        self.track_data = None
        self.track_widths = None
        self.raceline_data = None
        self.track_name = None

    @staticmethod
    def discover_all_tracks(datasets_dir="datasets", prefer_raceline=False):
        """
        Descubre automáticamente todos los circuitos disponibles en el directorio
        de datasets, devolviendo una lista ordenada de tuplas (nombre_circuito, ruta_csv).
        """
        base_path = Path(datasets_dir)
        if not base_path.is_dir():
            base_path = Path(__file__).resolve().parent.parent / datasets_dir
        if not base_path.is_dir():
            return []

        tracks = []
        for folder in sorted([d for d in base_path.iterdir() if d.is_dir()]):
            if prefer_raceline:
                candidates = list(folder.glob("*_raceline.csv")) or list(
                    folder.glob("*_centerline.csv")
                )
            else:
                candidates = list(folder.glob("*_centerline.csv")) or list(
                    folder.glob("*_raceline.csv")
                )
            if candidates:
                tracks.append((folder.name, str(candidates[0])))
        return tracks

    def _resolve_path(self) -> Path:
        path = Path(self.filepath)
        if path.is_file():
            return path
        # Fallback relativo al directorio raíz de Localization GPS
        base_dir = Path(__file__).resolve().parent.parent
        candidate = base_dir / self.filepath
        if candidate.is_file():
            return candidate
        return path

    def load_track(self, col_x='x_m', col_y='y_m', verbose=True):
        """
        Carga el circuito soportando tanto archivos centerline (separados por coma)
        como raceline (separados por punto y coma con metadatos en cabecera),
        devolviendo un array C-contiguo (N, 2) de coordenadas [x_m, y_m].
        """
        try:
            resolved_path = self._resolve_path()
            self.track_name = resolved_path.parent.name or resolved_path.stem

            # Detectar formato (raceline ';' vs centerline ',') leyendo las primeras líneas
            with open(resolved_path, encoding='utf-8') as f:
                head_lines = [f.readline() for _ in range(5)]

            header_line_idx = 0
            sep = ','
            for idx, line in enumerate(head_lines):
                if ';' in line and ('x_m' in line or 's_m' in line):
                    sep = ';'
                    header_line_idx = idx
                    break
                if ',' in line and 'x_m' in line:
                    sep = ','
                    header_line_idx = idx
                    break

            df = pd.read_csv(
                resolved_path,
                sep=sep,
                header=header_line_idx,
                skipinitialspace=True,
            )

            # Limpieza de cabeceras: quitamos '#', espacios y saltos de línea
            df.columns = df.columns.str.replace('#', '', regex=False).str.strip()

            if verbose:
                print(f"Columnas limpias detectadas: {list(df.columns)}")

            if col_x not in df.columns or col_y not in df.columns:
                if verbose:
                    print(
                        f"Aviso: No se encontraron '{col_x}' o '{col_y}'. "
                        "Usando las dos primeras columnas por defecto."
                    )
                col_x = df.columns[0]
                col_y = df.columns[1]

            x = df[col_x].to_numpy(dtype=np.float64)
            y = df[col_y].to_numpy(dtype=np.float64)

            # Sanitizar valores no finitos y waypoints consecutivos o de cierre duplicados
            finite_mask = np.isfinite(x) & np.isfinite(y)
            if 'w_tr_right_m' in df.columns and 'w_tr_left_m' in df.columns:
                w_r_raw = df['w_tr_right_m'].to_numpy(dtype=np.float64)
                w_l_raw = df['w_tr_left_m'].to_numpy(dtype=np.float64)
                finite_mask &= np.isfinite(w_r_raw) & np.isfinite(w_l_raw)
            else:
                w_r_raw = None
                w_l_raw = None

            df = df.loc[finite_mask].reset_index(drop=True)
            x = x[finite_mask]
            y = y[finite_mask]
            if w_r_raw is not None:
                w_r_raw = w_r_raw[finite_mask]
                w_l_raw = w_l_raw[finite_mask]

            if len(x) > 1:
                step_dist = np.hypot(np.diff(x), np.diff(y))
                keep_mask = np.concatenate(([True], step_dist > 1e-6))
                # Si el último punto duplica el punto de cierre inicial (seam_ds ~ 0), descartarlo
                if np.sum(keep_mask) > 2 and np.hypot(x[-1] - x[0], y[-1] - y[0]) < 1e-6:
                    keep_mask[-1] = False
                df = df.loc[keep_mask].reset_index(drop=True)
                x = x[keep_mask]
                y = y[keep_mask]
                if w_r_raw is not None:
                    w_r_raw = w_r_raw[keep_mask]
                    w_l_raw = w_l_raw[keep_mask]

            # Almacenar en memoria C-contigua (fila por waypoint) para óptima localidad de caché L1/L2
            self.track_data = np.ascontiguousarray(np.column_stack((x, y)), dtype=np.float64)

            # Extraer anchos de pista si existen (centerline)
            if w_r_raw is not None and w_l_raw is not None:
                self.track_widths = np.ascontiguousarray(
                    np.column_stack((w_r_raw, w_l_raw)), dtype=np.float64
                )

            # Extraer perfil cinemático completo si existe (raceline) con normalización de rumbo en S^1
            if {'s_m', 'psi_rad', 'kappa_radpm', 'vx_mps'}.issubset(set(df.columns)):
                psi_raw = df['psi_rad'].to_numpy(dtype=np.float64)
                psi_norm = (psi_raw + np.pi) % (2.0 * np.pi) - np.pi
                if len(x) > 1:
                    chord_psi = np.arctan2(np.diff(y), np.diff(x))
                    err_0 = np.median(np.abs((psi_norm[:-1] - chord_psi + np.pi) % (2.0 * np.pi) - np.pi))
                    for shift in (-0.5 * np.pi, 0.5 * np.pi):
                        err_s = np.median(
                            np.abs((psi_norm[:-1] + shift - chord_psi + np.pi) % (2.0 * np.pi) - np.pi)
                        )
                        if err_s + 0.1 < err_0:
                            psi_norm = (psi_norm + shift + np.pi) % (2.0 * np.pi) - np.pi
                            err_0 = err_s

                self.raceline_data = {
                    's_m': np.ascontiguousarray(df['s_m'].to_numpy(dtype=np.float64)),
                    'psi_rad': np.ascontiguousarray(psi_norm, dtype=np.float64),
                    'kappa_radpm': np.ascontiguousarray(df['kappa_radpm'].to_numpy(dtype=np.float64)),
                    'vx_mps': np.ascontiguousarray(df['vx_mps'].to_numpy(dtype=np.float64)),
                }
                if 'ax_mps2' in df.columns:
                    self.raceline_data['ax_mps2'] = np.ascontiguousarray(
                        df['ax_mps2'].to_numpy(dtype=np.float64)
                    )

            if verbose:
                print(
                    f"Circuito {self.track_name} cargado con exito: "
                    f"{len(self.track_data)} waypoints."
                )
            return self.track_data

        except Exception as e:
            print(f"Error al cargar el circuito '{self.filepath}': {e}")
            return None
