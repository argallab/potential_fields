import numpy as np
import pandas as pd
import scipy.spatial.transform
from pathlib import Path
from pfield_3d_prototype import plot_kinematics

# Resolve important paths
THIS_DIR = Path(__file__).resolve().parent
DATA_DIR = THIS_DIR.parent / "data"


def get_latest_csv_file() -> Path:
    # Default to canonical name used by the planner
    return DATA_DIR / "planned_path.csv"
    # If you wish to pick the most recent timestamped file instead, uncomment below:
    name_template = "planned_path_"
    files = [p for p in DATA_DIR.glob(f"{name_template}*.csv")]
    # Timestamps are in the format: YYYYMMDD_HHMMSS
    if not files:
        raise FileNotFoundError(
            f"No {name_template}<timestamp>.csv files found in '{DATA_DIR}'.")
    latest_file = max(files, key=lambda p: p.stat().st_ctime)
    return latest_file


def parse_csv_header(file_path: Path) -> dict:
    """
    Parses the commented-out header of the planned path CSV file.
    """
    header_data = {}
    with open(file_path, 'r') as f:
        for line in f:
            if not line.startswith('#'):
                break

            line = line.strip('# ').strip()

            try:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()

                # A simple parser to convert strings to lists of floats or single floats
                if '[' in value and ']' in value:
                    # It's a list (e.g., a vector or quaternion)
                    parsed_value = [float(x)
                                    for x in value.strip('[]').split(',')]
                else:
                    # It's a scalar
                    parsed_value = float(value)

                if key == "Goal Position":
                    header_data['goal_pos'] = np.array(parsed_value)
                elif key == "Goal Orientation":
                    header_data['goal_q'] = np.array(parsed_value)
                elif key == "Goal Tolerance":
                    header_data['goal_tolerance_m'] = parsed_value
                elif key == "Angular Tolerance":
                    header_data['angular_tolerance_rad'] = parsed_value
                elif key == "Num Joints":
                    header_data['num_joints'] = int(parsed_value)
                elif key == "Planning Method":
                    header_data['planning_method'] = str(parsed_value)
            except (ValueError, IndexError):
                # Ignore lines that don't fit the key: value pattern
                continue

    return header_data


def create_plots_with_kinematics(df: pd.DataFrame, header_data: dict) -> None:
    # Build result dict expected by plot_kinematics
    res = {
        "t": np.asarray(df['time_s'], dtype=float),
        "x": np.asarray(df['pos_x_m'], dtype=float),
        "y": np.asarray(df['pos_y_m'], dtype=float),
        "z": np.asarray(df['pos_z_m'], dtype=float),
        "vx": np.asarray(df['vel_x_m_s'], dtype=float),
        "vy": np.asarray(df['vel_y_m_s'], dtype=float),
        "vz": np.asarray(df['vel_z_m_s'], dtype=float),
        "ee_qx": np.asarray(df['q_x'], dtype=float),
        "ee_qy": np.asarray(df['q_y'], dtype=float),
        "ee_qz": np.asarray(df['q_z'], dtype=float),
        "ee_qw": np.asarray(df['q_w'], dtype=float),
        "wx": np.asarray(df['ang_vel_x_rad_s'], dtype=float),
        "wy": np.asarray(df['ang_vel_y_rad_s'], dtype=float),
        "wz": np.asarray(df['ang_vel_z_rad_s'], dtype=float),
        "min_clearance_m": np.asarray(df['min_obstacle_clearance_m'], dtype=float),
        # Static data from header
        "goal_pos": header_data.get('goal_pos'),
        "goal_q": header_data.get('goal_q'),
        "goal_tolerance_m": header_data.get('goal_tolerance_m'),
        "angular_tolerance_rad": header_data.get('angular_tolerance_rad'),
        "planning_method": header_data.get('planning_method', 'Unknown'),
    }
    # Extract joint data if available
    if 'num_joints' in header_data:
        try:
            n_joints = header_data['num_joints']
            # Construct column names
            j_pos_keys = [f'joint_{i+1}_rad' for i in range(n_joints)]
            j_vel_keys = [f'joint_vel_{i+1}_rad_s' for i in range(n_joints)]
            j_torque_keys = [
                f'joint_torque_{i+1}_N_m' for i in range(n_joints)]

            # The header row might have an empty column name, which pandas reads as Unnamed: X
            # We filter those out.
            valid_columns = [
                col for col in df.columns if 'unnamed' not in col.lower()]

            if all(k in valid_columns for k in j_pos_keys):
                res["joint_positions"] = df[j_pos_keys].to_numpy(dtype=float)

            if all(k in valid_columns for k in j_vel_keys):
                res["joint_velocities"] = df[j_vel_keys].to_numpy(dtype=float)

            if all(k in valid_columns for k in j_torque_keys):
                res["joint_torques"] = df[j_torque_keys].to_numpy(dtype=float)
        except Exception as e:
            print(f"Warning: Could not extract joint data: {e}")

    # Extract Link Clearances
    link_clearance_cols = [
        c for c in df.columns if 'clearance_m' in c and 'link_' in c]
    if link_clearance_cols:
        # Sort by link index to ensure correct order (link_0, link_1, ...)
        link_clearance_cols.sort(key=lambda x: int(x.split('_')[1]))
        res["link_clearances"] = df[link_clearance_cols].to_numpy(dtype=float)
        res["link_names"] = [c.replace('_clearance_m', '')
                             for c in link_clearance_cols]

    # Extract Attraction Force
    if 'att_force_x_N' in df.columns:
        res["attraction_force"] = df[['att_force_x_N',
                                      'att_force_y_N', 'att_force_z_N']].to_numpy(dtype=float)

    # Extract Repulsive Forces (Magnitudes per link)
    import re
    rep_cols = [c for c in df.columns if '_rep_force_x_N' in c]
    link_indices = []
    for c in rep_cols:
        m = re.search(r'link_(\d+)_rep_force_x_N', c)
        if m:
            link_indices.append(int(m.group(1)))
    link_indices.sort()

    if link_indices:
        rep_mags = []
        for i in link_indices:
            fx = df[f'link_{i}_rep_force_x_N']
            fy = df[f'link_{i}_rep_force_y_N']
            fz = df[f'link_{i}_rep_force_z_N']
            mag = np.sqrt(fx*fx + fy*fy + fz*fz)
            rep_mags.append(mag)
        res["link_repulsive_magnitudes"] = np.column_stack(rep_mags)

    # Save outputs under data/ with a consistent base name
    save_base = DATA_DIR / 'planned_path'
    plot_kinematics(
        title='Planned Path Kinematics',
        res=res,
        show=False,
        save_path=str(save_base),
        description='Xarm 7-DoF Demo'
    )


if __name__ == "__main__":
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    csv_file = get_latest_csv_file()
    print(f"Reading data from {csv_file}")

    # Parse static data from the header
    header_data = parse_csv_header(csv_file)

    # Read the time-series data, skipping the header comments
    df = pd.read_csv(csv_file, comment='#')

    create_plots_with_kinematics(df, header_data)
