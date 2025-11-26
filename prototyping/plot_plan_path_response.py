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


def create_plots_with_kinematics(df: pd.DataFrame) -> None:
    # Build result dict expected by plot_kinematics
    # new headers:
    # time_s,pos_x_m,pos_y_m,pos_z_m,q_x,q_y,q_z,q_w,vel_x_m_s,vel_y_m_s,vel_z_m_s,ang_vel_x_rad_s,ang_vel_y_rad_s,ang_vel_z_rad_s,min_obstacle_clearance_m,num_joints,joint_1_rad,joint_2_rad,joint_3_rad,joint_4_rad,joint_5_rad,joint_6_rad,joint_7_rad,joint_vel_1_rad_s,joint_vel_2_rad_s,joint_vel_3_rad_s,joint_vel_4_rad_s,joint_vel_5_rad_s,joint_vel_6_rad_s,joint_vel_7_rad_s
    res = {
        "t": np.asarray(df['time_s'], dtype=float),
        "x": np.asarray(df['pos_x_m'], dtype=float),
        "y": np.asarray(df['pos_y_m'], dtype=float),
        "z": np.asarray(df['pos_z_m'], dtype=float),
        "vx": np.asarray(df['vel_x_m_s'], dtype=float),
        "vy": np.asarray(df['vel_y_m_s'], dtype=float),
        "vz": np.asarray(df['vel_z_m_s'], dtype=float),
    }
    # Include orientation quaternions if available for angle_to_goal computation
    if {'q_x', 'q_y', 'q_z', 'q_w'}.issubset(df.columns):
        res["ee_qx"] = np.asarray(df['q_x'], dtype=float)
        res["ee_qy"] = np.asarray(df['q_y'], dtype=float)
        res["ee_qz"] = np.asarray(df['q_z'], dtype=float)
        res["ee_qw"] = np.asarray(df['q_w'], dtype=float)
    # Optional angular velocities and clearance
    if {'ang_vel_x_rad_s', 'ang_vel_y_rad_s', 'ang_vel_z_rad_s'}.issubset(df.columns):
        res["wx"] = np.asarray(df['ang_vel_x_rad_s'], dtype=float)
        res["wy"] = np.asarray(df['ang_vel_y_rad_s'], dtype=float)
        res["wz"] = np.asarray(df['ang_vel_z_rad_s'], dtype=float)
    if 'min_obstacle_clearance_m' in df.columns:
        res["min_clearance_m"] = np.asarray(
            df['min_obstacle_clearance_m'], dtype=float)

    # Extract joint data if available
    if 'num_joints' in df.columns:
        try:
            n_joints = int(df['num_joints'].iloc[0])
            # Construct column names
            j_pos_keys = [f'joint_{i+1}_rad' for i in range(n_joints)]
            j_vel_keys = [f'joint_vel_{i+1}_rad_s' for i in range(n_joints)]

            if all(k in df.columns for k in j_pos_keys):
                res["joint_positions"] = df[j_pos_keys].to_numpy(dtype=float)

            if all(k in df.columns for k in j_vel_keys):
                res["joint_velocities"] = df[j_vel_keys].to_numpy(dtype=float)
        except Exception as e:
            print(f"Warning: Could not extract joint data: {e}")

    # Infer goal as the final EE position (assumption for planned path CSV)
    goal = (float(df['pos_x_m'].iloc[-1]),
            float(df['pos_y_m'].iloc[-1]),
            float(df['pos_z_m'].iloc[-1]))
    goal_orientation = (float(df['q_x'].iloc[-1]),
                        float(df['q_y'].iloc[-1]),
                        float(df['q_z'].iloc[-1]),
                        float(df['q_w'].iloc[-1]))
    # Convert quaternion to Euler angles (roll, pitch, yaw) for goal_orientation
    quat = scipy.spatial.transform.Rotation.from_quat(goal_orientation)
    goal_euler = quat.as_euler('xyz', degrees=False)

    # Save outputs under data/ with a consistent base name
    save_base = DATA_DIR / 'planned_path'
    plot_kinematics(
        title='Planned Path Kinematics',
        res=res,
        show=False,
        save_path=str(save_base),
        description='Xarm 7-DoF Demo',
        goal=goal,
        goal_orientation=goal_euler
    )


if __name__ == "__main__":
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    csv_file = get_latest_csv_file()
    print(f"Reading data from {csv_file}")
    df = pd.read_csv(csv_file)
    create_plots_with_kinematics(df)
