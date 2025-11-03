import numpy as np
import pandas as pd
from pathlib import Path
from pfield_3d_prototype import plot_kinematics

# Resolve important paths
THIS_DIR = Path(__file__).resolve().parent
DATA_DIR = THIS_DIR.parent / "data"


def get_latest_csv_file() -> Path:
    # Default to canonical name used by the planner
    return DATA_DIR / "planned_path.csv"
    # If you wish to pick the most recent timestamped file instead, uncomment below:
    # name_template = "planned_path_"
    # files = [p for p in DATA_DIR.glob(f"{name_template}*.csv")]
    # if not files:
    #     raise FileNotFoundError(
    #         f"No {name_template}<timestamp>.csv files found in '{DATA_DIR}'.")
    # latest_file = max(files, key=lambda p: p.stat().st_ctime)
    # return latest_file


def create_plots_with_kinematics(df: pd.DataFrame) -> None:
    # Build result dict expected by plot_kinematics
    res = {
        "t": np.asarray(df['time_s'], dtype=float),
        "x": np.asarray(df['ee_px'], dtype=float),
        "y": np.asarray(df['ee_py'], dtype=float),
        "z": np.asarray(df['ee_pz'], dtype=float),
        "vx": np.asarray(df['ee_vx'], dtype=float),
        "vy": np.asarray(df['ee_vy'], dtype=float),
        "vz": np.asarray(df['ee_vz'], dtype=float),
    }
    # Optional angular velocities and clearance
    if {'ee_wx', 'ee_wy', 'ee_wz'}.issubset(df.columns):
        res["wx"] = np.asarray(df['ee_wx'], dtype=float)
        res["wy"] = np.asarray(df['ee_wy'], dtype=float)
        res["wz"] = np.asarray(df['ee_wz'], dtype=float)
    if 'min_clearance_m' in df.columns:
        res["min_clearance_m"] = np.asarray(df['min_clearance_m'], dtype=float)
    # Infer goal as the final EE position (assumption for planned path CSV)
    goal = (float(df['ee_px'].iloc[-1]),
            float(df['ee_py'].iloc[-1]),
            float(df['ee_pz'].iloc[-1]))

    # Save outputs under data/ with a consistent base name
    save_base = DATA_DIR / 'planned_path'
    plot_kinematics(
        title='Planned Path Kinematics',
        res=res,
        show=False,
        save_path=str(save_base),
        description='From planned_path.csv',
        goal=goal,
    )


if __name__ == "__main__":
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    csv_file = get_latest_csv_file()
    print(f"Reading data from {csv_file}")
    df = pd.read_csv(csv_file)
    create_plots_with_kinematics(df)
