import matplotlib.pyplot as plt
import pandas as pd
import os

# Read planned_path_<timestamp>.csv file
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))


def get_latest_csv_file():
    return "planned_path.csv"
    # name_template = "planned_path_"
    # files = [f for f in os.listdir(CURRENT_DIR) if f.startswith(
    #     name_template) and f.endswith('.csv')]
    # if not files:
    #     raise FileNotFoundError(
    #         "No planned_path_<timestamp>.csv files found in 'data' directory.")
    # latest_file = max(files, key=lambda f: os.path.getctime(
    #     os.path.join(CURRENT_DIR, f)))
    # return os.path.join(CURRENT_DIR, latest_file)


def extrapolate_accel(df):
    # Extrapolate acceleration data from position and velocity data
    # and the time intervals and add to the dataframe
    # as 'ee_ax', 'ee_ay', 'ee_az'
    df['ee_ax'] = df['ee_vx'].diff() / df['time_s'].diff()
    df['ee_ay'] = df['ee_vy'].diff() / df['time_s'].diff()
    df['ee_az'] = df['ee_vz'].diff() / df['time_s'].diff()
    df.fillna(0, inplace=True)  # Replace NaN values with 0
    return df


def create_plot(df):
    # Extract end-effector positions
    ee_px = df['ee_px']
    ee_py = df['ee_py']
    ee_pz = df['ee_pz']

    # Plotting the 3D path
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(ee_px, ee_py, ee_pz, label='Planned Path', color='blue')
    ax.scatter(ee_px.iloc[0], ee_py.iloc[0], ee_pz.iloc[0],
               color='green', s=100, label='Start')
    ax.scatter(ee_px.iloc[-1], ee_py.iloc[-1],
               ee_pz.iloc[-1], color='red', s=100, label='End')

    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title('End-Effector Planned Path')
    ax.legend()

    plt.savefig('planned_path_ee_path.png')

    # Create 4 2D plots on the same figure as subplots
    # 1. EE Position vs Time
    # 2. EE Velocity vs Time
    # 3. EE Acceleration vs Time
    # 4. Joint Angles vs Time (if joint data is available)
    time_s = df['time_s']
    ee_vx = df['ee_vx']
    ee_vy = df['ee_vy']
    ee_vz = df['ee_vz']
    joint_columns = [col for col in df.columns if col.startswith('joint_')]
    num_joints = len(joint_columns)
    fig, axs = plt.subplots(2, 2, figsize=(14, 14))
    # 1. EE Position vs Time
    axs[0, 0].plot(time_s, df['ee_px'], label='EE X', color='blue')
    axs[0, 0].plot(time_s, df['ee_py'], label='EE Y', color='orange')
    axs[0, 0].plot(time_s, df['ee_pz'], label='EE Z', color='green')
    axs[0, 0].set_title('End-Effector Position vs Time')
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('Position (m)')
    axs[0, 0].legend()
    axs[0, 0].grid()
    # 2. EE Velocity vs Time
    axs[1, 0].plot(time_s, ee_vx, label='EE Vx', color='blue')
    axs[1, 0].plot(time_s, ee_vy, label='EE Vy', color='orange')
    axs[1, 0].plot(time_s, ee_vz, label='EE Vz', color='green')
    axs[1, 0].set_title('End-Effector Velocity vs Time')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('Velocity (m/s)')
    axs[1, 0].legend()
    axs[1, 0].grid()
    # 3. EE Acceleration vs Time
    axs[0, 1].plot(time_s, df['ee_ax'], label='EE Ax', color='blue')
    axs[0, 1].plot(time_s, df['ee_ay'], label='EE Ay', color='orange')
    axs[0, 1].plot(time_s, df['ee_az'], label='EE Az', color='green')
    axs[0, 1].set_title('End-Effector Acceleration vs Time')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Acceleration (m/s²)')
    axs[0, 1].legend()
    axs[0, 1].grid()
    # 4. Joint Angles vs Time (if joint data is available)
    if num_joints > 0:
        for joint_col in joint_columns:
            axs[1, 1].plot(time_s, df[joint_col], label=joint_col)
        axs[1, 1].set_title('Joint Angles vs Time')
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].set_ylabel('Angle (rad)')
        axs[1, 1].legend()
        axs[1, 1].grid()
    else:
        axs[1, 1].text(0.5, 0.5, 'No Joint Data Available',
                       horizontalalignment='center', verticalalignment='center', transform=axs[1, 1].transAxes)
        axs[1, 1].set_title('Joint Angles vs Time')
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].set_ylabel('Angle (rad)')

    plt.tight_layout()
    plt.savefig('planned_path_2d_plots.png')
    plt.show()


if __name__ == "__main__":
    csv_file = get_latest_csv_file()
    print(f"Reading data from {csv_file}")
    df = pd.read_csv(csv_file)
    df = extrapolate_accel(df)
    create_plot(df)
