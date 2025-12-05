import numpy as np
import matplotlib.pyplot as plt


def compute_attractive_force_magnitude(distance: float, attractive_gain: float, d_star_threshold: float) -> float:
    """
    Computes the magnitude of the attractive linear force based on Choset's potential field formulation.

    |F| = k_att * distance, if distance <= d_star_threshold (Quadratic Potential)
    |F| = k_att * d_star_threshold, if distance > d_star_threshold (Conical Potential)
    """
    if distance <= d_star_threshold:
        # Quadratic Attraction Region (Linear force magnitude: F = k * d)
        return attractive_gain * distance
    else:
        # Conical Attraction Region (Constant force magnitude: F = k * d*)
        return attractive_gain * d_star_threshold


# --- 2. Setup Plotting Parameters ---
# k_att [N/m] - Assuming 1.0 N/m for visualization simplicity
ATTRACTIVE_GAIN = 1.0
D_STAR_THRESHOLDS = [0.05, 0.1, 0.5, 0.875, 1.0, 2.0]  # [m]
MAX_DISTANCE = 2.5  # [m]

# Generate 500 distance points for a smooth curve
distances = np.linspace(0, MAX_DISTANCE, 500)

# --- 3. Plotting ---
plt.figure(figsize=(12, 7))

# Iterate and plot for each threshold
for d_star in D_STAR_THRESHOLDS:
    # Calculate force magnitude array
    forces = np.array([
        compute_attractive_force_magnitude(d, ATTRACTIVE_GAIN, d_star)
        for d in distances
    ])

    # Plot the Force vs Distance line
    # Note: We assign the color to the plot line and then use it for the dashed line
    line, = plt.plot(distances, forces, label=f'$d^* = {d_star}$ m')

    # Add a vertical dashed line at the threshold
    # The ymax is calculated to ensure the line stops exactly at the kink point (d=d*, F=k*d*)
    max_y_limit = ATTRACTIVE_GAIN * MAX_DISTANCE * 0.8
    plt.axvline(x=d_star, color=line.get_color(),
                linestyle='--', alpha=0.6,
                label=f'Threshold at $d^* = {d_star}$ m')

# Set labels and title
plt.title('Attractive Force Magnitude vs. Distance (Choset Formulation)', fontsize=14)
plt.xlabel('Distance to Goal (m)', fontsize=12)
plt.ylabel(
    r'Force Magnitude $|F|$ (N) [Assuming $k_{att} = 1.0$]', fontsize=12)
plt.grid(True, linestyle=':', alpha=0.7)

# Create two separate legends: one for the force curves, one for the threshold lines
handles, labels = plt.gca().get_legend_handles_labels()
# Filter out the threshold lines from the main legend
curve_handles = [h for h, l in zip(handles, labels) if 'Threshold' not in l]
curve_labels = [l for l in labels if 'Threshold' not in l]

plt.legend(curve_handles, curve_labels, loc='lower right',
           title='Quadratic Threshold $d^*$', fontsize=10)

plt.xlim(0, MAX_DISTANCE)
# Set a reasonable y limit based on the maximum d* of 2.0
plt.ylim(0, ATTRACTIVE_GAIN * 1.2)
plt.tight_layout()

# You can save the figure to a file:
filePath = '../data/force_vs_distance_with_dstar.png'
plt.savefig(filePath)
print(f"Plot saved to {filePath}")
# Or display it:
# plt.show()
