from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

# Create a 10x10 grid
x, y = np.linspace(0, 10, 10), np.linspace(0, 10, 10)
X, Y = np.meshgrid(x, y)
# Define a goal position
goalPoint = np.array([5, 5])
# Create a vector field where each vector points towards the goal
U = goalPoint[0] - X
V = goalPoint[1] - Y
# Normalize the vectors
norm = np.sqrt(U**2 + V**2)
U /= norm
V /= norm
# Create a quiver plot
plt.quiver(X, Y, U, V, color='r')
# Plot the goal point
plt.plot(goalPoint[0], goalPoint[1], 'bo', markersize=10)
# Set the limits and labels
plt.xlim(0, 10)
plt.ylim(0, 10)
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Vector Field Towards Goal Point')
# Show the plot
plt.grid()
plt.show()


class Obstacle:
    def __init__(self, position, radius):
        self.position = position
        self.radius = radius

    def is_inside(self, point):
        return np.linalg.norm(point - self.position) < self.radius

    def distance_to(self, point):
        # Calculate the distance from the point to the edge of the obstacle
        return np.linalg.norm(point - self.position) - self.radius

    def within_influence(self, point, influence_radius):
        # Check if the point is within the influence radius of the obstacle
        return np.linalg.norm(point - self.position) <= (self.radius + influence_radius)


def getAttractiveVector(point, goalPoint):
    vector = (goalPoint - point) * 2.0
    norm = np.linalg.norm(vector)
    if norm > 0:
        vector /= norm
    return vector


def getRepulsiveVector(point, obstacles):
    vector = np.array([0.0, 0.0], dtype=float)
    for obs in obstacles:
        if obs.within_influence(point, 2 * obs.radius):
            distance = obs.distance_to(point)
            if distance < 1e-5:
                distance = 1e-5
            force_direction = (point - obs.position) / \
                np.linalg.norm(point - obs.position)
            force_magnitude = 0.85 * \
                (1 / distance - 1 / (2 * obs.radius))  # Repulsive force
            # Ensure non-negative force
            force_magnitude = max(0, force_magnitude)
            force_magnitude *= 1 / (distance ** 2)  # Inverse square law
            vector += force_magnitude * force_direction
    return vector


x = np.linspace(0, 10, 20)
y = np.linspace(0, 10, 20)
X, Y = np.meshgrid(x, y)
# Define a goal position
goalPoint = np.array([8, 2])
# Obstacles are circles with a center and a radius
obstacles = [
    Obstacle(np.array([2, 2]), 1),
    Obstacle(np.array([7, 7]), 1),
    Obstacle(np.array([5, 8]), 0.5),
]
# Now let's update the vector field at every point to include the repulsive force
U = np.zeros_like(X, dtype=float)
V = np.zeros_like(Y, dtype=float)
for i in range(len(x)):
    for j in range(len(y)):
        point = np.array([X[i, j], Y[i, j]])
        attractive_vector = getAttractiveVector(point, goalPoint)
        repulsive_vector = getRepulsiveVector(point, obstacles)
        # Combine the attractive and repulsive vectors
        combined_vector = attractive_vector + repulsive_vector
        U[i, j] = combined_vector[0]
        V[i, j] = combined_vector[1]
# Normalize vectors for plotting
for i in range(U.shape[0]):
    for j in range(U.shape[1]):
        norm = np.linalg.norm([U[i, j], V[i, j]])
        if norm > 0:
            U[i, j] /= norm
            V[i, j] /= norm
plt.figure()
plt.quiver(X, Y, U, V, color='r')
for obs in obstacles:
    circle = plt.Circle(obs.position, obs.radius, color='g', alpha=1.0)
    plt.gca().add_artist(circle)
    # Plot influence radius
    influence_circle = plt.Circle(
        obs.position, 2 * obs.radius, color='y', alpha=0.2)
    plt.gca().add_artist(influence_circle)
plt.plot(goalPoint[0], goalPoint[1], 'bo', markersize=10)
plt.axis('equal')
plt.show()

# Create a 3D representation of the vector field
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# Create a 3D grid
x = np.linspace(0, 10, 5)
y = np.linspace(0, 10, 5)
z = np.linspace(0, 10, 5)
X, Y, Z = np.meshgrid(x, y, z)
# Define a goal position
goalPoint = np.array([5, 5, 5])
# Create a vector field where each vector points towards the goal
U = goalPoint[0] - X
V = goalPoint[1] - Y
W = goalPoint[2] - Z
# Normalize the vectors
norm = np.sqrt(U**2 + V**2 + W**2)
U /= norm
V /= norm
W /= norm
# Create a quiver plot
ax.quiver(X, Y, Z, U, V, W, color='r')
# Plot the goal point
ax.scatter(goalPoint[0], goalPoint[1], goalPoint[2], color='b', s=100)
# Set the limits and labels
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)
ax.set_zlim(0, 10)
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axi10s')
ax.set_title('3D Vector Field Towards Goal Point')
ax.view_init(elev=20, azim=30)  # Set the view angle
ax.set_box_aspect([1, 1, 1])  # Make the axes equal
# Show the plot
plt.show()
