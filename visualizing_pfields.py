import matplotlib.pyplot as plt
import numpy as np


def create_basic_field():
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


def getAttractiveVector(point, goalPoint, attractive_gain=1.0):
    distance = np.linalg.norm(point - goalPoint)
    if distance < 1e-6:
        return np.zeros_like(point)
    direction = (goalPoint - point) / distance
    magnitude = attractive_gain * distance
    vector = magnitude * direction
    return vector


def getRepulsiveVector(point, obstacles, repulsive_gain=1.0, max_force=5.0):
    vector = np.zeros_like(point)
    for obs in obstacles:
        influence_radius = 2 * obs.radius
        if obs.within_influence(point, influence_radius):
            distance = obs.distance_to(point)
            if distance < 1e-5:
                distance = 1e-5
            force_direction = (point - obs.position) / \
                np.linalg.norm(point - obs.position)
            force_magnitude = repulsive_gain * \
                (1 / distance**2)  # Repulsive force
            if force_magnitude > max_force:
                force_magnitude = max_force
            # print(
            #     f"Repulsion Magnitude: {force_magnitude:.4f}, Direction: {force_direction}"
            # )
            vector += (force_magnitude * force_direction)
    return vector


def create_2D_vector_field_with_obstacles():
    # Create a 10x10 grid
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
    magnitude = np.sqrt(U**2 + V**2)
    U_norm = np.copy(U)
    V_norm = np.copy(V)
    # Normalize vectors for plotting
    for i in range(U.shape[0]):
        for j in range(U.shape[1]):
            norm = np.linalg.norm([U[i, j], V[i, j]])
            if norm > 0:
                U_norm[i, j] /= norm
                V_norm[i, j] /= norm
    plt.figure()
    plt.quiver(X, Y, U_norm, V_norm, magnitude, cmap='viridis')
    plt.colorbar(label='Magnitude')
    for obs in obstacles:
        circle = plt.Circle(obs.position, obs.radius, color='g', alpha=1.0)
        plt.gca().add_artist(circle)
        # Plot influence radius
        influence_circle = plt.Circle(
            obs.position, 2 * obs.radius, color='y', alpha=0.2)
        plt.gca().add_artist(influence_circle)
    plt.plot(goalPoint[0], goalPoint[1], 'bo', markersize=10)
    plt.title('Vector Field with Obstacles')
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.axis('equal')
    plt.show()


def create_3D_vector_field_with_obstacles():
    # Create a 3D representation of the vector field
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # Create a 3D grid
    x = np.linspace(0, 10, 10)
    y = np.linspace(0, 10, 10)
    z = np.linspace(0, 10, 10)
    X, Y, Z = np.meshgrid(x, y, z)
    # Define a goal position
    goalPoint = np.array([8, 2, 3])
    # Obstacles are spheres with a center and a radius
    obstacles = [
        Obstacle(np.array([2, 2, 2]), 1),
        # Obstacle(np.array([7, 7, 7]), 1),
        Obstacle(np.array([5, 8, 5]), 0.5)
    ]
    U = np.zeros_like(X, dtype=float)
    V = np.zeros_like(Y, dtype=float)
    W = np.zeros_like(Z, dtype=float)
    for i in range(len(x)):
        for j in range(len(y)):
            for k in range(len(z)):
                point = np.array([X[i, j, k], Y[i, j, k], Z[i, j, k]])
                attractive_vector = getAttractiveVector(point, goalPoint)
                repulsive_vector = getRepulsiveVector(point, obstacles)
                # Combine the attractive and repulsive vectors
                combined_vector = attractive_vector + repulsive_vector
                U[i, j, k] = combined_vector[0]
                V[i, j, k] = combined_vector[1]
                W[i, j, k] = combined_vector[2]
    # Create a quiver plot
    ax.quiver(X, Y, Z, U, V, W, length=0.75, normalize=True)
    # Plot the goal point
    ax.scatter(goalPoint[0], goalPoint[1], goalPoint[2], color='b', s=100)
    # Plot obstacles
    for obst in obstacles:
        # Create a sphere for the obstacle
        u = np.linspace(0, 2 * np.pi, 100)
        v = np.linspace(0, np.pi, 100)
        x_sphere = obst.radius * \
            np.outer(np.cos(u), np.sin(v)) + obst.position[0]
        y_sphere = obst.radius * \
            np.outer(np.sin(u), np.sin(v)) + obst.position[1]
        z_sphere = obst.radius * \
            np.outer(np.ones(np.size(u)), np.cos(v)) + obst.position[2]
        ax.plot_surface(x_sphere, y_sphere, z_sphere, color='g', alpha=1.0)
        # Plot influence radius
        u = np.linspace(0, 2 * np.pi, 100)
        v = np.linspace(0, np.pi, 100)
        x_influence = 2 * obst.radius * \
            np.outer(np.cos(u), np.sin(v)) + obst.position[0]
        y_influence = 2 * obst.radius * \
            np.outer(np.sin(u), np.sin(v)) + obst.position[1]
        z_influence = 2 * obst.radius * \
            np.outer(np.ones(np.size(u)), np.cos(v)) + obst.position[2]
        ax.plot_surface(x_influence, y_influence,
                        z_influence, color='y', alpha=0.2)
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


if __name__ == "__main__":
    # create_basic_field()
    # create_2D_vector_field_with_obstacles()
    create_3D_vector_field_with_obstacles()
