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
    plt.quiver(X, Y, U, V, color='b')
    # Plot the goal point
    plt.plot(goalPoint[0], goalPoint[1], 'go', markersize=10)
    # Set the limits and labels
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Vector Field Towards Goal Point')
    # Show the plot
    plt.grid()
    # plt.show()
    plt.savefig("basic_vector_field.png")
    print(f"Saved basic_vector_field.png")


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


def getAttractiveVector(point, goalPoint, attractive_gain=2.0, max_force=5.0):
    distance = np.linalg.norm(point - goalPoint)
    if distance < 1e-6:
        return np.zeros_like(point)
    direction = (point - goalPoint) / distance
    magnitude = attractive_gain * distance
    if magnitude > max_force:
        magnitude = max_force
    vector = -magnitude * direction
    return vector


def getRepulsiveVector(point, obstacles: list[Obstacle], repulsive_gain=1.0, max_force=5.0):
    vector = np.zeros_like(point)
    for obs in obstacles:
        influence_radius = 2 * obs.radius
        if obs.within_influence(point, influence_radius):
            distance = obs.distance_to(point)
            if distance < 1e-5:
                distance = 1e-5
            force_direction = (point - obs.position) / \
                np.linalg.norm(point - obs.position)
            # F = repulsiveGain * (1/influence - 1/distance) * (1 / distance^2)
            force_magnitude = repulsive_gain * \
                (1/distance - 1/influence_radius) * \
                (1 / distance**2)
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
        circle = plt.Circle(obs.position, obs.radius, color='r', alpha=1.0)
        plt.gca().add_artist(circle)
        # Plot influence radius
        influence_circle = plt.Circle(
            obs.position, 2 * obs.radius, color='y', alpha=0.2)
        plt.gca().add_artist(influence_circle)
    plt.plot(goalPoint[0], goalPoint[1], 'go', markersize=10)
    plt.title('Vector Field with Obstacles')
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.axis('equal')
    # plt.show()
    plt.savefig("2D_vector_field.png")
    print(f"Saved 2D_vector_field.png")


def create_3D_vector_field_with_obstacles():
    # Create a 3D representation of the vector field
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # Create a 3D grid
    x = np.linspace(0, 10, 5)
    y = np.linspace(0, 10, 5)
    z = np.linspace(0, 10, 5)
    X, Y, Z = np.meshgrid(x, y, z)
    # Define a goal position
    goalPoint = np.array([8, 2, 3])
    # Obstacles are spheres with a center and a radius
    obstacles = [
        Obstacle(np.array([2, 2, 2]), 1),
        Obstacle(np.array([7, 4, 7]), 2),
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
    ax.scatter(goalPoint[0], goalPoint[1], goalPoint[2], color='g', s=100)
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
        ax.plot_surface(x_sphere, y_sphere, z_sphere, color='r', alpha=1.0)
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
    ax.set_zlabel('Z-axis')
    ax.set_title('3D Vector Field Towards Goal Point')
    ax.view_init(elev=20, azim=30)  # Set the view angle
    ax.set_box_aspect([1, 1, 1])  # Make the axes equal
    # Show the plot
    # plt.show()
    plt.savefig("3D_vector_field.png")
    print(f"Saved 3D_vector_field.png")
    # Change the viewing angle and save another plot
    # Rotate the view
    ax.view_init(elev=20, azim=150)
    plt.savefig("3D_vector_field_rotated.png")
    print(f"Saved 3D_vector_field_rotated.png")


def graph_forces():
    # Create a graph of the attractive and repulsive forces
    # with respect to the distance from the obstacle/goal
    max_distance = np.sqrt(2) * 10
    max_force = 10.0
    distances = np.linspace(0.01, max_distance, 100)
    goal_point = np.array([0, 0])
    obstacle = Obstacle(np.array([0, 0]), 1)
    attractive_forces = [
        getAttractiveVector(
            point=np.array([d, 0]),
            goalPoint=goal_point,
            attractive_gain=1.0,
            max_force=max_force
        ) for d in distances
    ]
    repulsive_forces = [
        getRepulsiveVector(
            point=np.array([d, 0]),
            obstacles=[obstacle],
            repulsive_gain=1.0,
            max_force=max_force
        ) for d in distances
    ]
    # Calculate the magnitudes of the forces
    attractive_forces = np.linalg.norm(attractive_forces, axis=1)
    repulsive_forces = np.linalg.norm(repulsive_forces, axis=1)
    # Plot the forces on 2 graphs next to each other
    plt.figure()
    plt.plot(distances, repulsive_forces, label='Repulsive Force')
    # Plot influence radius as vertical line
    plt.axvline(x=2, color='r', linestyle='--', label='Influence Radius')
    plt.plot(distances, attractive_forces, label='Attractive Force')
    plt.title('Attractive and Repulsive Forces vs Distance to Goal/Obstacle')
    plt.text(
        6, 2.5, f"Max Force: {max_force:.1f} N\nRepulsive Gain: {1.0:.1f} N/m\nAttractive Gain: {1.0:.1f} N/m",
        fontsize=10, color='black', ha='center', va='center'
    )
    plt.xlabel('Distance [m]')
    plt.ylabel('Force Magnitude [N]')
    plt.xticks(np.arange(0, max_distance + 1, 1))
    plt.yticks(np.arange(0, max_force + 1, 1))
    plt.axhline(y=max_force, color='g', linestyle='--', label='Max Force')
    plt.xlim(0, max_distance * 1.15)
    plt.ylim(0, max_force * 1.15)
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.savefig("forces.png")
    print(f"Saved forces.png")
    # plt.show()


if __name__ == "__main__":
    create_basic_field()
    graph_forces()
    create_2D_vector_field_with_obstacles()
    create_3D_vector_field_with_obstacles()
