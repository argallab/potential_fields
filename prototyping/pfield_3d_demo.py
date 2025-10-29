# Re-run the 3D PF testbed (the previous state was cleared).

from dataclasses import dataclass
import numpy as np
import math
from typing import List, Tuple, Dict, Any, Optional
from abc import ABC, abstractmethod

import matplotlib.pyplot as plt
from pfield_3d_prototype import PotentialField3D, OBBObstacle, CylinderObstacle, SphereObstacle, \
    plot_kinematics


class Demo(ABC):
    def __init__(self, name: str, PF: PotentialField3D, description: str = ""):
        self.name = name
        self.PF = PF
        self.description = description  # Will be shown on kinematics plots as a text box

    @abstractmethod
    def create_obstacles(self):
        pass

    def add_pfs_to_compare(self, other_pfs: List[PotentialField3D]):
        self.other_pfs = other_pfs

    def run(self, start: Tuple[float, float, float],
            goal: Tuple[float, float, float],
            dt: float,
            tol: float,
            steps: int):
        self.create_obstacles()
        result = self.PF.plan_path(
            start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)
        print(
            f"[{self.name}] Goal reached:", result["goal_reached"],
            "Duration:", float(result["t"][-1]), "s",
            "End:", (float(result["x"][-1]),
                     float(result["y"][-1]), float(result["z"][-1])),
            "Path Length:", float(len(result["x"])), "m",
            "Min Path Clearance:",
            f"{self.PF.min_path_clearance(result['x'], result['y'], result['z']):.3f} m"
        )
        plot_kinematics(
            f"Kinematics vs Time [{self.name}]", result, show=True,
            save_path=f"demo_plots/{self.name.replace(' ', '_')}",
            description=self.description
        )
        self.PF.simulate(
            start, goal, dt=dt, goal_tolerance=tol, max_steps=steps,
            save_path=f"demo_gifs/{self.name.replace(' ', '_')}_Simulation.gif", writer="pillow", fps=35, dpi=90, save_every=10, bitrate=1800
        )


class ObstaclesInTheWayDemo(Demo):
    def __init__(self):
        super().__init__(
            name="Obstacles In The Way",
            PF=PotentialField3D(
                attractive_gain=1.2,
                repulsive_gain=0.22,
                linear_gain=1.0,
                max_speed=1.5,
                max_accel=3.2,
                beta_velocity=1.0,
            ),
            description="Sphere + OBB + Cylinder along the path",
        )

    def create_obstacles(self):
        self.PF.add_sphere(cx=1.5, cy=1.0, cz=0.7,
                           radius=0.7, influence_distance=1.0)
        self.PF.add_box(
            cx=3.0,
            cy=-0.5,
            cz=0.0,
            sx=1.2,
            sy=2.0,
            sz=1.0,
            yaw=np.deg2rad(25.0),
            pitch=np.deg2rad(10.0),
            roll=np.deg2rad(-5.0),
            influence_distance=0.9,
        )
        # Cylinder (axis along local Z)
        self.PF.add_cylinder(
            cx=1.0,
            cy=-1.0,
            cz=0.3,
            radius=0.45,
            height=1.2,
            yaw=np.deg2rad(15.0),
            pitch=np.deg2rad(0.0),
            roll=np.deg2rad(0.0),
            influence_distance=0.8,
        )


class NoObstaclesDemo(Demo):
    def __init__(self):
        super().__init__(
            name="No Obstacles",
            PF=PotentialField3D(
                attractive_gain=1.0,
                repulsive_gain=0.0,
                linear_gain=1.0,
                max_speed=2.0,
                max_accel=4.0,
                beta_velocity=1.0,
            ),
            description="Straight shot to goal with no obstacles",
        )

    def create_obstacles(self):
        # No obstacles to add
        return


class NarrowGapCorridorDemo(Demo):
    """
    Two long OBB 'walls' separated by a narrow gap (doorway).
    Tests: repulsion shaping near tight passages, speed/accel limits.
    """

    def __init__(self):
        super().__init__(
            name="Narrow Gap Corridor",
            PF=PotentialField3D(
                attractive_gain=1.2,
                repulsive_gain=0.35,
                linear_gain=1.0,
                max_speed=1.2,
                max_accel=2.0,
                beta_velocity=1.0,
            ),
            description="Two long OBB walls with a narrow doorway",
        )

    def create_obstacles(self):
        wall_len = 6.0
        wall_th = 0.3
        gap_y = 0.2  # half-gap
        inf = 0.6

        # Left wall (upper segment), right wall (lower segment) — rotated slightly
        self.PF.add_box(
            cx=-0.5, cy=+gap_y+1.0, cz=0.0, sx=wall_len, sy=wall_th, sz=1.5,
            yaw=np.deg2rad(5.0), influence_distance=inf
        )
        self.PF.add_box(
            cx=-0.5, cy=-gap_y-1.0, cz=0.0, sx=wall_len, sy=wall_th, sz=1.5,
            yaw=np.deg2rad(-5.0), influence_distance=inf
        )


class UTrapLocalMinimaDemo(Demo):
    """
    U-shaped cul-de-sac (three OBBs). Pure PFs often stall here.
    """

    def __init__(self):
        super().__init__(
            name="U Trap Local Minima",
            PF=PotentialField3D(
                attractive_gain=1.0,
                repulsive_gain=0.4,
                linear_gain=1.0,
                max_speed=1.0,
                max_accel=2.0,
                beta_velocity=1.0,
            ),
            description="U-shaped cul-de-sac — classic local minima case",
        )

    def create_obstacles(self):
        inf = 0.8
        # Bottom of U
        self.PF.add_box(
            cx=0.0, cy=-0.5, cz=0.0, sx=4.0, sy=0.3,
            sz=1.5, yaw=0.0, influence_distance=inf
        )
        # Sides of U
        self.PF.add_box(
            cx=-1.9, cy=0.7, cz=0.0, sx=0.3, sy=2.0,
            sz=1.5, yaw=0.0, influence_distance=inf
        )
        self.PF.add_box(
            cx=+1.9, cy=0.7, cz=0.0, sx=0.3, sy=2.0,
            sz=1.5, yaw=0.0, influence_distance=inf
        )


class ClutteredSpheresDemo(Demo):
    """
    Random 'forest' of spheres with varied radii/influence.
    Tests: vector superposition, oscillations, soft saturation ergonomics.
    """

    def __init__(self, seed: int = 7):
        super().__init__(
            name="Cluttered Spheres",
            PF=PotentialField3D(
                attractive_gain=1.1,
                repulsive_gain=0.25,
                linear_gain=1.0,
                max_speed=1.3,
                max_accel=2.5,
                beta_velocity=1.2,
            ),
            description="Random forest of spheres with a clearer center corridor",
        )
        self.seed = seed

    def create_obstacles(self):
        rng = np.random.default_rng(self.seed)
        # Place ~10 spheres in a region; keep a corridor open
        for _ in range(10):
            cx = rng.uniform(-1.5, 2.5)
            cy = rng.uniform(-1.0, 1.0)
            cz = rng.uniform(-0.4, 0.4)
            r = rng.uniform(0.25, 0.55)
            inf = rng.uniform(0.6, 1.0)
            # Keep center corridor near y ~ 0 slightly clearer
            if abs(cy) < 0.2 and rng.random() < 0.7:
                cy += rng.choice([-1, 1]) * rng.uniform(0.3, 0.6)
            self.PF.add_sphere(cx, cy, cz, r, influence_distance=inf)


class StartInsideObstacleDemo(Demo):
    """
    Start point is inside a sphere; should be pushed out along outward normal.
    Verifies signed distance handling and clamp near singularity.
    """

    def __init__(self):
        super().__init__(
            name="Start Inside Obstacle",
            PF=PotentialField3D(
                attractive_gain=1.0,
                repulsive_gain=0.35,
                linear_gain=1.0,
                max_speed=1.2,
                max_accel=2.2,
                beta_velocity=1.0,
            ),
            description="Start inside sphere; should be pushed outward",
        )

    def create_obstacles(self):
        self.PF.add_sphere(cx=0.0, cy=0.0, cz=0.0,
                           radius=0.7, influence_distance=1.0)


class HighSpeedLowAccelDemo(Demo):
    """
    Very high speed limit but tight acceleration limit — showcases the rate limiter.
    No obstacles; straight shot to goal.
    """

    def __init__(self):
        super().__init__(
            name="High Speed Low Accel",
            PF=PotentialField3D(
                attractive_gain=1.5,
                repulsive_gain=0.0,
                linear_gain=1.2,
                max_speed=4.0,    # high speed ceiling
                max_accel=0.5,    # small accel
                beta_velocity=1.0,
            ),
            description="Rate limiter engaged frequently due to tight accel",
        )

    def create_obstacles(self):
        # No obstacles
        return


class CollisionPreventionDemo(Demo):
    """
    Showcases how motion constraints prevent collisions in a tight turn near an obstacle.
    Overlays two runs with identical obstacles but different motion limits.
    """

    def __init__(self):
        # Use 'env' PF to store obstacles; per-run PFs will copy them
        super().__init__(
            name="Collision Prevention",
            PF=PotentialField3D(
                attractive_gain=1.2,
                repulsive_gain=0.25,
                linear_gain=1.0,
                max_speed=1.0,   # placeholders
                max_accel=1.0,
                beta_velocity=1.0,
            ),
            description="Compare risky vs safe motion limits with same field",
        )

    def create_obstacles(self):
        # Shared environment obstacles
        self.PF.add_sphere(cx=0.5, cy=0.0, cz=0.0,
                           radius=0.65, influence_distance=0.9)
        self.PF.add_box(
            cx=1.2, cy=-0.1, cz=0.0, sx=0.2, sy=1.2, sz=1.0,
            yaw=np.deg2rad(10.0), influence_distance=0.6
        )

    def run(
        self,
        start: Optional[Tuple[float, float, float]] = None,
        goal: Optional[Tuple[float, float, float]] = None,
        dt: float = 0.02,
        tol: float = 0.06,
        steps: int = 3500,
    ):
        # Defaults from the original function
        if start is None:
            start = (-1.8, -0.8, 0.0)
        if goal is None:
            goal = (2.0, 0.9, 0.0)

        # Ensure obstacles are created in env PF
        self.create_obstacles()

        env = self.PF

        # Run A: Risky — high speed, tight accel
        pf_risky = PotentialField3D(
            attractive_gain=env.attractive_gain,
            repulsive_gain=env.repulsive_gain,
            linear_gain=env.linear_gain,
            max_speed=3.5,
            max_accel=0.5,
            beta_velocity=1.0,
        )
        pf_risky._spheres = env._spheres.copy()
        pf_risky._obbs = env._obbs.copy()

        resA = pf_risky.plan_path(
            start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)
        clearanceA = pf_risky.min_path_clearance(
            resA["x"], resA["y"], resA["z"])

        # Run B: Safe — lower speed cap, higher accel
        pf_safe = PotentialField3D(
            attractive_gain=env.attractive_gain,
            repulsive_gain=env.repulsive_gain,
            linear_gain=env.linear_gain,
            max_speed=1.2,
            max_accel=2.5,
            beta_velocity=1.0,
        )
        pf_safe._spheres = env._spheres.copy()
        pf_safe._obbs = env._obbs.copy()

        resB = pf_safe.plan_path(
            start, goal, dt=dt, goal_tolerance=tol, max_steps=steps)
        clearanceB = pf_safe.min_path_clearance(
            resB["x"], resB["y"], resB["z"])

        print("[constraints demo]")
        print(
            f"  Risky: goal_reached={resA['goal_reached']}, min_clearance={clearanceA:.3f} m "
            f"({'COLLISION' if clearanceA < 0 else 'OK'})"
        )
        print(
            f"  Safe : goal_reached={resB['goal_reached']}, min_clearance={clearanceB:.3f} m "
            f"({'COLLISION' if clearanceB < 0 else 'OK'})"
        )

        # Visualization: overlay both paths with the same obstacles
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title("Damping & Motion Constraints Prevent Collisions")
        ax.set_box_aspect([1, 1, 1])

        xs = [start[0], goal[0]] + [s.cx for s in env._spheres] + \
            [b.cx for b in env._obbs]
        ys = [start[1], goal[1]] + [s.cy for s in env._spheres] + \
            [b.cy for b in env._obbs]
        zs = [start[2], goal[2]] + [s.cz for s in env._spheres] + \
            [b.cz for b in env._obbs]
        ax.set_xlim(min(xs) - 2, max(xs) + 2)
        ax.set_ylim(min(ys) - 2, max(ys) + 2)
        ax.set_zlim(min(zs) - 2, max(zs) + 2)
        ax.grid(False)

        env.draw_obstacles(ax)

        ax.plot(resA["x"], resA["y"], resA["z"], linewidth=2.0,
                label=f"Risky (min d={clearanceA:.2f} m)")
        ax.plot(
            resB["x"], resB["y"], resB["z"], linewidth=2.0, linestyle="--", label=f"Safe (min d={clearanceB:.2f} m)"
        )
        ax.scatter([start[0]], [start[1]], [start[2]], marker='o')
        ax.scatter([goal[0]], [goal[1]], [goal[2]], marker='^')

        ax.legend(loc="upper left")
        plt.show()

        # Optional: speed profiles
        def _speed(res):
            return np.sqrt(res["vx"] ** 2 + res["vy"] ** 2 + res["vz"] ** 2)

        fig2, ax2 = plt.subplots(figsize=(8, 3.5))
        ax2.plot(resA["t"], _speed(resA), label="Risky speed")
        ax2.plot(resB["t"], _speed(resB), label="Safe speed", linestyle="--")
        ax2.set_title("Speed vs Time")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("|v| [m/s]")
        ax2.grid(True, linestyle=':')
        ax2.legend()
        plt.show()


if __name__ == "__main__":
    ObstaclesInTheWayDemo().run(
        start=(-2.0, -1.5, -0.5), goal=(4.0, 1.0, 0.6), dt=0.01, tol=0.1, steps=4000
    )

    NoObstaclesDemo().run(
        start=(-2.0, -1.5, -0.5), goal=(4.0, 1.0, 0.6), dt=0.01, tol=0.1, steps=3000
    )

    NarrowGapCorridorDemo().run(
        start=(-3.0, 0.0, 0.0), goal=(3.0, 0.0, 0.0), dt=0.02, tol=0.05, steps=3500
    )

    UTrapLocalMinimaDemo().run(
        start=(0.0, 0.0, 0.0), goal=(0.0, 2.5, 0.0), dt=0.02, tol=0.05, steps=4000
    )

    ClutteredSpheresDemo(seed=7).run(
        start=(-2.0, 0.0, 0.0), goal=(3.0, 0.0, 0.0), dt=0.02, tol=0.06, steps=4000
    )

    StartInsideObstacleDemo().run(
        start=(0.0, 0.0, 0.0), goal=(2.0, 0.0, 0.0), dt=0.02, tol=0.05, steps=3000
    )

    HighSpeedLowAccelDemo().run(
        start=(-3.0, 0.0, 0.0), goal=(3.0, 0.0, 0.0), dt=0.02, tol=0.05, steps=2500
    )

    # # 8) Collision prevention comparison (overrides run)
    # CollisionPreventionDemo().run()
