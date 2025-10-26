# Papers and references for path planning using potential fields:
# https://pdf.sciencedirectassets.com/271436/1-s2.0-S0019057823X00081/1-s2.0-S0019057823000769/main.pdf?X-Amz-Security-Token=IQoJb3JpZ2luX2VjEH8aCXVzLWVhc3QtMSJHMEUCIQCU4tQdRvn779HmKhj749r8JWHVpd8CCaMoHJZ1xecWrQIgbn7pQI00rek3PANzj5Vec%2BP1CNfRXjoqTzIoozQNBlEqswUINxAFGgwwNTkwMDM1NDY4NjUiDA%2B%2BZMCgzrQsd9sOMCqQBS9Id2dMzomIrRRNsJuVSGr4tC5om26RG1iy3zZBqeda%2FMrhQUErNUSTGHFlt7TtkNxiBvRoPtACVltz7pZ6kf1eNPfElCS9hBrV8ogomOXGUtKxHGt5nAEcdzqZnmcugOZDcYBA3KyKhu8ePVEtsYODUNbvSExJp4G3KNgk2hZnBQVQuzwrr7RET0uhtO3YaW2yfrdWSyeC4xiuMBRgAtNW6K%2BrmY2rysh61ozdCpzGtAjzEnD%2FOjxWkXF7T8br8OOmasyrDMT9K%2FRQGBlnsQZtfTueAVJk%2FkjWWDpPVt91QaYi4vYZp%2FkJ9vlmpbUK%2FqExMDrQihmzOe%2BDWMeilyWmh1m83Pe4k35LAfvQg79ieutGF%2B%2FhXZKw%2FR7E3LOsdmQ2JArfAoykh6UqSv%2F7Pn9shV8cOCpp8fGLI7iwW3EKXBWFW276XLTXX%2BQ0iVduQBaP3wOmp6ivj7HXyQK5S3EsZhTsaey5d5MGR8phmcd4OUvVql5g%2FwduEQagOu9YgC2luvPts5f%2FpM9xJsHIyeUfc4sgyySRMVpn60g8hiLx98zhddR5njbqddSlk4Puw3Daw6GZvhjX0kzDeqEdkfWFLpLkT1s75dxYvrRACajg9i8zBnDhKjhErFNCCjU9K7wX2oeVuzhoiGc%2BjUiaYNN5Ed8exr8gDvdIPkB2%2FTkXobuJGkplWw8cAHfXrja0G2H1N85zKY2Hguq8b5D5ygfOVj04sxMJPOAaxsKfX2%2FAk5fA2Xggjj6lGHf5%2Bs%2B3nWLyjQBbzpEIbuPhLjRjeB3hgvOGPoUJT7HOYiDaxNs%2BcAiNVpPsoDVqD4OcDnkYp2rQ98d0BAMER6KdcO0w8h%2B42DYlJU1wUBM%2B2DQOKODBMMKu5ccGOrEB2E%2B%2Btt27OC0T%2FH1SrMMUHUPihh8psSp963zWt0pBV4rDpkPpxOASucP3wiH2u%2FzWpazVjy76S7wOPOes5dIGOdZQDdvYSOvKUH8ajCfyv1v%2FxMQ2kIpdttQ7GQw3IwMBq3tO35P6XZWe3yJykJKhrRwueJyh7yjN2maA3J4%2B%2BexrcwWBbJYIlpe55wERs1moPjO9%2BTAV6XAtTwGbnEzAk6%2FSgnq74gwgJXIxjOFn7Pw7&X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Date=20251022T230807Z&X-Amz-SignedHeaders=host&X-Amz-Expires=300&X-Amz-Credential=ASIAQ3PHCVTYSZTATCXI%2F20251022%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Signature=6eca94c9a375c35114949e84bf7e4720d877ba1f28d08537334a6a1c85a1f2e6&hash=ab082a44862373e9148b7a8790c4660833b6f3cd6830c4160f3fc493f256125e&host=68042c943591013ac2b2430a89b270f6af2c76d8dfd086a07176afe7c76c2c61&pii=S0019057823000769&tid=spdf-ffcfded1-a0d2-4cc3-a336-2626540b85b4&sid=29ac4d2c69581448154ac199593a2f901eedgxrqa&type=client&tsoh=d3d3LnNjaWVuY2VkaXJlY3QuY29t&rh=d3d3LnNjaWVuY2VkaXJlY3QuY29t&ua=1b145f5d5106055157&rr=992ca79edb09d84c&cc=us
# https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1087247&tag=1

# We'll implement a 2D Potential Field testbed that mirrors the common equations in your C++ headers:
# - Attractive force: F_att = K_att * (goal - pos)
# - Repulsive force (per obstacle): F_rep = K_rep * (1/d - 1/infl) * (1/d^2) * direction, for d < infl; else 0
# The "massless" assumption maps force directly to velocity via a gain (linearGain), with explicit
# velocity and acceleration constraints enforced at each step.
#
# The module includes:
# - PotentialField2D class with methods:
#     * add_circle / add_box for obstacles
#     * plan_path(start, goal, dt, max_steps, ...) -> dict with states and diagnostics
#     * simulate(...) : real-time matplotlib animation
# - Obstacle classes for circle/box with distance & gradient
#
# After defining the module, we'll run a small demo with one circular and one box obstacle.
from dataclasses import dataclass
import numpy as np
import math
import time
from typing import List, Tuple, Optional, Dict, Any, Callable

import matplotlib.pyplot as plt
from matplotlib import animation


@dataclass
class CircleObstacle:
    cx: float  # Center X coordinate
    cy: float  # Center Y coordinate
    radius: float  # Radius of the circle
    influence_distance: float = 1.0  # Distance from surface to influence zone boundary

    def distance_and_direction(self, p: np.ndarray) -> Tuple[float, np.ndarray]:
        """Signed distance from point to circle boundary (outside positive), and outward unit direction from center->p."""
        v = p - np.array([self.cx, self.cy], dtype=float)
        dist_center = np.linalg.norm(v)
        if dist_center == 0.0:
            # Arbitrary outward
            return self.radius, np.array([1.0, 0.0])
        direction = v / dist_center
        signed = dist_center - self.radius
        return signed, direction


@dataclass
class BoxObstacle:
    cx: float  # Center X coordinate
    cy: float  # Center Y coordinate
    half_w: float  # Half-width (along local x)
    half_h: float  # Half-height (along local y)
    theta: float = 0.0  # Rotation angle in radians
    influence_distance: float = 1.0  # Distance from surface to influence zone boundary

    def _world_to_local(self, p: np.ndarray) -> np.ndarray:
        c, s = math.cos(-self.theta), math.sin(-self.theta)
        R = np.array([[c, -s], [s, c]])
        return R @ (p - np.array([self.cx, self.cy]))

    def _local_to_world(self, v: np.ndarray) -> np.ndarray:
        c, s = math.cos(self.theta), math.sin(self.theta)
        R = np.array([[c, -s], [s, c]])
        return R @ v

    def distance_and_direction(self, p: np.ndarray) -> Tuple[float, np.ndarray]:
        """Signed distance to rectangle boundary (positive outside). Also returns outward unit normal direction."""
        pl = self._world_to_local(p)
        dx = abs(pl[0]) - self.half_w
        dy = abs(pl[1]) - self.half_h
        # Outside distance to rectangle
        outside_dx = max(dx, 0.0)
        outside_dy = max(dy, 0.0)
        outside_dist = math.hypot(outside_dx, outside_dy)
        inside_dist = min(max(dx, dy), 0.0)
        signed = outside_dist + inside_dist  # positive outside, negative inside

        # Approximate outward normal
        if signed > 0:  # outside: gradient points from nearest face/corner
            if outside_dx > 0 and outside_dy > 0:
                # corner normal
                n_local = np.array([outside_dx, outside_dy])
                n_local = n_local / (np.linalg.norm(n_local) + 1e-12)
            elif outside_dx > outside_dy:
                n_local = np.array([math.copysign(1.0, pl[0]), 0.0])
            else:
                n_local = np.array([0.0, math.copysign(1.0, pl[1])])
        else:
            # inside: normal points toward outside via nearest face
            if abs(dx) > abs(dy):
                n_local = np.array([math.copysign(1.0, pl[0]), 0.0])
            else:
                n_local = np.array([0.0, math.copysign(1.0, pl[1])])
        n_world = self._local_to_world(n_local)
        n_world = n_world / (np.linalg.norm(n_world) + 1e-12)
        return signed, n_world


class PotentialField2D:
    """
    2D Potential Field testbed using equations aligned with your C++ headers:
      - Attractive force:   F_att = K_att * (g - p)
      - Repulsive (per obs): if d < d_inf:
             F_rep = K_rep * (1/d - 1/d_inf) * (1/d^2) * n_hat
        else: 0
      where d is the distance from the point to the obstacle boundary (>=0 outside, <0 inside);
      n_hat is outward normal from obstacle toward the query point.
    Velocity is derived from force with a linear gain (massless model), then clipped to motion constraints.
    """

    def __init__(self,
                 attractive_gain: float = 1.0,
                 repulsive_gain: float = 1.0,
                 linear_gain: float = 1.0,
                 max_speed: float = 1.0,
                 max_accel: float = 2.0,
                 damping: float = 0.0):
        self.attractive_gain = float(attractive_gain)
        self.repulsive_gain = float(repulsive_gain)
        self.linear_gain = float(linear_gain)
        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)
        self.damping = float(damping)

        self._circles: List[CircleObstacle] = []
        self._boxes: List[BoxObstacle] = []

    # ----- Obstacles -----
    def add_circle(self, cx: float, cy: float, radius: float, influence_distance: float = 2.0):
        self._circles.append(CircleObstacle(
            cx, cy, radius, influence_distance))

    def add_box(self, cx: float, cy: float, w: float, h: float, theta: float = 0.0, influence_distance: float = 1.5):
        self._boxes.append(BoxObstacle(
            cx, cy, 0.5 * w, 0.5 * h, theta, influence_distance))

    # ----- Core Field -----
    def attractive_force(self, p: np.ndarray, g: np.ndarray) -> np.ndarray:
        return self.attractive_gain * (g - p)

    def repulsive_force(self, p: np.ndarray) -> np.ndarray:
        total = np.zeros(2, dtype=float)
        # Circles
        for c in self._circles:
            d, n_hat = c.distance_and_direction(p)
            d_inf = c.influence_distance
            if d < d_inf:
                # For points inside (d<0), clamp d to a small positive to avoid singularity, but keep direction
                d_eff = max(d, 1e-6)
                coeff = self.repulsive_gain * \
                    (1.0 / d_eff - 1.0 / d_inf) * (1.0 / (d_eff * d_eff))
                total += coeff * n_hat
        # Boxes
        for b in self._boxes:
            d, n_hat = b.distance_and_direction(p)
            d_inf = b.influence_distance
            if d < d_inf:
                d_eff = max(d, 1e-6)
                coeff = self.repulsive_gain * \
                    (1.0 / d_eff - 1.0 / d_inf) * (1.0 / (d_eff * d_eff))
                total += coeff * n_hat
        return total

    def total_force(self, p: np.ndarray, g: np.ndarray, v: np.ndarray) -> np.ndarray:
        return self.attractive_force(p, g) + self.repulsive_force(p) - self.damping * v

    # ----- Motion constraints -----
    @staticmethod
    def _clip_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
        n = np.linalg.norm(v)
        if n <= max_norm or max_norm <= 0:
            return v
        return v * (max_norm / (n + 1e-12))

    def _apply_motion_limits(self, v_cmd: np.ndarray, v_prev: np.ndarray, dt: float) -> np.ndarray:
        # Velocity cap
        v = self._clip_norm(
            v_cmd, self.max_speed if self.max_speed > 0 else np.inf)
        # Acceleration cap
        a = (v - v_prev) / max(dt, 1e-6)
        a = self._clip_norm(
            a, self.max_accel if self.max_accel > 0 else np.inf)
        v_limited = v_prev + a * dt
        # Re-apply velocity cap in case of numeric issues
        v_limited = self._clip_norm(
            v_limited, self.max_speed if self.max_speed > 0 else np.inf)
        return v_limited

    # ----- Planner (massless integration of field) -----
    def plan_path(self,
                  start: Tuple[float, float],
                  goal: Tuple[float, float],
                  dt: float = 0.02,
                  max_steps: int = 5000,
                  goal_tolerance: float = 0.05,
                  stagnation_window: int = 100,
                  stagnation_eps: float = 1e-4) -> Dict[str, Any]:
        p = np.array(start, dtype=float)
        g = np.array(goal, dtype=float)
        v_prev = np.zeros(2, dtype=float)

        xs, ys = [p[0]], [p[1]]
        vxs, vys = [0.0], [0.0]
        ts = [0.0]

        stagnation_buf: List[float] = []

        for k in range(1, max_steps + 1):
            F = self.total_force(p, g, v_prev)
            v_cmd = self.linear_gain * F
            v = self._apply_motion_limits(v_cmd, v_prev, dt)

            p = p + v * dt
            v_prev = v

            xs.append(p[0])
            ys.append(p[1])
            vxs.append(v[0])
            vys.append(v[1])
            ts.append(k * dt)

            # Goal check
            if np.linalg.norm(p - g) <= goal_tolerance:
                print("Goal reached at step", k)
                break

            # Stagnation detection
            stagnation_buf.append(np.linalg.norm(v))
            if len(stagnation_buf) > stagnation_window:
                stagnation_buf.pop(0)
                if np.mean(stagnation_buf) < stagnation_eps:
                    print("Planning stopped due to stagnation at step", k)
                    break

        return {
            "x": np.array(xs),
            "y": np.array(ys),
            "vx": np.array(vxs),
            "vy": np.array(vys),
            "t": np.array(ts),
            "goal_reached": np.linalg.norm(np.array([xs[-1], ys[-1]]) - g) <= goal_tolerance
        }

    # ----- Real-time simulation (matplotlib animation) -----
    def simulate(self,
                 start: Tuple[float, float],
                 goal: Tuple[float, float],
                 dt: float = 0.02,
                 max_steps: int = 5000,
                 goal_tolerance: float = 0.02):
        p = np.array(start, dtype=float)
        g = np.array(goal, dtype=float)
        v_prev = np.zeros(2, dtype=float)

        fig, ax = plt.subplots(figsize=(6, 6))
        ax.set_aspect('equal')
        # Auto limits based on obstacles + start/goal
        xs = [p[0], g[0]] + [c.cx for c in self._circles] + \
            [b.cx for b in self._boxes]
        ys = [p[1], g[1]] + [c.cy for c in self._circles] + \
            [b.cy for b in self._boxes]
        span_x = (min(xs) - 2.0, max(xs) + 2.0)
        span_y = (min(ys) - 2.0, max(ys) + 2.0)
        ax.set_xlim(span_x)
        ax.set_ylim(span_y)

        # Draw obstacles
        for c in self._circles:
            # Obstacle boundary
            circ = plt.Circle((c.cx, c.cy), c.radius, fill=False)
            ax.add_patch(circ)
            # Influence boundary: at (radius + influence_distance) from center
            inf_radius = c.radius + c.influence_distance
            inf = plt.Circle((c.cx, c.cy), inf_radius,
                             linestyle='--', fill=False)
            ax.add_patch(inf)
        for b in self._boxes:
            # corners in local frame
            corners = np.array([[+b.half_w, +b.half_h],
                                [+b.half_w, -b.half_h],
                                [-b.half_w, -b.half_h],
                                [-b.half_w, +b.half_h]])
            c, s = np.cos(b.theta), np.sin(b.theta)
            R = np.array([[c, -s], [s,  c]])
            world = (R @ corners.T).T + np.array([b.cx, b.cy])
            ax.add_patch(plt.Polygon(world, fill=False))

            # influence rectangle (inflate by influence_distance then same transform)
            inf_corners = np.array([[b.half_w + b.influence_distance,  b.half_h + b.influence_distance],
                                    [b.half_w + b.influence_distance, -
                                        b.half_h - b.influence_distance],
                                    [-b.half_w - b.influence_distance, -
                                        b.half_h - b.influence_distance],
                                    [-b.half_w - b.influence_distance,  b.half_h + b.influence_distance]])
            world_inf = (R @ inf_corners.T).T + np.array([b.cx, b.cy])
            ax.add_patch(plt.Polygon(world_inf, fill=False, linestyle='--'))

        # Start/goal markers
        start_scatter = ax.scatter([start[0]], [start[1]])
        goal_scatter = ax.scatter([goal[0]], [goal[1]])

        traj_line, = ax.plot([], [])
        agent_dot, = ax.plot([], [], marker='o')

        path_x, path_y = [p[0]], [p[1]]

        def init():
            traj_line.set_data([], [])
            agent_dot.set_data([p[0]], [p[1]])
            return traj_line, agent_dot

        def update(frame):
            nonlocal p, v_prev
            F = self.total_force(p, g, v_prev)
            v_cmd = self.linear_gain * F
            v = self._apply_motion_limits(v_cmd, v_prev, dt)
            p = p + v * dt
            v_prev = v

            path_x.append(p[0])
            path_y.append(p[1])

            traj_line.set_data(path_x, path_y)
            agent_dot.set_data([p[0]], [p[1]])
            # stop if goal
            if np.linalg.norm(p - g) <= goal_tolerance:
                anim.event_source.stop()
            return traj_line, agent_dot

        anim = animation.FuncAnimation(
            fig, update, init_func=init, frames=max_steps, interval=dt*1000, blit=True)
        plt.show()


# ======= Demo =======
def _demo():
    pf = PotentialField2D(
        attractive_gain=1.5,
        repulsive_gain=0.2,
        linear_gain=1.0,
        max_speed=1.2,
        max_accel=3.0,
        damping=0.0
    )
    # Obstacles
    pf.add_circle(cx=2.0, cy=1.5, radius=0.8, influence_distance=1.0)
    pf.add_box(cx=3.5, cy=-0.7, w=1.0, h=2.0,
               theta=np.radians(20.0),
               influence_distance=1.2)

    start = (-2.0, -1.5)
    goal = (4.2, 1.0)

    # Plan (non-animated) to inspect data quickly
    delta_time = 0.05
    goal_tolerance = 0.075
    max_iters = 4000
    result = pf.plan_path(
        start, goal, dt=delta_time,
        goal_tolerance=goal_tolerance,
        max_steps=max_iters
    )
    # Quick printout
    print("Goal reached:", result["goal_reached"], " Duration:", result["t"][-1], "s",
          " End:", (float(result["x"][-1]), float(result["y"][-1])))

    # Live simulation
    pf.simulate(
        start, goal, dt=delta_time,
        goal_tolerance=goal_tolerance,
        max_steps=max_iters
    )


if __name__ == "__main__":
    _demo()
