#!/usr/bin/env python3
"""
Co-simulation node using dtTileCache for dynamic obstacles.

Unlike simulate_crowd_cosim.py which represents LiDAR foreground points as
zero-velocity crowd agents, this node clusters the foreground cloud into
voxel-connected-component blobs, fits a bounding cylinder to each cluster,
and inserts the cylinders as dtTileCache obstacles. Only tiles that overlap
changed obstacles are rebaked per frame.

Requires a .tcbin file (tiled navmesh + tile cache) instead of a solo .bin.
Bake one with:
    python -c "
    from voxnav.navmesh import TileCache
    from voxnav._ctypes_bindings import nmBuildSettings
    s = nmBuildSettings(cellSize=0.1, cellHeight=0.1, agentHeight=1.8,
                        agentRadius=0.3, agentMaxClimb=0.2, agentMaxSlope=45,
                        regionMinSize=8, regionMergeSize=20,
                        edgeMaxLen=12, edgeMaxError=1.3, vertsPerPoly=6,
                        detailSampleDist=6, detailSampleMaxError=1,
                        partitionType=0,
                        filterLowHangingObstacles=1, filterLedgeSpans=1,
                        filterWalkableLowHeightSpans=1)
    tc = TileCache(obj_path='scene_yup.obj', settings=s, tile_size=32, max_obstacles=128)
    tc.save('scene.tcbin')
    "
"""

import time
import argparse
import os
import sys
import numpy as np
import open3d as o3d
import tkinter as tk
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from tf_transformations import euler_from_quaternion

try:
    from voxnav.navmesh import NavMesh, NavMeshQuery, Crowd, TileCache
except ImportError:
    _pkg_dir = os.path.dirname(os.path.abspath(__file__))
    if _pkg_dir not in sys.path:
        sys.path.insert(0, _pkg_dir)
    from navmesh import NavMesh, NavMeshQuery, Crowd, TileCache


def _pointcloud2_to_xyz(msg):
    """Parse a PointCloud2 message into an Nx3 float32 numpy array (Z-up)."""
    n = msg.width * msg.height
    if n == 0:
        return None
    try:
        fields = {f.name: f.offset for f in msg.fields}
        x_off, y_off, z_off = fields['x'], fields['y'], fields['z']
    except KeyError:
        return None
    ps = msg.point_step
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, ps)
    x = np.frombuffer(data[:, x_off:x_off + 4].tobytes(), dtype=np.float32)
    y = np.frombuffer(data[:, y_off:y_off + 4].tobytes(), dtype=np.float32)
    z = np.frombuffer(data[:, z_off:z_off + 4].tobytes(), dtype=np.float32)
    pts = np.column_stack([x, y, z])
    valid = np.isfinite(pts).all(axis=1)
    return pts[valid] if valid.any() else None


class TileCacheObstacleManager:
    """Clusters a live foreground point cloud into cylinder obstacles in the tile cache.

    Algorithm per frame:
      1. Voxelise the foreground cloud at `voxel_size` resolution.
      2. Find 3-D connected components of occupied voxels via BFS.
      3. Discard clusters with fewer than `min_cluster_voxels` voxels.
      4. Per cluster: compute XY centroid, max-radius from centroid + padding,
         Z bottom from minimum point Z minus half of height_padding.
      5. Diff against previous frame's cylinders by proximity matching.
      6. Remove disappeared / significantly changed cylinders; add new ones.
      7. Drain tile cache rebuild queue (up to `max_update_iters` iterations).
    """

    def __init__(
        self,
        tile_cache: TileCache,
        voxel_size: float = 0.3,
        min_cluster_voxels: int = 3,
        cylinder_padding: float = 0.15,
        height_padding: float = 0.20,
        max_obstacles: int = 64,
        max_update_iters: int = 20,
        decay_s: float = 1.5,
    ):
        self._tc = tile_cache
        self._voxel = float(voxel_size)
        self._min_vox = int(min_cluster_voxels)
        self._cpad = float(cylinder_padding)
        self._hpad = float(height_padding)
        self._max = int(max_obstacles)
        self._max_iters = int(max_update_iters)
        self._decay_s = float(decay_s)
        # Maps obstacle_ref (int) -> (cx, cy, cz_center, radius, height)  — Z-up
        self._active: dict[int, tuple] = {}
        self._last_seen: dict[int, float] = {}
        self._pending_rebuild: bool = False

    def _cluster_points(self, pts_zup: np.ndarray) -> list[tuple]:
        """Return list of (cx, cy, cz_center, radius, height) in Z-up coords."""
        if len(pts_zup) == 0:
            return []

        bmin = pts_zup.min(axis=0)
        # Map each point to a voxel index tuple
        vox_idx = np.floor((pts_zup - bmin) / self._voxel).astype(np.int32)
        dims = vox_idx.max(axis=0) + 1  # (Nx, Ny, Nz)

        # Build occupancy set and voxel → point-indices map
        occ_set: dict[tuple, list[int]] = {}
        for i, vi in enumerate(vox_idx):
            key = (int(vi[0]), int(vi[1]), int(vi[2]))
            occ_set.setdefault(key, []).append(i)

        # BFS connected components (6-connectivity)
        visited = set()
        cylinders = []
        DIRS = [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)]

        for seed in occ_set:
            if seed in visited:
                continue
            # BFS
            component_voxels = []
            queue = [seed]
            visited.add(seed)
            while queue:
                cur = queue.pop()
                component_voxels.append(cur)
                for dx, dy, dz in DIRS:
                    nb = (cur[0]+dx, cur[1]+dy, cur[2]+dz)
                    if nb not in visited and nb in occ_set:
                        visited.add(nb)
                        queue.append(nb)

            if len(component_voxels) < self._min_vox:
                continue

            # Collect original points in this cluster
            point_indices = []
            for vk in component_voxels:
                point_indices.extend(occ_set[vk])
            cluster_pts = pts_zup[point_indices]

            centroid_xy = cluster_pts[:, :2].mean(axis=0)
            dists_xy = np.linalg.norm(cluster_pts[:, :2] - centroid_xy, axis=1)
            radius = float(dists_xy.max()) + self._cpad

            if radius < 0.05:
                continue

            z_min = float(cluster_pts[:, 2].min())
            z_max = float(cluster_pts[:, 2].max())
            height = (z_max - z_min) + self._hpad
            cz_center = (z_min + z_max) / 2.0

            cylinders.append((
                float(centroid_xy[0]),
                float(centroid_xy[1]),
                cz_center,
                radius,
                height,
            ))

        return cylinders

    def update(self, pts_zup, dt: float) -> bool:
        """Process a new point cloud frame and update tile cache obstacles."""
        if pts_zup is None or len(pts_zup) == 0:
            now = time.time()
            expired_refs = [r for r, last in self._last_seen.items() if now - last > self._decay_s]
            for ref in expired_refs:
                self._tc.remove_obstacle(ref)
                del self._active[ref]
                del self._last_seen[ref]
                self._pending_rebuild = True

            tiles_settled = self._drain(dt) and self._pending_rebuild
            if tiles_settled:
                self._pending_rebuild = False
            return tiles_settled

        new_cylinders = self._cluster_points(pts_zup)
        if len(new_cylinders) > self._max:
            new_cylinders = new_cylinders[:self._max]

        # Diff: match new cylinders to active ones by proximity
        matched_new = set()     # indices into new_cylinders that matched
        matched_active = set()  # refs in self._active that matched
        to_readd = []           # (ref, new_cyl) pairs where geometry changed enough

        for ref, (acx, acy, acz, ar, ah) in list(self._active.items()):
            best_dist = float('inf')
            best_idx = -1
            for ni, (ncx, ncy, ncz, nr, nh) in enumerate(new_cylinders):
                if ni in matched_new:
                    continue
                d = math.sqrt((acx - ncx)**2 + (acy - ncy)**2)
                if d < max(ar, nr) and d < best_dist:
                    best_dist = d
                    best_idx = ni

            if best_idx >= 0:
                matched_active.add(ref)
                matched_new.add(best_idx)
                self._last_seen[ref] = time.time()
                # Check if geometry changed significantly (>10%)
                ncx, ncy, ncz, nr, nh = new_cylinders[best_idx]
                if abs(nr - ar) / max(ar, 1e-4) > 0.1 or abs(nh - ah) / max(ah, 1e-4) > 0.1:
                    to_readd.append((ref, new_cylinders[best_idx]))

        # Remove stale obstacles
        now = time.time()
        stale_refs = [r for r in list(self._active.keys()) if r not in matched_active and now - self._last_seen.get(r, 0) > self._decay_s]
        for ref in stale_refs:
            self._tc.remove_obstacle(ref)
            del self._active[ref]
            if ref in self._last_seen:
                del self._last_seen[ref]
            self._pending_rebuild = True

        # Remove and re-add changed geometry
        for ref, new_cyl in to_readd:
            self._tc.remove_obstacle(ref)
            del self._active[ref]
            if ref in self._last_seen:
                del self._last_seen[ref]
            ncx, ncy, ncz, nr, nh = new_cyl
            new_ref = self._tc.add_cylinder((ncx, ncy, ncz), nr, nh)
            self._pending_rebuild = True
            if new_ref:
                self._active[new_ref] = new_cyl
                self._last_seen[new_ref] = time.time()

        # Add genuinely new cylinders
        for ni, cyl in enumerate(new_cylinders):
            if ni in matched_new:
                continue
            cx, cy, cz, r, h = cyl
            ref = self._tc.add_cylinder((cx, cy, cz), r, h)
            self._pending_rebuild = True
            if ref:
                self._active[ref] = cyl
                self._last_seen[ref] = time.time()

        tiles_settled = self._drain(dt) and self._pending_rebuild
        if tiles_settled:
            self._pending_rebuild = False
        return tiles_settled

    def _drain(self, dt: float) -> bool:
        """Drain the tile cache rebuild queue, capped at max_update_iters."""
        for _ in range(self._max_iters):
            if self._tc.update(dt):
                return True
        return False

    def clear(self) -> None:
        """Remove all active obstacles from the tile cache."""
        for ref in list(self._active.keys()):
            self._tc.remove_obstacle(ref)
        self._active.clear()
        self._last_seen.clear()
        self._pending_rebuild = True
        self._drain(0.0)

    @property
    def count(self) -> int:
        return len(self._active)

    @property
    def active_positions(self) -> list:
        """Z-up centroid positions of active cylinders."""
        return [(cx, cy, cz) for cx, cy, cz, _, _ in self._active.values()]


class ObstacleCosimNode(Node):
    """ROS2 node: robot steering via Recast crowd + dynamic obstacles via dtTileCache."""

    def __init__(self, tcbin_file, obj_file=None):
        super().__init__('obstacle_cosim_node')

        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('snap_vextent', -1.0)
        self.declare_parameter('dyn_obstacle_source', 'cloud')   # 'cloud' | 'none'
        self.declare_parameter('robot_radius', 0.3)
        # TileCache-specific params
        self.declare_parameter('cluster_voxel_m', 0.3)
        self.declare_parameter('cluster_min_voxels', 3)
        self.declare_parameter('cylinder_padding', 0.15)
        self.declare_parameter('height_padding', 0.20)
        self.declare_parameter('max_tc_obstacles', 64)
        self.declare_parameter('obstacle_decay_s', 1.5)

        self.max_linear_speed  = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.update_rate       = self.get_parameter('update_rate').value
        self.kp_angular        = self.get_parameter('kp_angular').value
        _vext = self.get_parameter('snap_vextent').value
        self.snap_vextent: float | None = _vext if _vext > 0 else None
        self.dyn_source        = self.get_parameter('dyn_obstacle_source').value
        self.robot_radius      = float(self.get_parameter('robot_radius').value)
        self.cluster_voxel    = float(self.get_parameter('cluster_voxel_m').value)
        self.cluster_min_vox  = int(self.get_parameter('cluster_min_voxels').value)
        self.cyl_pad          = float(self.get_parameter('cylinder_padding').value)
        self.height_pad       = float(self.get_parameter('height_padding').value)
        self.max_tc_obs       = int(self.get_parameter('max_tc_obstacles').value)
        self.obstacle_decay_s = float(self.get_parameter('obstacle_decay_s').value)

        self.state_lock = threading.Lock()
        self.state = {
            "run": True,
            "obs_x": 0.0,
            "obs_y": 0.0,
            "obs_z": 0.0,
            "target_x": 0.0,
            "target_y": 0.0,
            "target_z": 0.0,
            "force_target": False,
            "request_move": False,
            "robot_pose": None,
            "robot_pos": None,
            "robot_heading": 0.0,
            "dyn_points": None,
            "dyn_obs_positions": [],
            "robot_vel": None,
            "navmesh_geom": None,
        }

        self.tcbin_file = tcbin_file
        self.obj_file   = obj_file
        self.tc         = None   # TileCache
        self.nm         = None   # NavMeshRaw (non-owning)
        self.nmq        = None
        self.crowd      = None
        self.robot_id   = None
        self.gui_obs_ref = 0    # tile cache ref for the GUI obstacle cylinder
        self.dyn_obs_mgr = None

        self.vis = None
        self.last_time = time.time()

        self.robot_pose_sub = self.create_subscription(
            Odometry, '/robotPose', self.robot_pose_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        if self.dyn_source == 'cloud':
            self._cloud_sub = self.create_subscription(
                PointCloud2, '/foreground_cloud', self.foreground_cloud_callback, 10)
            self.get_logger().info('Subscribed to /foreground_cloud for tile cache obstacles.')
        else:
            self.get_logger().info('Dynamic obstacles disabled (dyn_obstacle_source=none).')

        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.update_simulation)
        self.get_logger().info('ObstacleCosimNode initialized.')

    def robot_pose_callback(self, msg):
        with self.state_lock:
            self.state["robot_pose"] = msg
            pos = msg.pose.pose.position
            self.state["robot_pos"] = np.array([pos.x, pos.y, pos.z])
            orientation = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w])
            self.state["robot_heading"] = yaw

    def foreground_cloud_callback(self, msg):
        pts = _pointcloud2_to_xyz(msg)
        with self.state_lock:
            self.state["dyn_points"] = pts

    def update_simulation(self):
        if self.crowd is None or self.nm is None:
            return

        with self.state_lock:
            if not self.state["run"]:
                return

            if self.state["robot_pos"] is not None:
                self.crowd.sync_agent_pos(self.robot_id, self.state["robot_pos"],
                                          self.snap_vextent)

            # Update GUI obstacle as a tile cache cylinder
            new_obs_pos = np.array([
                self.state["obs_x"], self.state["obs_y"], self.state["obs_z"]])
            if self.state.get("obs_moved", False):
                self.state["obs_moved"] = False
                if self.gui_obs_ref:
                    self.tc.remove_obstacle(self.gui_obs_ref)
                self.gui_obs_ref = self.tc.add_cylinder(new_obs_pos, 0.6, 1.0)

            if self.state["force_target"]:
                req_pos = np.array([
                    self.state["target_x"],
                    self.state["target_y"],
                    self.state["target_z"],
                ])
                ref, end_snp = self.nmq.find_nearest_poly(req_pos)
                if ref > 0:
                    end_pos = end_snp
                    self.state["target_x"] = float(end_snp[0])
                    self.state["target_y"] = float(end_snp[1])
                    self.state["target_z"] = float(end_snp[2])
                else:
                    end_pos = req_pos
                self.crowd.request_move_target(self.robot_id, end_pos, self.nmq)
                self.state["force_target"] = False
                self.get_logger().info(f'New target: {np.round(end_pos, 3)}')

            if self.state["request_move"]:
                target_pos = np.array([
                    self.state["target_x"],
                    self.state["target_y"],
                    self.state["target_z"],
                ])
                if not self.crowd.request_move_target(self.robot_id, target_pos, self.nmq):
                    self.get_logger().warn('Move failed: target not on navmesh.')
                else:
                    self.get_logger().info(f'Move requested to: {np.round(target_pos, 3)}')
                self.state["request_move"] = False

            dyn_pts = self.state["dyn_points"]
            self.state["dyn_points"] = None

        # Update dynamic tile cache obstacles from LiDAR
        if self.dyn_obs_mgr is not None:
            current_time = time.time()
            dt_for_drain = max(0.001, time.time() - self.last_time)
            tiles_settled = self.dyn_obs_mgr.update(dyn_pts, dt_for_drain)
            with self.state_lock:
                self.state["dyn_obs_positions"] = self.dyn_obs_mgr.active_positions
            if tiles_settled:
                verts, tris = self.nm.get_geometry(swap_yz=True)
                with self.state_lock:
                    self.state["navmesh_geom"] = (verts, tris)
                    target_pos = np.array([self.state["target_x"],
                                            self.state["target_y"],
                                            self.state["target_z"]])
                self.crowd.request_move_target(self.robot_id, target_pos, self.nmq)

        # Compute dt and step crowd simulation
        current_time = time.time()
        dt = min(current_time - self.last_time, 0.1)
        self.last_time = current_time

        self.crowd.update(dt)

        pos, vel = self.crowd.get_agent_pos(self.robot_id)
        if vel is not None and pos is not None:
            with self.state_lock:
                self.state["robot_vel"] = vel
            self.cmd_vel_pub.publish(self.compute_cmd_vel(vel))

    def compute_cmd_vel(self, recast_vel):
        cmd_vel = Twist()
        if recast_vel is None or np.linalg.norm(recast_vel) < 1e-4:
            return cmd_vel

        power_factor = 4.0

        with self.state_lock:
            current_heading = self.state["robot_heading"]

        vel_x, vel_y, _ = recast_vel
        desired_heading = math.atan2(vel_y, vel_x)
        angular_error = self.normalize_angle(desired_heading - current_heading)

        angular_vel = self.kp_angular * angular_error
        angular_vel = np.clip(angular_vel, -self.max_angular_speed, self.max_angular_speed)

        speed = min(math.sqrt(vel_x**2 + vel_y**2), self.max_linear_speed)
        alignment = max(0.0, math.cos(angular_error))
        linear_vel = speed * math.pow(alignment, power_factor)

        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        return cmd_vel

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def initialize_recast(self, bmin, bmax):
        self.tc  = TileCache(tcbin_path=self.tcbin_file)
        self.nm  = self.tc.navmesh
        self.nmq = NavMeshQuery(self.nm)
        # Only robot agent in crowd (no obstacle agents)
        self.crowd = Crowd(self.nm, max_agents=2, max_agent_radius=0.5)

        # Set up robot avoidance profile
        robot_profile = self.crowd.get_obstacle_avoidance_params(3)
        if robot_profile is not None:
            robot_profile.horizTime    = 2.5
            robot_profile.weightToi    = 2.0
            robot_profile.weightDesVel = 2.8
            robot_profile.weightSide   = 1.1
            robot_profile.weightCurVel = 2.5
            self.crowd.set_obstacle_avoidance_params(4, robot_profile)

        start_pos = np.array([-4.0, 0.0, 0.0])
        end_pos   = np.array([4.0,  0.0, 0.0])

        verts, _ = self.nm.get_geometry(swap_yz=True)
        if len(verts) > 0:
            ref, start_snp = self.nmq.find_nearest_poly(bmin + np.array([1, 1, 0]))
            ref2, end_snp  = self.nmq.find_nearest_poly(bmax - np.array([1, 1, 0]))
            if ref > 0 and ref2 > 0:
                start_pos = start_snp
                end_pos   = end_snp

        with self.state_lock:
            self.state["target_x"] = end_pos[0]
            self.state["target_y"] = end_pos[1]
            self.state["target_z"] = end_pos[2]

        self.robot_id = self.crowd.add_agent(
            start_pos,
            radius=self.robot_radius,
            maxAcceleration=12.0,
            maxSpeed=float(self.max_linear_speed),
            collisionQueryRange=12.0,
            separationWeight=1.0,
            obstacleWeight=2.0,
            updateFlags=1 | 2 | 4 | 8 | 16,
            obstacleAvoidanceType=3,
        )

        if self.dyn_source == 'cloud':
            self.dyn_obs_mgr = TileCacheObstacleManager(
                self.tc,
                voxel_size=self.cluster_voxel,
                min_cluster_voxels=self.cluster_min_vox,
                cylinder_padding=self.cyl_pad,
                height_padding=self.height_pad,
                max_obstacles=self.max_tc_obs,
                decay_s=self.obstacle_decay_s,
            )

        self.get_logger().info('Recast tiled navmesh initialized.')
        self.get_logger().info(f'Robot agent ID: {self.robot_id}')
        self.get_logger().info(
            f'TileCache obstacle approach: source={self.dyn_source}, '
            f'voxel={self.cluster_voxel}m, max_tc={self.max_tc_obs}')
        return end_pos

    def shutdown(self):
        with self.state_lock:
            self.state["run"] = False

        if self.dyn_obs_mgr is not None:
            self.dyn_obs_mgr.clear()

        if self.gui_obs_ref:
            self.tc.remove_obstacle(self.gui_obs_ref)
            self.gui_obs_ref = 0

        if self.crowd is not None:
            self.crowd.close()
        if self.nmq is not None:
            self.nmq.close()
        # Do NOT close self.nm — its lifetime is owned by self.tc
        if self.tc is not None:
            self.tc.close()


def create_gui(node, bmin, bmax):
    root = tk.Tk()
    root.title("Obstacle Co-Simulation Control (TileCache)")
    root.geometry("380x350")

    with node.state_lock:
        x_var  = tk.DoubleVar(value=node.state["obs_x"])
        y_var  = tk.DoubleVar(value=node.state["obs_y"])
        z_var  = tk.DoubleVar(value=node.state["obs_z"])
        tx_var = tk.DoubleVar(value=node.state["target_x"])
        ty_var = tk.DoubleVar(value=node.state["target_y"])
        tz_var = tk.DoubleVar(value=node.state["target_z"])

    def _set_obs(key, val):
        with node.state_lock:
            node.state[key] = float(val)
            node.state["obs_moved"] = True

    def on_tx(val):
        with node.state_lock:
            node.state["target_x"] = float(val)
            node.state["force_target"] = True

    def on_ty(val):
        with node.state_lock:
            node.state["target_y"] = float(val)
            node.state["force_target"] = True

    def on_tz(val):
        with node.state_lock:
            node.state["target_z"] = float(val)
            node.state["force_target"] = True

    def on_start_move():
        with node.state_lock:
            node.state["request_move"] = True

    tk.Label(root, text="GUI Obstacle Position", font=('Arial', 10, 'bold')).pack(pady=(10, 5))
    tk.Scale(root, from_=bmin[0], to=bmax[0], resolution=0.1,
             orient=tk.HORIZONTAL, label="Obstacle X",
             variable=x_var, command=lambda v: _set_obs("obs_x", v)).pack(fill=tk.X, padx=10)
    tk.Scale(root, from_=bmin[1], to=bmax[1], resolution=0.1,
             orient=tk.HORIZONTAL, label="Obstacle Y",
             variable=y_var, command=lambda v: _set_obs("obs_y", v)).pack(fill=tk.X, padx=10)
    tk.Scale(root, from_=-50.0, to=50.0, resolution=0.1,
             orient=tk.HORIZONTAL, label="Obstacle Z",
             variable=z_var, command=lambda v: _set_obs("obs_z", v)).pack(fill=tk.X, padx=10)

    tk.Label(root, text="Target Position", font=('Arial', 10, 'bold')).pack(pady=(10, 5))
    tk.Scale(root, from_=bmin[0], to=bmax[0], resolution=0.1,
             orient=tk.HORIZONTAL, label="Target X",
             variable=tx_var, command=on_tx).pack(fill=tk.X, padx=10)
    tk.Scale(root, from_=bmin[1], to=bmax[1], resolution=0.1,
             orient=tk.HORIZONTAL, label="Target Y",
             variable=ty_var, command=on_ty).pack(fill=tk.X, padx=10)
    tk.Scale(root, from_=bmin[2], to=bmax[2], resolution=0.1,
             orient=tk.HORIZONTAL, label="Target Z",
             variable=tz_var, command=on_tz).pack(fill=tk.X, padx=10)

    tk.Button(root, text="Start Robot Movement",
              command=on_start_move).pack(fill=tk.X, padx=10, pady=(10, 4))

    def on_close():
        with node.state_lock:
            node.state["run"] = False
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    return root


def make_wireframe_from(verts, tris):
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices  = o3d.utility.Vector3dVector(verts.astype(np.float64))
    mesh.triangles = o3d.utility.Vector3iVector(tris.astype(np.int32))
    ls = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
    ls.paint_uniform_color((0.0, 0.6, 0.2))
    return ls


def make_wireframe(nm):
    verts, tris = nm.get_geometry(swap_yz=True)
    return make_wireframe_from(verts, tris)


def main():
    script_args = []
    ros_args    = []
    if '--ros-args' in sys.argv:
        idx = sys.argv.index('--ros-args')
        script_args = sys.argv[1:idx]
        ros_args    = sys.argv[idx:]
    else:
        script_args = sys.argv[1:]

    parser = argparse.ArgumentParser()
    parser.add_argument("tcbin", help="Tiled navmesh tile-cache .tcbin file")
    parser.add_argument("--obj", help="Optional OBJ file for visualization", default=None)
    args = parser.parse_args(script_args)

    rclpy.init(args=ros_args)
    node = ObstacleCosimNode(args.tcbin, args.obj)

    executor = None
    vis      = None
    root     = None

    try:
        # Load tile cache temporarily just to get navmesh geometry for bounds
        temp_tc = TileCache(tcbin_path=args.tcbin)
        verts, tris = temp_tc.navmesh.get_geometry(swap_yz=True)
        temp_tc.close()

        bmin = np.array([-20.0, -20.0, 0.0])
        bmax = np.array([20.0,  20.0,  0.0])
        if len(verts) > 0:
            bmin = np.min(verts, axis=0)
            bmax = np.max(verts, axis=0)
            with node.state_lock:
                node.state["obs_x"] = float((bmin[0] + bmax[0]) / 2)
                node.state["obs_y"] = float((bmin[1] + bmax[1]) / 2)
                node.state["obs_z"] = float((bmin[2] + bmax[2]) / 2)

        end_pos = node.initialize_recast(bmin, bmax)
        root = create_gui(node, bmin, bmax)

        vis = o3d.visualization.Visualizer()
        vis.create_window()

        pcd = o3d.geometry.PointCloud()
        if len(verts) > 0:
            pcd.points = o3d.utility.Vector3dVector(verts.astype(np.float64))
            pcd.paint_uniform_color([0.3, 0.3, 0.3])
            vis.add_geometry(pcd)

        nm_wire = make_wireframe(node.nm)
        vis.add_geometry(nm_wire)

        if args.obj:
            obj_mesh = o3d.io.read_triangle_mesh(args.obj)
            if not obj_mesh.has_vertex_normals():
                obj_mesh.compute_vertex_normals()
            vis.add_geometry(obj_mesh)

        robot_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)
        robot_mesh.paint_uniform_color([0.1, 0.4, 1.0])
        robot_mesh.compute_vertex_normals()
        vis.add_geometry(robot_mesh)

        target_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
        target_mesh.paint_uniform_color([0.0, 1.0, 0.0])
        target_mesh.translate(end_pos)
        target_mesh.compute_vertex_normals()
        vis.add_geometry(target_mesh)

        # GUI obstacle — shown as red cylinder (position mirrors tile cache cylinder)
        obs_mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=0.6, height=1.0)
        obs_mesh.paint_uniform_color([1.0, 0.2, 0.2])
        obs_mesh.compute_vertex_normals()
        vis.add_geometry(obs_mesh)

        vel_arrow_mesh = o3d.geometry.TriangleMesh.create_arrow()
        vel_arrow_mesh.paint_uniform_color([0.0, 1.0, 1.0])
        vel_arrow_mesh.compute_vertex_normals()
        vis.add_geometry(vel_arrow_mesh)

        # Dynamic obstacle pool: orange cylinders (one per active cluster)
        _HIDDEN = np.array([(bmin[0] + bmax[0]) / 2, (bmin[1] + bmax[1]) / 2, bmin[2] - 2.0])
        dyn_obs_meshes = []
        for _ in range(node.max_tc_obs):
            m = o3d.geometry.TriangleMesh.create_cylinder(radius=0.3, height=0.8)
            m.paint_uniform_color([1.0, 0.6, 0.0])
            m.compute_vertex_normals()
            m.translate(_HIDDEN)
            vis.add_geometry(m)
            dyn_obs_meshes.append(m)
        _dyn_base_verts = np.asarray(
            o3d.geometry.TriangleMesh.create_cylinder(radius=0.3, height=0.8).vertices).copy()

        node.get_logger().info("Co-simulation started. Waiting for /robotPose...")
        node.get_logger().info("GUI obstacle is injected as a tile-cache cylinder.")

        executor = MultiThreadedExecutor()
        executor.add_node(node)
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()

        while True:
            with node.state_lock:
                if not node.state["run"]:
                    break

            root.update_idletasks()
            root.update()

            if not vis.poll_events():
                break
            vis.update_renderer()

            with node.state_lock:
                robot_pos   = node.state["robot_pos"]
                robot_vel   = node.state["robot_vel"]
                target_pos  = np.array([node.state["target_x"],
                                        node.state["target_y"],
                                        node.state["target_z"]])
                obs_pos_viz = np.array([node.state["obs_x"],
                                        node.state["obs_y"],
                                        node.state["obs_z"]])
                dyn_positions = list(node.state["dyn_obs_positions"])
                navmesh_geom = node.state.pop("navmesh_geom", None)

            if navmesh_geom is not None:
                verts, tris = navmesh_geom
                vis.remove_geometry(nm_wire, reset_bounding_box=False)
                nm_wire = make_wireframe_from(verts, tris)
                vis.add_geometry(nm_wire, reset_bounding_box=False)

            if robot_pos is not None:
                robot_mesh.vertices = o3d.geometry.TriangleMesh.create_sphere(radius=0.3).vertices
                robot_mesh.translate(robot_pos)
                vis.update_geometry(robot_mesh)

            if robot_vel is not None and robot_pos is not None:
                speed = np.linalg.norm(robot_vel)
                if speed > 0.05:
                    _arrow = o3d.geometry.TriangleMesh.create_arrow(
                        cylinder_radius=0.05, cone_radius=0.1,
                        cylinder_height=speed, cone_height=0.2)
                    v_norm = robot_vel / speed
                    z_axis = np.array([0.0, 0.0, 1.0])
                    axis = np.cross(z_axis, v_norm)
                    axis_norm = np.linalg.norm(axis)
                    if axis_norm > 1e-6:
                        axis /= axis_norm
                        angle = np.arccos(np.clip(np.dot(z_axis, v_norm), -1.0, 1.0))
                        R = _arrow.get_rotation_matrix_from_axis_angle(axis * angle)
                        _arrow.rotate(R, center=(0, 0, 0))
                    elif np.dot(z_axis, v_norm) < 0:
                        R = _arrow.get_rotation_matrix_from_axis_angle(
                            np.array([1.0, 0.0, 0.0]) * np.pi)
                        _arrow.rotate(R, center=(0, 0, 0))
                    _arrow.translate(robot_pos)
                    vel_arrow_mesh.vertices = _arrow.vertices
                else:
                    _arrow = o3d.geometry.TriangleMesh.create_arrow()
                    _arrow.translate(_HIDDEN)
                    vel_arrow_mesh.vertices = _arrow.vertices
                vis.update_geometry(vel_arrow_mesh)

            target_mesh.vertices = o3d.geometry.TriangleMesh.create_sphere(radius=0.1).vertices
            target_mesh.translate(target_pos)
            vis.update_geometry(target_mesh)

            obs_mesh.vertices = o3d.geometry.TriangleMesh.create_cylinder(
                radius=0.6, height=1.0).vertices
            obs_mesh.translate(obs_pos_viz)
            vis.update_geometry(obs_mesh)

            for i, m in enumerate(dyn_obs_meshes):
                m.vertices = o3d.utility.Vector3dVector(_dyn_base_verts)
                pos = np.array(dyn_positions[i]) if i < len(dyn_positions) else _HIDDEN
                m.translate(pos)
                vis.update_geometry(m)

            time.sleep(1 / 60)

    finally:
        with node.state_lock:
            node.state["run"] = False

        if vis is not None:
            try: vis.destroy_window()
            except Exception: pass

        if root is not None:
            try: root.destroy()
            except Exception: pass

        node.shutdown()

        if executor is not None:
            executor.shutdown()

        rclpy.shutdown()
        node.get_logger().info("ObstacleCosimNode shut down.")


if __name__ == "__main__":
    main()
