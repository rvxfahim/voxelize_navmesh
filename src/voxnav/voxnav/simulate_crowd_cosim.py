#!/usr/bin/env python3
"""
Co-simulation node for real robot integration with Recast navigation.

Subscribes to /robotPose (nav_msgs/Odometry) for real robot position,
computes steering using Recast/Detour crowd simulation, and publishes
velocity commands to /cmd_vel (geometry_msgs/Twist).

GUI controls obstacle position and target destination.
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
from tf_transformations import euler_from_quaternion

# Try to import from the voxnav package
try:
    from voxnav.navmesh import NavMesh, NavMeshQuery, Crowd
except ImportError:
    _pkg_dir = os.path.dirname(os.path.abspath(__file__))
    if _pkg_dir not in sys.path:
        sys.path.insert(0, _pkg_dir)
    from navmesh import NavMesh, NavMeshQuery, Crowd


class CrowdCosimNode(Node):
    """ROS2 node for co-simulation with real robot."""
    
    def __init__(self, navmesh_file, obj_file=None):
        super().__init__('crowd_cosim_node')
        
        # Declare parameters
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('kp_angular', 2.0)
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.update_rate = self.get_parameter('update_rate').value
        self.kp_angular = self.get_parameter('kp_angular').value
        
        # Thread-safe state
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
            "robot_pose": None,  # Latest Odometry message
            "robot_pos": None,   # Extracted position [x, y, z]
            "robot_heading": 0.0  # Yaw angle
        }
        
        # Recast/Detour objects
        self.navmesh_file = navmesh_file
        self.obj_file = obj_file
        self.nm = None
        self.nmq = None
        self.crowd = None
        self.robot_id = None
        self.obs_id = None
        
        # Visualization
        self.vis = None
        self.robot_mesh = None
        self.target_mesh = None
        self.obs_mesh = None
        self.pcd = None
        self.nm_wire = None
        self.obj_mesh = None
        
        # Timing
        self.last_time = time.time()
        
        # ROS pub/sub
        self.robot_pose_sub = self.create_subscription(
            Odometry,
            '/robotPose',
            self.robot_pose_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Timer for simulation updates (30 Hz default)
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.update_simulation)
        
        self.get_logger().info(f'Co-simulation node initialized')
        self.get_logger().info(f'Max linear speed: {self.max_linear_speed} m/s')
        self.get_logger().info(f'Max angular speed: {self.max_angular_speed} rad/s')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
    
    def robot_pose_callback(self, msg):
        """Callback for /robotPose (Odometry)."""
        with self.state_lock:
            self.state["robot_pose"] = msg
            
            # Extract position
            pos = msg.pose.pose.position
            self.state["robot_pos"] = np.array([pos.x, pos.y, pos.z])
            
            # Extract heading (yaw) from quaternion
            orientation = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
            ])
            self.state["robot_heading"] = yaw
    
    def update_simulation(self):
        """Timer callback for simulation updates."""
        if self.crowd is None or self.nm is None:
            return
        
        with self.state_lock:
            if not self.state["run"]:
                return
            
            # Update robot position from /robotPose
            if self.state["robot_pos"] is not None:
                self.crowd.force_agent_pos(self.robot_id, self.state["robot_pos"])
            
            # Update obstacle position from GUI
            obs_pos = np.array([
                self.state["obs_x"],
                self.state["obs_y"],
                self.state["obs_z"]
            ])
            self.crowd.force_agent_pos(self.obs_id, obs_pos)
            
            # Handle target changes from GUI
            if self.state["force_target"]:
                req_target_pos = np.array([
                    self.state["target_x"],
                    self.state["target_y"],
                    self.state["target_z"]
                ])
                ref, end_snp = self.nmq.find_nearest_poly(req_target_pos)
                if ref > 0:
                    end_pos = end_snp
                    self.state["target_x"] = float(end_snp[0])
                    self.state["target_y"] = float(end_snp[1])
                    self.state["target_z"] = float(end_snp[2])
                else:
                    end_pos = req_target_pos
                
                self.crowd.request_move_target(self.robot_id, end_pos, self.nmq)
                self.state["force_target"] = False
                self.get_logger().info(f'New target set: {np.round(end_pos, 3)}')
            
            # Handle move request from GUI button
            if self.state["request_move"]:
                target_pos = np.array([
                    self.state["target_x"],
                    self.state["target_y"],
                    self.state["target_z"]
                ])
                if not self.crowd.request_move_target(self.robot_id, target_pos, self.nmq):
                    self.get_logger().warn('Could not start movement: target not on navmesh')
                else:
                    self.get_logger().info(f'Move requested to: {np.round(target_pos, 3)}')
                self.state["request_move"] = False
        
        # Compute dt
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        if dt > 0.1:
            dt = 0.1
        
        # Update crowd simulation
        self.crowd.update(dt)
        
        # Get steering velocity and publish cmd_vel
        pos, vel = self.crowd.get_agent_pos(self.robot_id)
        if vel is not None and pos is not None:
            cmd_vel = self.compute_cmd_vel(vel)
            self.cmd_vel_pub.publish(cmd_vel)
    
    def compute_cmd_vel(self, recast_vel):
        """
        Transform Recast velocity vector to Twist cmd_vel.
        
        Args:
            recast_vel: 3D velocity vector [vx, vy, vz] from Recast
            
        Returns:
            Twist message with computed velocities
        """
        cmd_vel = Twist()
        
        if recast_vel is None or np.linalg.norm(recast_vel) < 1e-4:
            # No velocity, stop
            return cmd_vel
        
        with self.state_lock:
            current_heading = self.state["robot_heading"]
        
        # Compute desired heading from velocity vector
        vel_x, vel_y, vel_z = recast_vel
        desired_heading = math.atan2(vel_y, vel_x)
        
        # Compute angular error (normalized to [-pi, pi])
        angular_error = self.normalize_angle(desired_heading - current_heading)
        
        # P-controller for angular velocity
        angular_vel = self.kp_angular * angular_error
        angular_vel = np.clip(angular_vel, -self.max_angular_speed, self.max_angular_speed)
        
        # Linear velocity (magnitude of velocity vector in XY plane)
        speed = math.sqrt(vel_x**2 + vel_y**2)
        speed = min(speed, self.max_linear_speed)
        
        cmd_vel.linear.x = speed
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = angular_vel
        
        return cmd_vel
    
    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def initialize_recast(self, bmin, bmax):
        """Initialize Recast navmesh and crowd."""
        self.nm = NavMesh(self.navmesh_file)
        self.nmq = NavMeshQuery(self.nm)
        self.crowd = Crowd(self.nm, max_agents=10, max_agent_radius=0.5)
        
        # Find reasonable start and end positions
        start_pos = np.array([-4.0, 0.0, 0.0])
        end_pos = np.array([4.0, 0.0, 0.0])
        
        verts, _ = self.nm.get_geometry(swap_yz=True)
        if len(verts) > 0:
            ref, start_snp = self.nmq.find_nearest_poly(bmin + np.array([1, 1, 0]))
            ref2, end_snp = self.nmq.find_nearest_poly(bmax - np.array([1, 1, 0]))
            if ref > 0 and ref2 > 0:
                start_pos = start_snp
                end_pos = end_snp
        
        with self.state_lock:
            self.state["target_x"] = end_pos[0]
            self.state["target_y"] = end_pos[1]
            self.state["target_z"] = end_pos[2]
        
        # Add robot agent (position will be updated from /robotPose)
        self.robot_id = self.crowd.add_agent(
            start_pos,
            radius=0.3,
            maxAcceleration=8.0,
            maxSpeed=2.0,
            collisionQueryRange=2.0
        )
        
        # Add obstacle agent
        obs_pos = np.array([
            self.state["obs_x"],
            self.state["obs_y"],
            self.state["obs_z"]
        ])
        self.obs_id = self.crowd.add_agent(
            obs_pos,
            radius=0.6,
            maxAcceleration=0.0,
            maxSpeed=0.0
        )
        
        self.get_logger().info('Recast crowd initialized')
        self.get_logger().info(f'Robot agent ID: {self.robot_id}')
        self.get_logger().info(f'Obstacle agent ID: {self.obs_id}')
        
        return end_pos
    
    def shutdown(self):
        """Clean shutdown of resources."""
        with self.state_lock:
            self.state["run"] = False
        
        if self.crowd is not None:
            self.crowd.close()
        if self.nmq is not None:
            self.nmq.close()
        if self.nm is not None:
            self.nm.close()


def create_gui(node, bmin, bmax):
    """Create Tkinter GUI for obstacle and target control."""
    root = tk.Tk()
    root.title("Robot Co-Simulation Control")
    root.geometry("380x350")
    
    with node.state_lock:
        x_var = tk.DoubleVar(value=node.state["obs_x"])
        y_var = tk.DoubleVar(value=node.state["obs_y"])
        z_var = tk.DoubleVar(value=node.state["obs_z"])
        tx_var = tk.DoubleVar(value=node.state["target_x"])
        ty_var = tk.DoubleVar(value=node.state["target_y"])
        tz_var = tk.DoubleVar(value=node.state["target_z"])
    
    def on_x(val):
        with node.state_lock:
            node.state["obs_x"] = float(val)
    
    def on_y(val):
        with node.state_lock:
            node.state["obs_y"] = float(val)
    
    def on_z(val):
        with node.state_lock:
            node.state["obs_z"] = float(val)
    
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
    
    # Obstacle position sliders
    tk.Label(root, text="Obstacle Position", font=('Arial', 10, 'bold')).pack(pady=(10, 5))
    
    scale_x = tk.Scale(root, from_=bmin[0], to=bmax[0], resolution=0.1,
                       orient=tk.HORIZONTAL, label="Obstacle X",
                       variable=x_var, command=on_x)
    scale_x.pack(fill=tk.X, padx=10)
    
    scale_y = tk.Scale(root, from_=bmin[1], to=bmax[1], resolution=0.1,
                       orient=tk.HORIZONTAL, label="Obstacle Y",
                       variable=y_var, command=on_y)
    scale_y.pack(fill=tk.X, padx=10)
    
    scale_z = tk.Scale(root, from_=-50.0, to=50.0, resolution=0.1,
                       orient=tk.HORIZONTAL, label="Obstacle Z",
                       variable=z_var, command=on_z)
    scale_z.pack(fill=tk.X, padx=10)
    
    # Target position sliders
    tk.Label(root, text="Target Position", font=('Arial', 10, 'bold')).pack(pady=(10, 5))
    
    scale_tx = tk.Scale(root, from_=bmin[0], to=bmax[0], resolution=0.1,
                        orient=tk.HORIZONTAL, label="Target X",
                        variable=tx_var, command=on_tx)
    scale_tx.pack(fill=tk.X, padx=10)
    
    scale_ty = tk.Scale(root, from_=bmin[1], to=bmax[1], resolution=0.1,
                        orient=tk.HORIZONTAL, label="Target Y",
                        variable=ty_var, command=on_ty)
    scale_ty.pack(fill=tk.X, padx=10)
    
    scale_tz = tk.Scale(root, from_=bmin[2], to=bmax[2], resolution=0.1,
                        orient=tk.HORIZONTAL, label="Target Z",
                        variable=tz_var, command=on_tz)
    scale_tz.pack(fill=tk.X, padx=10)
    
    # Start movement button
    start_btn = tk.Button(root, text="Start Robot Movement", command=on_start_move)
    start_btn.pack(fill=tk.X, padx=10, pady=(10, 4))
    
    def on_close():
        with node.state_lock:
            node.state["run"] = False
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_close)
    return root


def make_wireframe(nm):
    """Create wireframe visualization of navmesh."""
    verts, tris = nm.get_geometry(swap_yz=True)
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(verts.astype(np.float64))
    mesh.triangles = o3d.utility.Vector3iVector(tris.astype(np.int32))
    ls = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
    ls.paint_uniform_color((0.0, 0.6, 0.2))
    return ls


def main():
    # Separate ROS args from script args
    # ROS args come after --ros-args
    import sys
    script_args = []
    ros_args = []
    
    if '--ros-args' in sys.argv:
        idx = sys.argv.index('--ros-args')
        script_args = sys.argv[1:idx]
        ros_args = sys.argv[idx+1:]
    else:
        script_args = sys.argv[1:]
    
    parser = argparse.ArgumentParser()
    parser.add_argument("bin", help="Navmesh .bin file")
    parser.add_argument("--obj", help="Optional OBJ file to render", default=None)
    args = parser.parse_args(script_args)
    
    # Initialize ROS with ROS-specific args
    rclpy.init(args=ros_args)
    
    # Create node
    node = CrowdCosimNode(args.bin, args.obj)
    
    # Initialize variables for cleanup
    executor = None
    vis = None
    root = None
    
    try:
        # First, need to load navmesh to get geometry
        # Temporarily load navmesh just to get bounds
        temp_nm = NavMesh(args.bin)
        verts, tris = temp_nm.get_geometry(swap_yz=True)
        
        # Determine bounds
        bmin = np.array([-20.0, -20.0, 0.0])
        bmax = np.array([20.0, 20.0, 0.0])
        
        if len(verts) > 0:
            bmin = np.min(verts, axis=0)
            bmax = np.max(verts, axis=0)
            with node.state_lock:
                node.state["obs_x"] = float((bmin[0] + bmax[0]) / 2)
                node.state["obs_y"] = float((bmin[1] + bmax[1]) / 2)
                node.state["obs_z"] = float((bmin[2] + bmax[2]) / 2)
        
        # Initialize Recast (this will properly load the navmesh in the node)
        end_pos = node.initialize_recast(bmin, bmax)
        
        # Create GUI (must be in main thread)
        root = create_gui(node, bmin, bmax)
        
        # Create visualization
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        
        # Add navmesh point cloud
        pcd = o3d.geometry.PointCloud()
        if len(verts) > 0:
            pcd.points = o3d.utility.Vector3dVector(verts.astype(np.float64))
            pcd.paint_uniform_color([0.3, 0.3, 0.3])
            vis.add_geometry(pcd)
        
        # Add navmesh wireframe
        nm_wire = make_wireframe(node.nm)
        vis.add_geometry(nm_wire)
        
        # Add OBJ mesh if provided
        if args.obj:
            obj_mesh = o3d.io.read_triangle_mesh(args.obj)
            if not obj_mesh.has_vertex_normals():
                obj_mesh.compute_vertex_normals()
            vis.add_geometry(obj_mesh)
        
        # Robot mesh (blue sphere)
        robot_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)
        robot_mesh.paint_uniform_color([0.1, 0.4, 1.0])
        robot_mesh.compute_vertex_normals()
        vis.add_geometry(robot_mesh)
        
        # Target mesh (green sphere)
        target_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
        target_mesh.paint_uniform_color([0.0, 1.0, 0.0])
        target_mesh.translate(end_pos)
        target_mesh.compute_vertex_normals()
        vis.add_geometry(target_mesh)
        
        # Obstacle mesh (red cylinder)
        obs_mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=0.6, height=1.0)
        obs_mesh.paint_uniform_color([1.0, 0.2, 0.2])
        obs_mesh.compute_vertex_normals()
        vis.add_geometry(obs_mesh)
        
        node.get_logger().info("Co-simulation started. Waiting for /robotPose...")
        node.get_logger().info("Control obstacle and target via GUI. Close windows to exit.")
        
        # Create executor for ROS in separate thread
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        # Spin ROS in background thread
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()
        
        # Main loop: GUI updates and visualization
        while True:
            with node.state_lock:
                if not node.state["run"]:
                    break
            
            # Update GUI
            root.update_idletasks()
            root.update()
            
            # Update visualization
            if not vis.poll_events():
                break
            vis.update_renderer()
            
            # Update robot position in visualization
            with node.state_lock:
                robot_pos = node.state["robot_pos"]
                target_pos = np.array([
                    node.state["target_x"],
                    node.state["target_y"],
                    node.state["target_z"]
                ])
                obs_pos_viz = np.array([
                    node.state["obs_x"],
                    node.state["obs_y"],
                    node.state["obs_z"]
                ])
            
            if robot_pos is not None:
                robot_mesh.vertices = o3d.geometry.TriangleMesh.create_sphere(radius=0.3).vertices
                robot_mesh.translate(robot_pos)
                vis.update_geometry(robot_mesh)
            
            # Update target mesh
            target_mesh.vertices = o3d.geometry.TriangleMesh.create_sphere(radius=0.1).vertices
            target_mesh.translate(target_pos)
            vis.update_geometry(target_mesh)
            
            # Update obstacle mesh
            obs_mesh.vertices = o3d.geometry.TriangleMesh.create_cylinder(radius=0.6, height=1.0).vertices
            R = obs_mesh.get_rotation_matrix_from_xyz((np.pi/2, 0, 0))
            obs_mesh.rotate(R, center=(0, 0, 0))
            obs_mesh.translate(obs_pos_viz)
            vis.update_geometry(obs_mesh)
            
            time.sleep(1/60)
    
    finally:
        with node.state_lock:
            node.state["run"] = False
        
        # Cleanup
        if vis is not None:
            try:
                vis.destroy_window()
            except:
                pass
        
        if root is not None:
            try:
                root.destroy()
            except:
                pass
        
        node.shutdown()
        
        if executor is not None:
            executor.shutdown()
        
        rclpy.shutdown()
        
        node.get_logger().info("Co-simulation node shut down")


if __name__ == "__main__":
    main()
