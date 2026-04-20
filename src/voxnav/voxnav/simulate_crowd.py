import time
import argparse
import os
import sys
import numpy as np
import open3d as o3d
import tkinter as tk

# Try to import from the voxnav package; if not available, add the local directory to path
try:
    from voxnav.navmesh import NavMesh, NavMeshQuery, Crowd
except ImportError:
    # Add package directory to path for local imports (when running from source)
    _pkg_dir = os.path.dirname(os.path.abspath(__file__))
    if _pkg_dir not in sys.path:
        sys.path.insert(0, _pkg_dir)
    from voxnav.navmesh import NavMesh, NavMeshQuery, Crowd

# Globals to communicate between Tkinter and Open3D
state = {
    "run": True,
    "obs_x": 0.0,
    "obs_y": 0.0,
    "obs_z": 0.0,
    "start_x": 0.0,
    "start_y": 0.0,
    "start_z": 0.0,
    "target_x": 0.0,
    "target_y": 0.0,
    "target_z": 0.0,
    "force_start": False,
    "force_target": False,
    "request_move": False
}

def create_gui(bmin, bmax):
    root = tk.Tk()
    root.title("Crowd Control")
    root.geometry("380x500")
    
    x_var = tk.DoubleVar(value=state["obs_x"])
    y_var = tk.DoubleVar(value=state["obs_y"])
    z_var = tk.DoubleVar(value=state["obs_z"])
    sx_var = tk.DoubleVar(value=state["start_x"])
    sy_var = tk.DoubleVar(value=state["start_y"])
    sz_var = tk.DoubleVar(value=state["start_z"])
    tx_var = tk.DoubleVar(value=state["target_x"])
    ty_var = tk.DoubleVar(value=state["target_y"])
    tz_var = tk.DoubleVar(value=state["target_z"])
    
    def on_x(val): state["obs_x"] = float(val)
    def on_y(val): state["obs_y"] = float(val)
    def on_z(val): state["obs_z"] = float(val)
    
    def on_sx(val): 
        state["start_x"] = float(val)
        state["force_start"] = True
        
    def on_sy(val): 
        state["start_y"] = float(val)
        state["force_start"] = True

    def on_sz(val):
        state["start_z"] = float(val)
        state["force_start"] = True

    def on_tx(val):
        state["target_x"] = float(val)
        state["force_target"] = True

    def on_ty(val):
        state["target_y"] = float(val)
        state["force_target"] = True

    def on_tz(val):
        state["target_z"] = float(val)
        state["force_target"] = True

    def on_start_move():
        state["request_move"] = True

    scale_x = tk.Scale(root, from_=bmin[0], to=bmax[0], resolution=0.1, orient=tk.HORIZONTAL, label="Obstacle X", variable=x_var, command=on_x)
    scale_x.pack(fill=tk.X, padx=10)
    
    scale_y = tk.Scale(root, from_=bmin[1], to=bmax[1], resolution=0.1, orient=tk.HORIZONTAL, label="Obstacle Y", variable=y_var, command=on_y)
    scale_y.pack(fill=tk.X, padx=10)

    scale_z = tk.Scale(root, from_=-50.0, to=50.0, resolution=0.1, orient=tk.HORIZONTAL, label="Obstacle Z", variable=z_var, command=on_z)
    scale_z.pack(fill=tk.X, padx=10)

    # Added Agent Start Position Sliders
    scale_sx = tk.Scale(root, from_=bmin[0], to=bmax[0], resolution=0.1, orient=tk.HORIZONTAL, label="Agent Start X", variable=sx_var, command=on_sx)
    scale_sx.pack(fill=tk.X, padx=10)
    
    scale_sy = tk.Scale(root, from_=bmin[1], to=bmax[1], resolution=0.1, orient=tk.HORIZONTAL, label="Agent Start Y", variable=sy_var, command=on_sy)
    scale_sy.pack(fill=tk.X, padx=10)

    scale_sz = tk.Scale(root, from_=bmin[2], to=bmax[2], resolution=0.1, orient=tk.HORIZONTAL, label="Agent Start Z", variable=sz_var, command=on_sz)
    scale_sz.pack(fill=tk.X, padx=10)

    scale_tx = tk.Scale(root, from_=bmin[0], to=bmax[0], resolution=0.1, orient=tk.HORIZONTAL, label="Target X", variable=tx_var, command=on_tx)
    scale_tx.pack(fill=tk.X, padx=10)

    scale_ty = tk.Scale(root, from_=bmin[1], to=bmax[1], resolution=0.1, orient=tk.HORIZONTAL, label="Target Y", variable=ty_var, command=on_ty)
    scale_ty.pack(fill=tk.X, padx=10)

    scale_tz = tk.Scale(root, from_=bmin[2], to=bmax[2], resolution=0.1, orient=tk.HORIZONTAL, label="Target Z", variable=tz_var, command=on_tz)
    scale_tz.pack(fill=tk.X, padx=10)

    start_btn = tk.Button(root, text="Start Agent Movement", command=on_start_move)
    start_btn.pack(fill=tk.X, padx=10, pady=(8, 4))

    def on_close():
        state["run"] = False
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    return root

def make_wireframe(nm):
    verts, tris = nm.get_geometry(swap_yz=True)
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(verts.astype(np.float64))
    mesh.triangles = o3d.utility.Vector3iVector(tris.astype(np.int32))
    ls = o3d.geometry.LineSet.create_from_triangle_mesh(mesh)
    ls.paint_uniform_color((0.0, 0.6, 0.2))
    return ls

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bin", help="Navmesh .bin file")
    parser.add_argument("--obj", help="Optional OBJ file to render along with the navmesh", default=None)
    args = parser.parse_args()

    nm = None
    nmq = None
    crowd = None
    vis = None
    root = None
    try:
        nm = NavMesh(args.bin)
        nmq = NavMeshQuery(nm)
        crowd = Crowd(nm, max_agents=10, max_agent_radius=0.5)

        start_pos = np.array([-4.0, 0.0, 0.0]) # Z-up
        end_pos = np.array([4.0, 0.0, 0.0])
        
        verts, tris = nm.get_geometry(swap_yz=True)
        bmin = np.array([-20.0, -20.0, 0.0])
        bmax = np.array([20.0, 20.0, 0.0])
        
        if len(verts) > 0:
            bmin = np.min(verts, axis=0)
            bmax = np.max(verts, axis=0)
            state["obs_x"] = float((bmin[0]+bmax[0])/2)
            state["obs_y"] = float((bmin[1]+bmax[1])/2)
            state["obs_z"] = float((bmin[2]+bmax[2])/2)
        
            ref, start_snp = nmq.find_nearest_poly(bmin + np.array([1, 1, 0]))
            ref2, end_snp = nmq.find_nearest_poly(bmax - np.array([1, 1, 0]))
            if ref > 0 and ref2 > 0:
                start_pos = start_snp
                end_pos = end_snp
                
        state["start_x"] = start_pos[0]
        state["start_y"] = start_pos[1]
        state["start_z"] = start_pos[2]
        state["target_x"] = end_pos[0]
        state["target_y"] = end_pos[1]
        state["target_z"] = end_pos[2]
                
        agent_id = crowd.add_agent(start_pos, radius=0.3, maxAcceleration=8.0, maxSpeed=2.0, collisionQueryRange=2.0)

        obs_pos = np.array([state["obs_x"], state["obs_y"], state["obs_z"]]) 
        obs_id = crowd.add_agent(obs_pos, radius=0.6, maxAcceleration=0.0, maxSpeed=0.0, obstacleWeight=0.0)

        # Tk must be initialized from the main thread.
        root = create_gui(bmin, bmax)

        # VisualizerWithEditing.get_picked_points() is not safe in a live poll loop.
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        
        # We must add a PointCloud to pick points successfully
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(verts.astype(np.float64))
        pcd.paint_uniform_color([0.3, 0.3, 0.3])
        vis.add_geometry(pcd)
        
        nm_wire = make_wireframe(nm)
        vis.add_geometry(nm_wire)
        
        if args.obj:
            obj_mesh = o3d.io.read_triangle_mesh(args.obj)
            if not obj_mesh.has_vertex_normals():
                obj_mesh.compute_vertex_normals()
            vis.add_geometry(obj_mesh)

        agent_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)
        agent_mesh.paint_uniform_color([0.1, 0.4, 1.0])
        agent_mesh.compute_vertex_normals()
        vis.add_geometry(agent_mesh)

        target_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
        target_mesh.paint_uniform_color([0.0, 1.0, 0.0])
        target_mesh.translate(end_pos)
        target_mesh.compute_vertex_normals()
        vis.add_geometry(target_mesh)
        
        obs_mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=0.6, height=1.0)
        obs_mesh.paint_uniform_color([1.0, 0.2, 0.2])
        obs_mesh.compute_vertex_normals()
        vis.add_geometry(obs_mesh)

        last_time = time.time()
        last_picks_len = 0
        move_check_remaining = 0
        move_baseline_pos = None
        
        print("Simulation started. Set start/target in Tk and click 'Start Agent Movement'. Close windows to exit.")
        
        while state["run"]:
            root.update_idletasks()
            root.update()

            if not vis.poll_events():
                break
            vis.update_renderer()

            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            if dt > 0.1:
                dt = 0.1
                
            req_obs_pos = np.array([state["obs_x"], state["obs_y"], state["obs_z"]])
            crowd.force_agent_pos(obs_id, req_obs_pos)
                
            # Handle Agent Start Pos Override
            if state["force_start"]:
                req_start_pos = np.array([state["start_x"], state["start_y"], state["start_z"]])
                ref, start_snp = nmq.find_nearest_poly(req_start_pos)
                if ref > 0:
                    if not crowd.teleport_agent(agent_id, start_snp):
                        print("Could not set start: snapped start is not on navmesh.")
                    else:
                        state["start_x"] = float(start_snp[0])
                        state["start_y"] = float(start_snp[1])
                        state["start_z"] = float(start_snp[2])
                else:
                    if not crowd.teleport_agent(agent_id, req_start_pos):
                        print("Could not set start: requested start is not on navmesh.")
                
                state["force_start"] = False

            if state["force_target"]:
                req_target_pos = np.array([state["target_x"], state["target_y"], state["target_z"]])
                ref, end_snp = nmq.find_nearest_poly(req_target_pos)
                if ref > 0:
                    end_pos = end_snp
                    state["target_x"] = float(end_snp[0])
                    state["target_y"] = float(end_snp[1])
                    state["target_z"] = float(end_snp[2])
                else:
                    end_pos = req_target_pos

                crowd.request_move_target(agent_id, end_pos, nmq)
                target_mesh.vertices = o3d.geometry.TriangleMesh.create_sphere(radius=0.1).vertices
                target_mesh.translate(end_pos)
                vis.update_geometry(target_mesh)
                state["force_target"] = False

            if state["request_move"]:
                move_baseline_pos, _ = crowd.get_agent_pos(agent_id)
                if not crowd.request_move_target(agent_id, end_pos, nmq):
                    print("Could not start movement: target is not on navmesh.")
                    move_check_remaining = 0
                else:
                    print(f"Move requested: target={np.round(end_pos, 3)}")
                    move_check_remaining = 20
                state["request_move"] = False
                
            crowd.update(dt)
            
            pos, vel = crowd.get_agent_pos(agent_id)
            if move_check_remaining > 0 and move_baseline_pos is not None and pos is not None:
                moved_dist = float(np.linalg.norm(pos - move_baseline_pos))
                speed = float(np.linalg.norm(vel)) if vel is not None else 0.0
                if moved_dist > 1e-3 or speed > 5e-3:
                    move_check_remaining = 0
                else:
                    move_check_remaining -= 1
            elif move_check_remaining == 0 and move_baseline_pos is not None:
                moved_dist = float(np.linalg.norm(pos - move_baseline_pos)) if pos is not None else 0.0
                speed = float(np.linalg.norm(vel)) if vel is not None else 0.0
                if moved_dist < 1e-3 and speed < 5e-3:
                    print("Move request accepted but no movement yet. Check start/target placement and nearby blockers.")
                move_baseline_pos = None
            if pos is not None:
                agent_mesh.vertices = o3d.geometry.TriangleMesh.create_sphere(radius=0.3).vertices
                agent_mesh.translate(pos)
                vis.update_geometry(agent_mesh)

            opos, ovel = crowd.get_agent_pos(obs_id)
            if opos is not None:
                obs_mesh.vertices = o3d.geometry.TriangleMesh.create_cylinder(radius=0.6, height=1.0).vertices
                R = obs_mesh.get_rotation_matrix_from_xyz((np.pi/2, 0, 0))
                obs_mesh.rotate(R, center=(0,0,0))
                obs_mesh.translate(opos)
                vis.update_geometry(obs_mesh)

            time.sleep(1/60)
    finally:
        state["run"] = False
        if vis is not None:
            vis.destroy_window()
        if root is not None:
            try:
                root.destroy()
            except tk.TclError:
                pass
        if crowd is not None:
            crowd.close()
        if nmq is not None:
            nmq.close()
        if nm is not None:
            nm.close()

if __name__ == "__main__":
    main()
