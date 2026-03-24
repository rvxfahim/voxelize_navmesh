import argparse
import trimesh

def main():
    parser = argparse.ArgumentParser(description="Reorient an existing OBJ file to Y-up for Recast")
    parser.add_argument("input_obj", help="Input OBJ file")
    parser.add_argument("output_obj", help="Output OBJ file")
    args = parser.parse_args()

    print(f"Loading {args.input_obj} ...")
    mesh = trimesh.load(args.input_obj)

    print("Swapping Y and Z axes ...")
    # Swap Y↔Z
    verts = mesh.vertices.copy()
    verts[:, 1], verts[:, 2] = mesh.vertices[:, 2].copy(), mesh.vertices[:, 1].copy()
    
    print("Reversing face winding ...")
    # Reverse face winding to fix normals after axis swap
    faces = mesh.faces[:, ::-1].copy()

    out_mesh = trimesh.Trimesh(vertices=verts, faces=faces)
    
    print(f"Exporting to {args.output_obj} ...")
    out_mesh.export(args.output_obj)
    print("Done.")

if __name__ == "__main__":
    main()
