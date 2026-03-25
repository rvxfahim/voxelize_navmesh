import json
from dataclasses import dataclass
from pathlib import Path

from .recast_config import RecastBuildConfig


PROJECT_VERSION = 1


@dataclass
class NavmeshProject:
    mesh_path: str
    config: RecastBuildConfig
    last_navmesh_bin: str | None = None

    def to_dict(self, project_path: Path | None = None) -> dict:
        mesh_path = self.mesh_path
        last_navmesh_bin = self.last_navmesh_bin
        if project_path is not None:
            base = project_path.parent
            try:
                mesh_path = str(Path(self.mesh_path).resolve().relative_to(base.resolve()))
            except Exception:
                mesh_path = self.mesh_path
            if self.last_navmesh_bin:
                try:
                    last_navmesh_bin = str(Path(self.last_navmesh_bin).resolve().relative_to(base.resolve()))
                except Exception:
                    last_navmesh_bin = self.last_navmesh_bin
        return {
            "version": PROJECT_VERSION,
            "mesh_path": mesh_path,
            "config": self.config.to_dict(),
            "last_navmesh_bin": last_navmesh_bin,
        }

    @classmethod
    def from_dict(cls, data: dict, project_path: Path | None = None) -> "NavmeshProject":
        if int(data.get("version", 0)) != PROJECT_VERSION:
            raise ValueError(f"Unsupported project version: {data.get('version')}")
        mesh_path = data["mesh_path"]
        last_navmesh_bin = data.get("last_navmesh_bin")
        if project_path is not None:
            base = project_path.parent
            mesh_path = str((base / mesh_path).resolve()) if not Path(mesh_path).is_absolute() else mesh_path
            if last_navmesh_bin and not Path(last_navmesh_bin).is_absolute():
                last_navmesh_bin = str((base / last_navmesh_bin).resolve())
        return cls(
            mesh_path=mesh_path,
            config=RecastBuildConfig.from_dict(data.get("config", {})),
            last_navmesh_bin=last_navmesh_bin,
        )


def save_project(project: NavmeshProject, path: str) -> None:
    p = Path(path)
    payload = project.to_dict(project_path=p)
    p.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_project(path: str) -> NavmeshProject:
    p = Path(path)
    data = json.loads(p.read_text(encoding="utf-8"))
    return NavmeshProject.from_dict(data, project_path=p)
