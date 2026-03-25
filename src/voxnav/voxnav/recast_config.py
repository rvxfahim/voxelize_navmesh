from dataclasses import dataclass
from typing import Optional


@dataclass
class RecastBuildConfig:
    cell_size: float = 0.3
    cell_height: float = 0.2
    agent_height: float = 2.0
    agent_radius: float = 0.6
    agent_max_climb: float = 0.9
    agent_max_slope: float = 45.0
    region_min_size: float = 8.0
    region_merge_size: float = 20.0
    edge_max_len: float = 12.0
    edge_max_error: float = 1.3
    verts_per_poly: int = 6
    detail_sample_dist: float = 6.0
    detail_sample_max_error: float = 1.0
    partition_type: int = 0
    filter_low_hanging_obstacles: bool = True
    filter_ledge_spans: bool = True
    filter_walkable_low_height_spans: bool = True
    # Bounding box constraints (None = use mesh bounds)
    bounds_min: Optional[tuple[float, float, float]] = None
    bounds_max: Optional[tuple[float, float, float]] = None

    def validate(self) -> None:
        if self.cell_size <= 0:
            raise ValueError("cell_size must be > 0")
        if self.cell_height <= 0:
            raise ValueError("cell_height must be > 0")
        if self.verts_per_poly < 3:
            raise ValueError("verts_per_poly must be >= 3")
        if self.partition_type not in (0, 1, 2):
            raise ValueError("partition_type must be 0 (Watershed), 1 (Monotone), or 2 (Layers)")

    def to_dict(self) -> dict:
        return {
            "cell_size": self.cell_size,
            "cell_height": self.cell_height,
            "agent_height": self.agent_height,
            "agent_radius": self.agent_radius,
            "agent_max_climb": self.agent_max_climb,
            "agent_max_slope": self.agent_max_slope,
            "region_min_size": self.region_min_size,
            "region_merge_size": self.region_merge_size,
            "edge_max_len": self.edge_max_len,
            "edge_max_error": self.edge_max_error,
            "verts_per_poly": self.verts_per_poly,
            "detail_sample_dist": self.detail_sample_dist,
            "detail_sample_max_error": self.detail_sample_max_error,
            "partition_type": self.partition_type,
            "filter_low_hanging_obstacles": self.filter_low_hanging_obstacles,
            "filter_ledge_spans": self.filter_ledge_spans,
            "filter_walkable_low_height_spans": self.filter_walkable_low_height_spans,
            "bounds_min": self.bounds_min,
            "bounds_max": self.bounds_max,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "RecastBuildConfig":
        cfg = cls(**{k: v for k, v in data.items() if k in cls.__dataclass_fields__})
        cfg.validate()
        return cfg
