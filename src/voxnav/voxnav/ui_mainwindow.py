from pathlib import Path

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QAction,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QDoubleSpinBox,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from .baker_backend import build_navmesh_from_obj
from .project_io import NavmeshProject, load_project, save_project
from .recast_config import RecastBuildConfig
from .viewer_controller import ViewerController


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt Navmesh Baker")
        self.resize(720, 640)

        self._viewer = None
        self._viewer_enabled = False
        self._viewer_timer = QTimer(self)
        self._viewer_timer.timeout.connect(self._tick_viewer)

        self._project_path: str | None = None
        self._dirty = False
        self._baked = None

        self._mesh_path = QLineEdit()
        self._mesh_path.setPlaceholderText("Path to OBJ mesh (Y-up)")
        self._mesh_path.editingFinished.connect(self._on_mesh_edited)

        form = QFormLayout()
        self._controls = {}

        def add_float(name: str, lo: float, hi: float, step: float, value: float):
            sb = QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setSingleStep(step)
            sb.setDecimals(4)
            sb.setValue(value)
            sb.valueChanged.connect(self._mark_dirty)
            self._controls[name] = sb
            form.addRow(name, sb)

        def add_int(name: str, lo: int, hi: int, value: int):
            sb = QSpinBox()
            sb.setRange(lo, hi)
            sb.setValue(value)
            sb.valueChanged.connect(self._mark_dirty)
            self._controls[name] = sb
            form.addRow(name, sb)

        cfg = RecastBuildConfig()
        add_float("cell_size", 0.01, 2.0, 0.01, cfg.cell_size)
        add_float("cell_height", 0.01, 2.0, 0.01, cfg.cell_height)
        add_float("agent_height", 0.1, 10.0, 0.1, cfg.agent_height)
        add_float("agent_radius", 0.001, 10.0, 0.01, cfg.agent_radius)
        add_float("agent_max_climb", 0.0, 10.0, 0.1, cfg.agent_max_climb)
        add_float("agent_max_slope", 0.0, 90.0, 1.0, cfg.agent_max_slope)
        add_float("region_min_size", 0.0, 300.0, 1.0, cfg.region_min_size)
        add_float("region_merge_size", 0.0, 300.0, 1.0, cfg.region_merge_size)
        add_float("edge_max_len", 0.0, 100.0, 0.5, cfg.edge_max_len)
        add_float("edge_max_error", 0.1, 10.0, 0.1, cfg.edge_max_error)
        add_int("verts_per_poly", 3, 6, cfg.verts_per_poly)
        add_float("detail_sample_dist", 0.0, 32.0, 0.5, cfg.detail_sample_dist)
        add_float("detail_sample_max_error", 0.0, 32.0, 0.1, cfg.detail_sample_max_error)

        self._partition = QComboBox()
        self._partition.addItems(["Watershed", "Monotone", "Layers"])
        self._partition.currentIndexChanged.connect(self._mark_dirty)
        form.addRow("partition_type", self._partition)

        self._f_low = QCheckBox("Low Hanging Obstacles")
        self._f_low.setChecked(cfg.filter_low_hanging_obstacles)
        self._f_low.toggled.connect(self._mark_dirty)
        form.addRow("filter_low_hanging_obstacles", self._f_low)

        self._f_ledge = QCheckBox("Ledge Spans")
        self._f_ledge.setChecked(cfg.filter_ledge_spans)
        self._f_ledge.toggled.connect(self._mark_dirty)
        form.addRow("filter_ledge_spans", self._f_ledge)

        self._f_low_height = QCheckBox("Walkable Low Height Spans")
        self._f_low_height.setChecked(cfg.filter_walkable_low_height_spans)
        self._f_low_height.toggled.connect(self._mark_dirty)
        form.addRow("filter_walkable_low_height_spans", self._f_low_height)

        self._show_obj = QCheckBox("Show OBJ in preview")
        self._show_obj.setChecked(True)
        self._show_obj.toggled.connect(self._on_preview_mode_changed)
        form.addRow("preview_show_obj", self._show_obj)

        self._obj_wire = QCheckBox("OBJ wireframe mode")
        self._obj_wire.setChecked(False)
        self._obj_wire.toggled.connect(self._on_preview_mode_changed)
        form.addRow("preview_obj_wireframe", self._obj_wire)

        self._navmesh_hi_contrast = QCheckBox("High contrast navmesh")
        self._navmesh_hi_contrast.setChecked(True)
        self._navmesh_hi_contrast.toggled.connect(self._on_preview_mode_changed)
        form.addRow("preview_navmesh_high_contrast", self._navmesh_hi_contrast)

        browse = QPushButton("Browse OBJ")
        browse.clicked.connect(self._browse_obj)
        bake = QPushButton("Bake Navmesh")
        bake.clicked.connect(self._bake)
        export = QPushButton("Export .bin")
        export.clicked.connect(self._export_navmesh)
        buttons = QHBoxLayout()
        buttons.addWidget(browse)
        buttons.addWidget(bake)
        buttons.addWidget(export)

        root = QWidget()
        lay = QVBoxLayout(root)
        lay.addWidget(QLabel("Input OBJ"))
        lay.addWidget(self._mesh_path)
        lay.addLayout(form)
        lay.addLayout(buttons)
        self.setCentralWidget(root)

        self._status = self.statusBar()
        self._status.showMessage("Ready")

        self._build_menus()

    def _build_menus(self):
        menu = self.menuBar().addMenu("Project")
        act_new = QAction("New", self)
        act_new.triggered.connect(self._new_project)
        menu.addAction(act_new)
        act_open = QAction("Open...", self)
        act_open.triggered.connect(self._open_project)
        menu.addAction(act_open)
        act_save = QAction("Save", self)
        act_save.triggered.connect(self._save_project)
        menu.addAction(act_save)
        act_save_as = QAction("Save As...", self)
        act_save_as.triggered.connect(lambda: self._save_project(save_as=True))
        menu.addAction(act_save_as)

    def _tick_viewer(self):
        if not self._viewer_enabled or self._viewer is None:
            return
        if not self._viewer.poll():
            self._viewer_timer.stop()
            self._viewer_enabled = False

    def _ensure_viewer(self) -> bool:
        if self._viewer_enabled and self._viewer is not None:
            return True
        try:
            self._viewer = ViewerController()
            self._viewer_enabled = True
            self._viewer_timer.start(16)
            return True
        except Exception as e:
            self._viewer_enabled = False
            self._viewer = None
            self._status.showMessage("Preview disabled")
            QMessageBox.warning(
                self,
                "Preview disabled",
                f"Could not initialize Open3D preview window.\n{e}\n\nBaking/export still work.",
            )
            return False

    def _mark_dirty(self, *_):
        self._dirty = True

    def _current_config(self) -> RecastBuildConfig:
        cfg = RecastBuildConfig(
            cell_size=float(self._controls["cell_size"].value()),
            cell_height=float(self._controls["cell_height"].value()),
            agent_height=float(self._controls["agent_height"].value()),
            agent_radius=float(self._controls["agent_radius"].value()),
            agent_max_climb=float(self._controls["agent_max_climb"].value()),
            agent_max_slope=float(self._controls["agent_max_slope"].value()),
            region_min_size=float(self._controls["region_min_size"].value()),
            region_merge_size=float(self._controls["region_merge_size"].value()),
            edge_max_len=float(self._controls["edge_max_len"].value()),
            edge_max_error=float(self._controls["edge_max_error"].value()),
            verts_per_poly=int(self._controls["verts_per_poly"].value()),
            detail_sample_dist=float(self._controls["detail_sample_dist"].value()),
            detail_sample_max_error=float(self._controls["detail_sample_max_error"].value()),
            partition_type=int(self._partition.currentIndex()),
            filter_low_hanging_obstacles=bool(self._f_low.isChecked()),
            filter_ledge_spans=bool(self._f_ledge.isChecked()),
            filter_walkable_low_height_spans=bool(self._f_low_height.isChecked()),
        )
        cfg.validate()
        return cfg

    def _apply_config(self, cfg: RecastBuildConfig):
        self._controls["cell_size"].setValue(cfg.cell_size)
        self._controls["cell_height"].setValue(cfg.cell_height)
        self._controls["agent_height"].setValue(cfg.agent_height)
        self._controls["agent_radius"].setValue(cfg.agent_radius)
        self._controls["agent_max_climb"].setValue(cfg.agent_max_climb)
        self._controls["agent_max_slope"].setValue(cfg.agent_max_slope)
        self._controls["region_min_size"].setValue(cfg.region_min_size)
        self._controls["region_merge_size"].setValue(cfg.region_merge_size)
        self._controls["edge_max_len"].setValue(cfg.edge_max_len)
        self._controls["edge_max_error"].setValue(cfg.edge_max_error)
        self._controls["verts_per_poly"].setValue(cfg.verts_per_poly)
        self._controls["detail_sample_dist"].setValue(cfg.detail_sample_dist)
        self._controls["detail_sample_max_error"].setValue(cfg.detail_sample_max_error)
        self._partition.setCurrentIndex(cfg.partition_type)
        self._f_low.setChecked(cfg.filter_low_hanging_obstacles)
        self._f_ledge.setChecked(cfg.filter_ledge_spans)
        self._f_low_height.setChecked(cfg.filter_walkable_low_height_spans)

    def _on_mesh_edited(self):
        mesh = self._mesh_path.text().strip()
        if mesh and Path(mesh).exists():
            try:
                if self._ensure_viewer():
                    self._viewer.set_input_mesh(mesh)
                    self._apply_preview_settings()
            except Exception as e:
                QMessageBox.warning(self, "Mesh load failed", str(e))
        self._mark_dirty()

    def _apply_preview_settings(self):
        if not self._viewer_enabled or self._viewer is None:
            return
        if not self._show_obj.isChecked():
            self._viewer.set_input_mesh_display_mode("hidden")
        elif self._obj_wire.isChecked():
            self._viewer.set_input_mesh_display_mode("wireframe")
        else:
            self._viewer.set_input_mesh_display_mode("solid")
        self._viewer.set_navmesh_high_contrast(self._navmesh_hi_contrast.isChecked())

    def _on_preview_mode_changed(self, *_):
        self._apply_preview_settings()
        self._mark_dirty()

    def _browse_obj(self):
        path, _ = QFileDialog.getOpenFileName(self, "Choose OBJ mesh", "", "OBJ (*.obj)")
        if not path:
            return
        self._mesh_path.setText(path)
        self._on_mesh_edited()

    def _bake(self):
        mesh = self._mesh_path.text().strip()
        if not mesh:
            QMessageBox.warning(self, "Missing mesh", "Choose an OBJ first.")
            return
        try:
            cfg = self._current_config()
            if self._baked is not None:
                self._baked.close()
            self._status.showMessage("Baking navmesh...")
            self._baked = build_navmesh_from_obj(mesh, cfg)
            verts, tris = self._baked.get_geometry()
            if self._ensure_viewer():
                self._viewer.set_navmesh_wireframe(verts, tris)
                self._apply_preview_settings()
            self._status.showMessage("Bake complete")
            self._dirty = True
        except Exception as e:
            QMessageBox.critical(self, "Bake failed", str(e))
            self._status.showMessage("Bake failed")

    def _export_navmesh(self):
        if self._baked is None:
            QMessageBox.warning(self, "No navmesh", "Bake a navmesh first.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Export navmesh .bin", "", "Navmesh (*.bin)")
        if not path:
            return
        try:
            self._baked.save(path)
            self._status.showMessage(f"Saved: {path}")
            self._dirty = True
        except Exception as e:
            QMessageBox.critical(self, "Export failed", str(e))

    def _new_project(self):
        if not self._confirm_discard():
            return
        self._project_path = None
        self._mesh_path.setText("")
        self._apply_config(RecastBuildConfig())
        self._dirty = False
        self._status.showMessage("New project")

    def _open_project(self):
        if not self._confirm_discard():
            return
        path, _ = QFileDialog.getOpenFileName(self, "Open project", "", "Navmesh Project (*.voxproj.json)")
        if not path:
            return
        try:
            proj = load_project(path)
            self._project_path = path
            self._mesh_path.setText(proj.mesh_path)
            self._apply_config(proj.config)
            self._on_mesh_edited()
            self._dirty = False
            self._status.showMessage(f"Opened: {path}")
        except Exception as e:
            QMessageBox.critical(self, "Open failed", str(e))

    def _save_project(self, save_as: bool = False):
        path = self._project_path
        if save_as or not path:
            path, _ = QFileDialog.getSaveFileName(self, "Save project", "", "Navmesh Project (*.voxproj.json)")
            if not path:
                return
        cfg = self._current_config()
        proj = NavmeshProject(mesh_path=self._mesh_path.text().strip(), config=cfg)
        try:
            save_project(proj, path)
            self._project_path = path
            self._dirty = False
            self._status.showMessage(f"Saved: {path}")
        except Exception as e:
            QMessageBox.critical(self, "Save failed", str(e))

    def _confirm_discard(self) -> bool:
        if not self._dirty:
            return True
        ans = QMessageBox.question(self, "Unsaved changes", "Discard unsaved changes?")
        return ans == QMessageBox.Yes

    def closeEvent(self, event):
        if self._confirm_discard():
            if self._baked is not None:
                self._baked.close()
            if self._viewer is not None:
                self._viewer.close()
            event.accept()
        else:
            event.ignore()
