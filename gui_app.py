from __future__ import annotations

import ctypes
import math
import os
import sys
from typing import Any, Dict, Optional

if os.name == "nt":
    system_icu = os.path.join(os.environ.get("SystemRoot", r"C:\Windows"), "System32", "icuuc.dll")
    if os.path.exists(system_icu):
        try:
            ctypes.WinDLL(system_icu)
        except OSError:
            pass

try:
    from PySide6.QtCore import QPoint, QRect, QSize, Qt, QTimer, Signal
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLayout,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QScrollArea,
        QSizePolicy,
        QSlider,
        QSpinBox,
        QToolButton,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:
    raise SystemExit(f"PySide6 加载失败：{exc}") from exc

from parahand import JointDefinition, ParaHand


class FlowLayout(QLayout):
    def __init__(self, parent: Optional[QWidget] = None, margin: int = 0, spacing: int = 12):
        super().__init__(parent)
        self._items = []
        self.setContentsMargins(margin, margin, margin, margin)
        self.setSpacing(spacing)

    def addItem(self, item):
        self._items.append(item)

    def count(self) -> int:
        return len(self._items)

    def itemAt(self, index: int):
        if 0 <= index < len(self._items):
            return self._items[index]
        return None

    def takeAt(self, index: int):
        if 0 <= index < len(self._items):
            return self._items.pop(index)
        return None

    def expandingDirections(self):
        return Qt.Orientations(Qt.Orientation(0))

    def hasHeightForWidth(self) -> bool:
        return True

    def heightForWidth(self, width: int) -> int:
        return self._do_layout(QRect(0, 0, width, 0), test_only=True)

    def setGeometry(self, rect: QRect):
        super().setGeometry(rect)
        self._do_layout(rect, test_only=False)

    def sizeHint(self) -> QSize:
        return self.minimumSize()

    def minimumSize(self) -> QSize:
        size = QSize()
        for item in self._items:
            size = size.expandedTo(item.minimumSize())
        margins = self.contentsMargins()
        size += QSize(margins.left() + margins.right(), margins.top() + margins.bottom())
        return size

    def _do_layout(self, rect: QRect, test_only: bool) -> int:
        margins = self.contentsMargins()
        area = rect.adjusted(margins.left(), margins.top(), -margins.right(), -margins.bottom())
        x = area.x()
        y = area.y()
        line_height = 0
        spacing = self.spacing()

        for item in self._items:
            hint = item.sizeHint()
            next_x = x + hint.width() + spacing
            if line_height > 0 and next_x - spacing > area.right():
                x = area.x()
                y += line_height + spacing
                next_x = x + hint.width() + spacing
                line_height = 0

            if not test_only:
                item.setGeometry(QRect(QPoint(x, y), hint))

            x = next_x
            line_height = max(line_height, hint.height())

        return y + line_height - rect.y() + margins.bottom()


class JointRowWidget(QGroupBox):
    targetChanged = Signal(str, float)
    stepAdjustStarted = Signal(str, int)
    stepAdjustStopped = Signal(str)
    zeroJogStarted = Signal(str, int)
    zeroJogStopped = Signal(str)
    activeToggled = Signal(str, bool)
    configSaveRequested = Signal(str, object)

    SLIDER_SCALE = 100
    MOTOR_MODE = "motor"
    PARAHAND_MODE = "parahand"

    def __init__(
        self,
        joint_name: str,
        definition: JointDefinition,
        parent: Optional[QWidget] = None,
        display_name: Optional[str] = None,
    ):
        super().__init__(display_name or joint_name, parent)
        self.joint_name = joint_name
        self.definition = definition
        self._is_pip_dip = joint_name.endswith(".pip_dip")
        self._syncing = False
        self._target_initialized = False
        self._connected = False
        self._control_mode = self.MOTOR_MODE
        self._jog_enabled = True
        self._zero_jog_visible = False
        self._display_initialized = False
        self._display_min = 0.0
        self._display_max = 0.0
        self._display_unit_suffix = " °"
        self._display_decimals = 2
        self._display_step = 1.0

        self.minus_button = QPushButton("-")
        self.plus_button = QPushButton("+")
        self.zero_minus_button = QPushButton("-")
        self.zero_plus_button = QPushButton("+")
        self.zero_label = QLabel("调零")
        self.zero_controls_widget = QWidget()
        self.slider = QSlider(Qt.Horizontal)
        self.spinbox = QDoubleSpinBox()
        self.actual_label = QLabel("[--]")
        self.motor_target_label = QLabel("")
        self.motor_actual_angle_label = QLabel("")
        self.active_checkbox = QCheckBox()

        self.details_toggle_button = QToolButton()
        self.details_container = QWidget()
        self.status_grid_widget = QWidget()
        self.online_key_label = QLabel("online")
        self.online_value_label = QLabel("--")
        self.state_key_label = QLabel("state")
        self.state_value_label = QLabel("--")
        self.bus_key_label = QLabel("bus")
        self.bus_value_label = QLabel("--")
        self.error_key_label = QLabel("error")
        self.error_value_label = QLabel("--")
        self.speed_key_label = QLabel("speed")
        self.speed_value_label = QLabel("--")
        self.temp_key_label = QLabel("temp")
        self.temp_value_label = QLabel("--")
        self.current_key_label = QLabel("current")
        self.current_value_label = QLabel("--")
        self.extra_labels_widget = QWidget()

        self.motor_id_spinbox = QSpinBox()
        self.min_spinbox = QDoubleSpinBox()
        self.max_spinbox = QDoubleSpinBox()
        self.gui_min_mm_label = QLabel("ParaHand最小")
        self.gui_min_mm_spinbox = QDoubleSpinBox()
        self.gui_max_mm_label = QLabel("ParaHand最大")
        self.gui_max_mm_spinbox = QDoubleSpinBox()
        self.reverse_checkbox = QCheckBox("反向")
        self.save_config_button = QPushButton("保存配置")
        self.config_status_label = QLabel("已载入")
        self.config_toggle_button = QToolButton()
        self.config_container = QWidget()
        self.config_header_widget = QWidget()

        self._build_ui()
        self._connect_signals()
        self.update_definition(definition)
        self._toggle_details_panel(False)
        self._toggle_config_panel(False)
        self.set_motion_enabled(False)
        self.set_config_editable(False)

    def _build_ui(self):
        self.minus_button.setFixedWidth(32)
        self.plus_button.setFixedWidth(32)
        self.zero_minus_button.setFixedWidth(32)
        self.zero_plus_button.setFixedWidth(32)

        self.slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self.spinbox.setDecimals(2)
        self.spinbox.setSingleStep(1.0)
        self.spinbox.setKeyboardTracking(False)
        self.spinbox.setSuffix(" °")
        self.spinbox.setFixedWidth(96)
        self.spinbox.setButtonSymbols(QDoubleSpinBox.NoButtons)
        self.spinbox.setAlignment(Qt.AlignCenter)

        self.motor_id_spinbox.setRange(1, 16)
        self.motor_id_spinbox.setFixedWidth(64)

        for angle_spinbox in (self.min_spinbox, self.max_spinbox):
            angle_spinbox.setDecimals(2)
            angle_spinbox.setRange(-360.0, 360.0)
            angle_spinbox.setSingleStep(1.0)
            angle_spinbox.setKeyboardTracking(False)
            angle_spinbox.setSuffix(" °")
            angle_spinbox.setFixedWidth(96)

        for mm_spinbox in (self.gui_min_mm_spinbox, self.gui_max_mm_spinbox):
            mm_spinbox.setDecimals(2)
            mm_spinbox.setRange(-1000.0, 1000.0)
            mm_spinbox.setSingleStep(0.1)
            mm_spinbox.setKeyboardTracking(False)
            mm_spinbox.setSuffix(" mm")
            mm_spinbox.setFixedWidth(96)

        self.details_toggle_button.setText("详情")
        self.details_toggle_button.setCheckable(True)
        self.details_toggle_button.setArrowType(Qt.RightArrow)
        self.details_toggle_button.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)

        self.config_toggle_button.setText("配置")
        self.config_toggle_button.setCheckable(True)
        self.config_toggle_button.setArrowType(Qt.RightArrow)
        self.config_toggle_button.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)

        self.actual_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.motor_target_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.motor_actual_angle_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.config_status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        controls_layout = QHBoxLayout()
        controls_layout.setContentsMargins(0, 0, 0, 0)
        controls_layout.setSpacing(6)
        controls_layout.addWidget(self.active_checkbox)
        controls_layout.addWidget(self.minus_button)
        controls_layout.addWidget(self.slider, 1)
        controls_layout.addWidget(self.spinbox)
        controls_layout.addWidget(self.plus_button)
        controls_layout.addSpacing(6)
        controls_layout.addWidget(self.details_toggle_button)

        status_grid = QGridLayout()
        status_grid.setContentsMargins(0, 0, 0, 0)
        status_grid.setHorizontalSpacing(10)
        status_grid.setVerticalSpacing(4)
        status_grid.addWidget(self.online_key_label, 0, 0)
        status_grid.addWidget(self.online_value_label, 0, 1)
        status_grid.addWidget(self.state_key_label, 1, 0)
        status_grid.addWidget(self.state_value_label, 1, 1)
        status_grid.addWidget(self.bus_key_label, 1, 2)
        status_grid.addWidget(self.bus_value_label, 1, 3)
        status_grid.addWidget(self.error_key_label, 2, 0)
        status_grid.addWidget(self.error_value_label, 2, 1)
        status_grid.addWidget(self.speed_key_label, 2, 2)
        status_grid.addWidget(self.speed_value_label, 2, 3)
        status_grid.addWidget(self.temp_key_label, 3, 0)
        status_grid.addWidget(self.temp_value_label, 3, 1)
        status_grid.addWidget(self.current_key_label, 3, 2)
        status_grid.addWidget(self.current_value_label, 3, 3)
        self.status_grid_widget.setLayout(status_grid)

        extra_layout = QVBoxLayout()
        extra_layout.setContentsMargins(0, 0, 0, 0)
        extra_layout.setSpacing(2)
        extra_layout.addWidget(self.motor_target_label)
        extra_layout.addWidget(self.motor_actual_angle_label)
        self.extra_labels_widget.setLayout(extra_layout)

        zero_layout = QHBoxLayout()
        zero_layout.setContentsMargins(0, 0, 0, 0)
        zero_layout.addWidget(self.zero_label)
        zero_layout.addWidget(self.zero_minus_button)
        zero_layout.addWidget(self.zero_plus_button)
        zero_layout.addStretch()
        self.zero_controls_widget.setLayout(zero_layout)

        config_header_layout = QHBoxLayout()
        config_header_layout.setContentsMargins(0, 0, 0, 0)
        config_header_layout.addWidget(self.config_toggle_button)
        config_header_layout.addWidget(self.save_config_button)
        config_header_layout.addWidget(self.config_status_label)
        config_header_layout.addStretch()
        self.config_header_widget.setLayout(config_header_layout)

        config_layout = QVBoxLayout()
        config_layout.setContentsMargins(18, 0, 0, 0)
        config_layout.setSpacing(6)

        motor_id_layout = QHBoxLayout()
        motor_id_layout.setContentsMargins(0, 0, 0, 0)
        motor_id_layout.addWidget(QLabel("电机ID"))
        motor_id_layout.addWidget(self.motor_id_spinbox)
        motor_id_layout.addStretch()

        range_layout = QHBoxLayout()
        range_layout.setContentsMargins(0, 0, 0, 0)
        range_layout.addWidget(QLabel("最小/最大"))
        range_layout.addWidget(self.min_spinbox)
        range_layout.addWidget(QLabel("/"))
        range_layout.addWidget(self.max_spinbox)
        range_layout.addStretch()

        reverse_layout = QHBoxLayout()
        reverse_layout.setContentsMargins(0, 0, 0, 0)
        reverse_layout.addWidget(QLabel("反向"))
        reverse_layout.addWidget(self.reverse_checkbox)
        reverse_layout.addStretch()

        config_layout.addLayout(motor_id_layout)
        config_layout.addLayout(range_layout)

        if self._is_pip_dip:
            gui_range_layout = QHBoxLayout()
            gui_range_layout.setContentsMargins(0, 0, 0, 0)
            gui_range_layout.addWidget(self.gui_min_mm_label)
            gui_range_layout.addWidget(self.gui_min_mm_spinbox)
            gui_range_layout.addWidget(self.gui_max_mm_label)
            gui_range_layout.addWidget(self.gui_max_mm_spinbox)
            gui_range_layout.addStretch()
            config_layout.addLayout(gui_range_layout)
        else:
            self.gui_min_mm_label.hide()
            self.gui_min_mm_spinbox.hide()
            self.gui_max_mm_label.hide()
            self.gui_max_mm_spinbox.hide()

        config_layout.addLayout(reverse_layout)
        self.config_container.setLayout(config_layout)

        details_layout = QVBoxLayout()
        details_layout.setContentsMargins(12, 0, 0, 0)
        details_layout.setSpacing(6)
        details_layout.addWidget(self.status_grid_widget)
        details_layout.addWidget(self.extra_labels_widget)
        details_layout.addWidget(self.zero_controls_widget)
        details_layout.addWidget(self.config_header_widget)
        details_layout.addWidget(self.config_container)
        self.details_container.setLayout(details_layout)

        layout = QVBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addLayout(controls_layout)
        layout.addWidget(self.details_container)
        self.setLayout(layout)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)

    def _connect_signals(self):
        self.slider.valueChanged.connect(self._on_slider_changed)
        self.slider.sliderReleased.connect(self._on_slider_released)
        self.spinbox.editingFinished.connect(self._on_spinbox_committed)
        self.minus_button.pressed.connect(lambda: self.stepAdjustStarted.emit(self.joint_name, -1))
        self.minus_button.released.connect(lambda: self.stepAdjustStopped.emit(self.joint_name))
        self.plus_button.pressed.connect(lambda: self.stepAdjustStarted.emit(self.joint_name, 1))
        self.plus_button.released.connect(lambda: self.stepAdjustStopped.emit(self.joint_name))
        self.zero_minus_button.pressed.connect(lambda: self.zeroJogStarted.emit(self.joint_name, 2))
        self.zero_minus_button.released.connect(lambda: self.zeroJogStopped.emit(self.joint_name))
        self.zero_plus_button.pressed.connect(lambda: self.zeroJogStarted.emit(self.joint_name, 1))
        self.zero_plus_button.released.connect(lambda: self.zeroJogStopped.emit(self.joint_name))

        self.motor_id_spinbox.valueChanged.connect(self._mark_config_dirty)
        self.min_spinbox.valueChanged.connect(self._mark_config_dirty)
        self.max_spinbox.valueChanged.connect(self._mark_config_dirty)
        self.gui_min_mm_spinbox.valueChanged.connect(self._mark_config_dirty)
        self.gui_max_mm_spinbox.valueChanged.connect(self._mark_config_dirty)
        self.reverse_checkbox.toggled.connect(self._mark_config_dirty)
        self.active_checkbox.toggled.connect(self._on_active_toggled)
        self.save_config_button.clicked.connect(self._request_config_save)
        self.details_toggle_button.toggled.connect(self._toggle_details_panel)
        self.config_toggle_button.toggled.connect(self._toggle_config_panel)

    def _mark_config_dirty(self, *_args):
        if self._syncing:
            return
        self.config_status_label.setText("未保存")

    def _on_active_toggled(self, checked: bool):
        self._mark_config_dirty()
        self._apply_motion_state()
        if not checked:
            self.update_feedback({})
        if not self._syncing:
            self.activeToggled.emit(self.joint_name, checked)

    def _request_config_save(self):
        payload = {
            "motor_id": self.motor_id_spinbox.value(),
            "range_deg": [self.min_spinbox.value(), self.max_spinbox.value()],
            "reverse": self.reverse_checkbox.isChecked(),
            "enabled": self.active_checkbox.isChecked(),
            "gui_range_mm": None,
        }
        if self._is_pip_dip:
            payload["gui_range_mm"] = [self.gui_min_mm_spinbox.value(), self.gui_max_mm_spinbox.value()]
        self.configSaveRequested.emit(self.joint_name, payload)

    def _toggle_details_panel(self, expanded: bool):
        self.details_toggle_button.blockSignals(True)
        self.details_toggle_button.setChecked(expanded)
        self.details_toggle_button.setArrowType(Qt.DownArrow if expanded else Qt.RightArrow)
        self.details_toggle_button.setText("详情")
        self.details_toggle_button.blockSignals(False)
        self.details_container.setVisible(expanded)
        self.updateGeometry()

    def _toggle_config_panel(self, expanded: bool):
        self.config_toggle_button.blockSignals(True)
        self.config_toggle_button.setChecked(expanded)
        self.config_toggle_button.setArrowType(Qt.DownArrow if expanded else Qt.RightArrow)
        self.config_toggle_button.setText("配置")
        self.config_toggle_button.blockSignals(False)
        self.config_container.setVisible(expanded)
        self.updateGeometry()

    def _clamp_display_value(self, value: float) -> float:
        return max(self._display_min, min(self._display_max, float(value)))

    def _value_to_slider(self, value: float) -> int:
        return int(round(value * self.SLIDER_SCALE))

    def _slider_to_value(self, slider_value: int) -> float:
        return slider_value / self.SLIDER_SCALE

    def _on_slider_changed(self, slider_value: int):
        if self._syncing:
            return
        display_value = self._slider_to_value(slider_value)
        self._syncing = True
        self.spinbox.setValue(display_value)
        self._syncing = False

    def _on_slider_released(self):
        value = self.set_target_value(self.spinbox.value())
        self.targetChanged.emit(self.joint_name, value)

    def _on_spinbox_committed(self):
        if self._syncing:
            return
        value = self.set_target_value(self.spinbox.value())
        self.targetChanged.emit(self.joint_name, value)

    def set_display_profile(
        self,
        mode: str,
        display_min: float,
        display_max: float,
        unit_suffix: str,
        *,
        decimals: int = 2,
        step: float = 1.0,
        main_adjust_enabled: bool = True,
        zero_jog_visible: bool = False,
    ):
        current_value = self.spinbox.value() if self._target_initialized else float(display_min)
        self._syncing = True
        self._control_mode = mode
        self._display_min = float(display_min)
        self._display_max = float(display_max)
        self._display_unit_suffix = unit_suffix
        self._display_decimals = int(decimals)
        self._display_step = float(step)
        self._jog_enabled = main_adjust_enabled
        self._zero_jog_visible = bool(zero_jog_visible)
        self.spinbox.setDecimals(self._display_decimals)
        self.spinbox.setSingleStep(self._display_step)
        self.spinbox.setSuffix(self._display_unit_suffix)
        self.spinbox.setRange(self._display_min, self._display_max)
        self.slider.setRange(self._value_to_slider(self._display_min), self._value_to_slider(self._display_max))
        self.zero_controls_widget.setVisible(self._zero_jog_visible)
        self.extra_labels_widget.setVisible(self._control_mode == self.PARAHAND_MODE and self._is_pip_dip)
        self._syncing = False
        self._display_initialized = True
        self.set_target_value(current_value)
        self._apply_motion_state()

    def set_target_value(self, display_value: float) -> float:
        clamped_value = self._clamp_display_value(display_value)
        self._syncing = True
        self.spinbox.setValue(clamped_value)
        self.slider.setValue(self._value_to_slider(clamped_value))
        self._syncing = False
        self._target_initialized = True
        return clamped_value

    def get_target_value(self) -> float:
        return self.spinbox.value()

    def update_definition(self, definition: JointDefinition):
        current_target = self.spinbox.value() if self._target_initialized else 0.0
        self.definition = definition

        self._syncing = True
        self.motor_id_spinbox.setValue(definition.motor_id)
        self.min_spinbox.setValue(definition.min_deg)
        self.max_spinbox.setValue(definition.max_deg)
        self.reverse_checkbox.setChecked(definition.reverse)
        self.active_checkbox.setChecked(definition.enabled)
        self._syncing = False

        if not self._display_initialized:
            self.set_display_profile(
                self.MOTOR_MODE,
                definition.min_deg,
                definition.max_deg,
                " °",
                decimals=2,
                step=1.0,
                main_adjust_enabled=True,
                zero_jog_visible=True,
            )
        else:
            self.set_target_value(current_target)

        self.config_status_label.setText("已保存")
        self._apply_motion_state()

    def set_gui_range_mm(self, min_mm: float, max_mm: float):
        self._syncing = True
        self.gui_min_mm_spinbox.setValue(float(min_mm))
        self.gui_max_mm_spinbox.setValue(float(max_mm))
        self._syncing = False

    def _apply_motion_state(self):
        effective = self._connected and self.active_checkbox.isChecked()
        self.minus_button.setEnabled(effective and self._jog_enabled)
        self.plus_button.setEnabled(effective and self._jog_enabled)
        self.zero_minus_button.setEnabled(effective and self._zero_jog_visible)
        self.zero_plus_button.setEnabled(effective and self._zero_jog_visible)
        self.slider.setEnabled(effective)
        self.spinbox.setEnabled(effective)

    def set_motion_enabled(self, enabled: bool):
        self._connected = enabled
        self._apply_motion_state()

    def set_config_editable(self, enabled: bool):
        self.motor_id_spinbox.setEnabled(enabled)
        self.min_spinbox.setEnabled(enabled)
        self.max_spinbox.setEnabled(enabled)
        self.gui_min_mm_spinbox.setEnabled(enabled and self._is_pip_dip)
        self.gui_max_mm_spinbox.setEnabled(enabled and self._is_pip_dip)
        self.reverse_checkbox.setEnabled(enabled)
        self.save_config_button.setEnabled(enabled)
        self.config_toggle_button.setEnabled(True)
        self.details_toggle_button.setEnabled(True)
        self.active_checkbox.setEnabled(True)

    def is_joint_enabled(self) -> bool:
        return self.active_checkbox.isChecked()

    def set_actual_display_value(self, value: Optional[float], unit_suffix: Optional[str] = None):
        if not self.active_checkbox.isChecked():
            self.actual_label.setText("[未启用]")
            return
        suffix = self._display_unit_suffix if unit_suffix is None else unit_suffix
        self.actual_label.setText(f"[{self._format_display_value(value, suffix)}]")

    def set_motor_target_text(self, text: str):
        self.motor_target_label.setText(text)

    def set_motor_actual_angle_text(self, text: str):
        self.motor_actual_angle_label.setText(text)

    def update_feedback(
        self,
        feedback: Dict[str, Any],
        actual_display_value: Optional[float] = None,
        actual_unit_suffix: Optional[str] = None,
    ):
        if not self.active_checkbox.isChecked():
            self.actual_label.setText("[未启用]")
            self.motor_target_label.setText("")
            self.motor_actual_angle_label.setText("")
            self.online_value_label.setText("未启用")
            self.state_value_label.setText("--")
            self.bus_value_label.setText("--")
            self.error_value_label.setText("--")
            self.speed_value_label.setText("--")
            self.temp_value_label.setText("--")
            self.current_value_label.setText("--")
            return

        self.set_actual_display_value(actual_display_value, actual_unit_suffix)
        self.online_value_label.setText(self._format_bool(feedback.get('online')))
        self.state_value_label.setText(self._format_value(feedback.get('fsm_state')))
        self.bus_value_label.setText(self._format_number(feedback.get('bus_voltage'), 'V'))
        self.error_value_label.setText(self._format_value(feedback.get('error_code')))
        self.speed_value_label.setText(self._format_number(feedback.get('speed_pct'), '%'))
        self.temp_value_label.setText(self._format_number(feedback.get('temp'), '°C'))
        self.current_value_label.setText(self._format_number(feedback.get('current_mA'), 'mA'))

    def _format_display_value(self, value: Any, suffix: str) -> str:
        if value is None:
            return "--"
        return f"{float(value):.2f}{suffix}"

    def _format_bool(self, value: Any) -> str:
        if value is None:
            return "--"
        return "在线" if bool(value) else "离线"

    def _format_number(self, value: Any, suffix: str) -> str:
        if value is None:
            return f"-- {suffix}"
        return f"{float(value):.2f} {suffix}"

    def _format_value(self, value: Any) -> str:
        if value is None:
            return "--"
        return str(value)


class MainWindow(QMainWindow):
    MOTOR_MODE = "motor"
    PARAHAND_MODE = "parahand"
    DEFAULT_PIP_DIP_GUI_RANGE_MM = (0.0, 25.0)

    def __init__(self, config_path: Optional[str] = None):
        super().__init__()
        self.hand = ParaHand(config_path)
        self.row_widgets: Dict[str, JointRowWidget] = {}
        self.control_mode = self.MOTOR_MODE
        self.hand_joint_order: list[str] = []
        self._motor_target_values_deg: Dict[str, float] = {}
        self._hand_target_positions: Dict[str, float] = {}
        self._feedback_initialized = False
        self._refresh_hand_joint_order()

        self._syncing_connection_config = False
        self._connection_config_dirty = False
        self.feedback_timer = QTimer(self)
        self.feedback_timer.setInterval(100)
        self.feedback_timer.timeout.connect(self._refresh_feedback)
        self.step_repeat_timer = QTimer(self)
        self.step_repeat_timer.setInterval(50)
        self.step_repeat_timer.timeout.connect(self._repeat_step_adjust)
        self._active_step_adjust_joint: Optional[str] = None
        self._active_step_adjust_direction = 0

        self.connect_button = QPushButton("连接")
        self.disconnect_button = QPushButton("断开")
        self.enable_checkbox = QCheckBox("全局使能")
        self.zero_button = QPushButton("置零")
        self.mode_label = QLabel("控制模式")
        self.control_mode_combo = QComboBox()
        self.control_mode_combo.addItem("电机模式", self.MOTOR_MODE)
        self.control_mode_combo.addItem("ParaHand模式", self.PARAHAND_MODE)
        self.status_label = QLabel("未连接")
        self.config_label = QLabel(f"配置: {self.hand.config_path}")
        self.config_help_label = QLabel(
            "顶部可切换电机模式与 ParaHand 模式；ParaHand 模式下普通关节显示角度，pip_dip 显示毫米并统一走 set_hand_positions。"
        )
        self.step_hint_label = QLabel("说明：ParaHand 模式下，pip_dip 关节的点动步长固定为 gui_step 的 0.5 倍。")
        self.connection_config_toggle_button = QToolButton()
        self.connection_config_header_widget = QWidget()
        self.connection_config_container = QWidget()
        self.connection_config_box = QGroupBox()
        self.ctrl_frequency_spinbox = QDoubleSpinBox()
        self.port_input = QLineEdit()
        self.baudrate_spinbox = QSpinBox()
        self.timeout_spinbox = QDoubleSpinBox()
        self.write_timeout_spinbox = QDoubleSpinBox()
        self.gui_step_spinbox = QDoubleSpinBox()
        self.save_connection_config_button = QPushButton("保存控制器配置")
        self.connection_config_status_label = QLabel("已载入")

        self.empty_label = QLabel("config.yaml 中还没有有效的 joint 配置，请先填写 id / range / reverse。")
        self.scroll_area = QScrollArea()
        self.rows_container = QWidget()
        self.rows_layout = QVBoxLayout()

        self._build_ui()
        self._toggle_connection_config_panel(False)
        self._load_connection_config_fields()
        self._connect_signals()
        self._populate_joint_rows()
        self._refresh_hand_joint_order()
        self._set_connection_state(False)
        self._apply_control_mode_to_rows()
        self._set_status(f"就绪 | 配置: {self.hand.config_path}")

    def _build_ui(self):
        self.setWindowTitle("ParaHand GUI")
        self.resize(1200, 820)

        central_widget = QWidget()
        main_layout = QVBoxLayout(central_widget)

        toolbar_layout = QHBoxLayout()
        toolbar_layout.addWidget(self.connect_button)
        toolbar_layout.addWidget(self.disconnect_button)
        toolbar_layout.addWidget(self.enable_checkbox)
        toolbar_layout.addWidget(self.mode_label)
        toolbar_layout.addWidget(self.control_mode_combo)
        toolbar_layout.addStretch()

        status_layout = QHBoxLayout()
        status_layout.setContentsMargins(0, 0, 0, 0)
        status_layout.addWidget(self.status_label)
        status_layout.addStretch()

        self.config_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.config_label.setWordWrap(True)
        self.status_label.setWordWrap(True)
        self.config_help_label.setWordWrap(True)
        self.step_hint_label.setWordWrap(True)
        self.empty_label.setWordWrap(True)

        self.ctrl_frequency_spinbox.setDecimals(2)
        self.ctrl_frequency_spinbox.setRange(0.01, 1000.0)
        self.ctrl_frequency_spinbox.setSingleStep(1.0)
        self.ctrl_frequency_spinbox.setKeyboardTracking(False)
        self.ctrl_frequency_spinbox.setSuffix(" Hz")
        self.ctrl_frequency_spinbox.setFixedWidth(100)

        self.port_input.setClearButtonEnabled(True)
        self.port_input.setMinimumWidth(100)

        self.baudrate_spinbox.setRange(1, 10_000_000)
        self.baudrate_spinbox.setSingleStep(100)
        self.baudrate_spinbox.setFixedWidth(110)

        for timeout_spinbox in (self.timeout_spinbox, self.write_timeout_spinbox):
            timeout_spinbox.setDecimals(3)
            timeout_spinbox.setRange(0.0, 60.0)
            timeout_spinbox.setSingleStep(0.01)
            timeout_spinbox.setKeyboardTracking(False)
            timeout_spinbox.setSuffix(" s")
            timeout_spinbox.setFixedWidth(100)

        self.gui_step_spinbox.setDecimals(2)
        self.gui_step_spinbox.setRange(0.01, 1000.0)
        self.gui_step_spinbox.setSingleStep(0.5)
        self.gui_step_spinbox.setKeyboardTracking(False)
        self.gui_step_spinbox.setValue(1.0)
        self.gui_step_spinbox.setSuffix(" step")
        self.gui_step_spinbox.setFixedWidth(100)

        self.connection_config_toggle_button.setText("控制器配置")
        self.connection_config_toggle_button.setCheckable(True)
        self.connection_config_toggle_button.setArrowType(Qt.RightArrow)
        self.connection_config_toggle_button.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.connection_config_status_label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.addWidget(self.connection_config_toggle_button)
        header_layout.addStretch()
        self.connection_config_header_widget.setLayout(header_layout)

        connection_grid = QGridLayout()
        connection_grid.setContentsMargins(12, 0, 0, 0)
        connection_grid.setHorizontalSpacing(16)
        connection_grid.setVerticalSpacing(8)
        connection_grid.addWidget(QLabel("控制频率"), 0, 0)
        connection_grid.addWidget(self.ctrl_frequency_spinbox, 0, 1)
        connection_grid.addWidget(QLabel("timeout_s"), 0, 2)
        connection_grid.addWidget(self.timeout_spinbox, 0, 3)
        connection_grid.addWidget(QLabel("port"), 1, 0)
        connection_grid.addWidget(self.port_input, 1, 1)
        connection_grid.addWidget(QLabel("write_timeout_s"), 1, 2)
        connection_grid.addWidget(self.write_timeout_spinbox, 1, 3)
        connection_grid.addWidget(QLabel("baudrate"), 2, 0)
        connection_grid.addWidget(self.baudrate_spinbox, 2, 1)
        connection_grid.addWidget(QLabel("jog步长"), 2, 2)
        connection_grid.addWidget(self.gui_step_spinbox, 2, 3)

        footer_layout = QHBoxLayout()
        footer_layout.setContentsMargins(12, 0, 0, 0)
        footer_layout.addWidget(self.save_connection_config_button)
        footer_layout.addWidget(self.connection_config_status_label)
        footer_layout.addStretch()

        container_layout = QVBoxLayout()
        container_layout.setContentsMargins(0, 0, 0, 0)
        container_layout.setSpacing(6)
        container_layout.addLayout(connection_grid)
        container_layout.addLayout(footer_layout)
        self.connection_config_container.setLayout(container_layout)

        box_layout = QVBoxLayout()
        box_layout.setContentsMargins(8, 8, 8, 8)
        box_layout.setSpacing(6)
        box_layout.addWidget(self.connection_config_header_widget)
        box_layout.addWidget(self.connection_config_container)
        self.connection_config_box.setLayout(box_layout)

        self.rows_layout.setContentsMargins(0, 0, 0, 0)
        self.rows_layout.setSpacing(12)
        self.rows_container.setLayout(self.rows_layout)

        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setWidget(self.rows_container)

        main_layout.addLayout(toolbar_layout)
        main_layout.addLayout(status_layout)
        main_layout.addWidget(self.config_label)
        main_layout.addWidget(self.config_help_label)
        main_layout.addWidget(self.step_hint_label)
        main_layout.addWidget(self.connection_config_box)
        main_layout.addWidget(self.scroll_area)
        self.setCentralWidget(central_widget)

    def _connect_signals(self):
        self.connect_button.clicked.connect(self._connect_device)
        self.disconnect_button.clicked.connect(self._disconnect_device)
        self.enable_checkbox.toggled.connect(self._toggle_enable)
        self.control_mode_combo.currentIndexChanged.connect(self._on_control_mode_changed)
        self.connection_config_toggle_button.toggled.connect(self._toggle_connection_config_panel)
        self.ctrl_frequency_spinbox.valueChanged.connect(self._mark_connection_config_dirty)
        self.port_input.textChanged.connect(self._mark_connection_config_dirty)
        self.baudrate_spinbox.valueChanged.connect(self._mark_connection_config_dirty)
        self.timeout_spinbox.valueChanged.connect(self._mark_connection_config_dirty)
        self.write_timeout_spinbox.valueChanged.connect(self._mark_connection_config_dirty)
        self.save_connection_config_button.clicked.connect(self._save_connection_config)
        self.gui_step_spinbox.valueChanged.connect(self._apply_gui_step_to_rows)

    def _toggle_connection_config_panel(self, expanded: bool):
        self.connection_config_toggle_button.blockSignals(True)
        self.connection_config_toggle_button.setChecked(expanded)
        self.connection_config_toggle_button.setArrowType(Qt.DownArrow if expanded else Qt.RightArrow)
        self.connection_config_toggle_button.setText("控制器配置")
        self.connection_config_toggle_button.blockSignals(False)
        self.connection_config_container.setVisible(expanded)

    def _load_connection_config_fields(self, status_text: str = "已载入"):
        self._syncing_connection_config = True
        self.ctrl_frequency_spinbox.setValue(float(self.hand.ctrl_frequency))
        self.port_input.setText(self.hand.motor.port)
        self.baudrate_spinbox.setValue(int(self.hand.motor.baudrate))
        self.timeout_spinbox.setValue(float(self.hand.motor.timeout_s))
        self.write_timeout_spinbox.setValue(float(self.hand.motor.write_timeout_s))
        self._syncing_connection_config = False
        self._connection_config_dirty = False
        self.connection_config_status_label.setText(status_text)

    def _mark_connection_config_dirty(self, *_args):
        if self._syncing_connection_config:
            return
        self._connection_config_dirty = True
        self.connection_config_status_label.setText("未保存")

    def _set_connection_config_editable(self, enabled: bool):
        self.ctrl_frequency_spinbox.setEnabled(enabled)
        self.port_input.setEnabled(enabled)
        self.baudrate_spinbox.setEnabled(enabled)
        self.timeout_spinbox.setEnabled(enabled)
        self.write_timeout_spinbox.setEnabled(enabled)
        self.save_connection_config_button.setEnabled(enabled)

    def _save_connection_config(self):
        if self.hand.connected:
            QMessageBox.information(self, "请先断开", "修改控制器配置前请先断开设备连接。")
            return

        try:
            self.hand.update_connection_config(
                self.ctrl_frequency_spinbox.value(),
                self.port_input.text(),
                self.baudrate_spinbox.value(),
                self.timeout_spinbox.value(),
                self.write_timeout_spinbox.value(),
            )
            self.hand.save_config()
            self._load_connection_config_fields("已保存")
            self._set_status("控制器配置已保存")
        except Exception as exc:
            QMessageBox.critical(self, "保存控制器配置失败", str(exc))
            self._set_status(f"保存控制器配置失败: {exc}")

    def _format_motor_angle_text(self, angle_deg: Optional[float]) -> str:
        if angle_deg is None:
            return "--"
        return f"{float(angle_deg):.2f} °"

    def _compute_pip_dip_motor_target_deg(self, joint_name: str) -> Optional[float]:
        if joint_name not in self._hand_target_positions:
            return None
        finger_name, _ = self.hand._split_joint_name(joint_name)
        mcp_2_name = f"{finger_name}.mcp_2"
        mcp_2_rad = self._hand_target_positions.get(mcp_2_name)
        pip_dip_m = self._hand_target_positions.get(joint_name)
        if mcp_2_rad is None or pip_dip_m is None:
            return None
        return self.hand._conpensated_pip_dip(float(mcp_2_rad), float(pip_dip_m))

    def _refresh_hand_joint_order(self):
        previous_motor_targets = getattr(self, "_motor_target_values_deg", {})
        previous_hand_targets = getattr(self, "_hand_target_positions", {})
        self.hand_joint_order = self.hand.get_hand_joint_order()
        self._motor_target_values_deg = {
            joint_name: float(previous_motor_targets.get(joint_name, 0.0))
            for joint_name in self.hand_joint_order
        }
        self._hand_target_positions = {
            joint_name: float(previous_hand_targets.get(joint_name, 0.0))
            for joint_name in self.hand_joint_order
        }

    def _read_gui_range_mm(self, joint_name: str) -> tuple[float, float]:
        finger_config, local_joint_name = self.hand._get_joint_config_entry(self.hand.config, joint_name)
        joint_config = finger_config.get(local_joint_name)
        default_min, default_max = self.DEFAULT_PIP_DIP_GUI_RANGE_MM
        if not isinstance(joint_config, dict):
            return default_min, default_max
        gui_range_mm = joint_config.get("gui_range_mm")
        if not isinstance(gui_range_mm, (list, tuple)) or len(gui_range_mm) != 2:
            return default_min, default_max
        min_mm = float(gui_range_mm[0])
        max_mm = float(gui_range_mm[1])
        if min_mm > max_mm:
            return default_min, default_max
        return min_mm, max_mm

    def _write_gui_range_mm(self, joint_name: str, gui_range_mm: Optional[list[float]]):
        finger_config, local_joint_name = self.hand._get_joint_config_entry(self.hand.config, joint_name)
        joint_config = finger_config.get(local_joint_name)
        if not isinstance(joint_config, dict):
            raise KeyError(f"未找到关节映射: {joint_name}")
        if gui_range_mm is None:
            joint_config.pop("gui_range_mm", None)
            return
        if len(gui_range_mm) != 2:
            raise ValueError("gui_range_mm 必须是长度为2的列表")
        min_mm = float(gui_range_mm[0])
        max_mm = float(gui_range_mm[1])
        if min_mm > max_mm:
            raise ValueError("gui_range_mm 顺序无效")
        joint_config["gui_range_mm"] = [min_mm, max_mm]

    def _display_value_to_hand_position(self, joint_name: str, display_value: float) -> float:
        if self.hand.is_pip_dip_joint(joint_name):
            return float(display_value) / 1000.0
        return math.radians(float(display_value))

    def _hand_position_to_display_value(self, joint_name: str, position_value: Optional[float]) -> Optional[float]:
        if position_value is None:
            return None
        if self.hand.is_pip_dip_joint(joint_name):
            return float(position_value) * 1000.0
        return math.degrees(float(position_value))

    def _inverse_compensated_pip_dip(self, mcp_2_angle_rad: float, pip_dip_deg: float) -> float:
        return (
            ((float(pip_dip_deg) + 255.0) * 5.0 * math.pi / 180.0)
            + math.sqrt(587.75 - 378.98 * math.cos(2 - float(mcp_2_angle_rad)))
            - 27.23
        ) / 1000.0

    def _joint_target_map_to_hand_positions(self, targets_deg: Dict[str, Any]) -> Dict[str, float]:
        positions: Dict[str, float] = {}
        for joint_name in self.hand_joint_order:
            if joint_name not in targets_deg:
                raise KeyError(f"缺少关节 {joint_name} 的角度输入")
            joint_value_deg = float(targets_deg[joint_name])
            if self.hand.is_pip_dip_joint(joint_name):
                finger_name, _ = self.hand._split_joint_name(joint_name)
                mcp_2_name = f"{finger_name}.mcp_2"
                if mcp_2_name not in targets_deg:
                    raise KeyError(f"未找到 {joint_name} 对应的 {mcp_2_name} 角度输入")
                positions[joint_name] = self._inverse_compensated_pip_dip(
                    math.radians(float(targets_deg[mcp_2_name])),
                    joint_value_deg,
                )
            else:
                positions[joint_name] = math.radians(joint_value_deg)
        return positions

    def _get_hand_positions_feedback(self, joint_feedback: Dict[str, Dict[str, Any]]) -> Dict[str, Optional[float]]:
        positions: Dict[str, Optional[float]] = {}
        for joint_name in self.hand_joint_order:
            definition = self.hand.joint_to_motor[joint_name]
            if not definition.enabled:
                positions[joint_name] = None
                continue

            joint_position_deg = joint_feedback.get(joint_name, {}).get("position_deg")
            if joint_position_deg is None:
                positions[joint_name] = None
                continue

            if self.hand.is_pip_dip_joint(joint_name):
                finger_name, _ = self.hand._split_joint_name(joint_name)
                mcp_2_name = f"{finger_name}.mcp_2"
                mcp_2_deg = joint_feedback.get(mcp_2_name, {}).get("position_deg")
                if mcp_2_deg is None:
                    positions[joint_name] = None
                    continue
                positions[joint_name] = self._inverse_compensated_pip_dip(
                    math.radians(float(mcp_2_deg)),
                    float(joint_position_deg),
                )
            else:
                positions[joint_name] = math.radians(float(joint_position_deg))
        return positions

    def _sync_hand_targets_from_motor_targets(self):
        if not self.hand_joint_order:
            return
        self._hand_target_positions = self._joint_target_map_to_hand_positions(self._motor_target_values_deg)

    def _sync_motor_targets_from_hand_targets(self):
        if not self.hand_joint_order:
            return
        self._motor_target_values_deg = self.hand.hand_position_map_to_joint_targets_deg(self._hand_target_positions)

    def _display_target_for_joint(self, joint_name: str) -> float:
        if self.control_mode == self.PARAHAND_MODE:
            display_value = self._hand_position_to_display_value(
                joint_name,
                self._hand_target_positions.get(joint_name),
            )
            return 0.0 if display_value is None else float(display_value)
        return float(self._motor_target_values_deg.get(joint_name, 0.0))

    def _group_joints_by_finger(self) -> Dict[str, list[tuple[str, str, JointDefinition]]]:
        grouped: Dict[str, list[tuple[str, str, JointDefinition]]] = {}
        for joint_name, definition in self.hand.joint_to_motor.items():
            finger_name, local_joint_name = joint_name.split(".", 1)
            grouped.setdefault(finger_name, []).append((joint_name, local_joint_name, definition))
        return grouped

    def _populate_joint_rows(self):
        self.row_widgets.clear()

        while self.rows_layout.count():
            item = self.rows_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

        if not self.hand.joint_to_motor:
            self.rows_layout.addWidget(self.empty_label)
            self.rows_layout.addStretch()
            return

        self.empty_label.hide()
        for finger_name, joints in self._group_joints_by_finger().items():
            finger_box = QGroupBox(finger_name)
            finger_layout = QVBoxLayout()
            finger_layout.setContentsMargins(8, 8, 8, 8)
            finger_layout.setSpacing(8)

            for joint_name, local_joint_name, definition in joints:
                row = JointRowWidget(joint_name, definition, display_name=local_joint_name)
                if self.hand.is_pip_dip_joint(joint_name):
                    row.set_gui_range_mm(*self._read_gui_range_mm(joint_name))
                row.targetChanged.connect(self._set_joint_target)
                row.stepAdjustStarted.connect(self._start_step_adjust)
                row.stepAdjustStopped.connect(self._stop_step_adjust)
                row.zeroJogStarted.connect(self._start_joint_jog)
                row.zeroJogStopped.connect(self._stop_joint_jog)
                row.activeToggled.connect(self._set_joint_enabled)
                row.configSaveRequested.connect(self._save_joint_config)
                self.row_widgets[joint_name] = row
                finger_layout.addWidget(row)

            finger_layout.addStretch()
            finger_box.setLayout(finger_layout)
            self.rows_layout.addWidget(finger_box)
        self.rows_layout.addStretch()

    def _apply_control_mode_to_rows(self):
        if not self.row_widgets:
            return

        for joint_name, row in self.row_widgets.items():
            row.set_motor_target_text("")
            row.set_motor_actual_angle_text("")
            definition = self.hand.joint_to_motor[joint_name]
            if self.control_mode == self.MOTOR_MODE:
                clamped_value = row.set_display_profile(
                    self.MOTOR_MODE,
                    definition.min_deg,
                    definition.max_deg,
                    " °",
                    decimals=2,
                    step=self.gui_step_spinbox.value(),
                    main_adjust_enabled=True,
                    zero_jog_visible=True,
                )
                row.set_target_value(self._motor_target_values_deg.get(joint_name, clamped_value))
                if not self.hand.connected:
                    row.set_actual_display_value(None, " °")
                self._motor_target_values_deg[joint_name] = row.get_target_value()
                continue

            if self.hand.is_pip_dip_joint(joint_name):
                display_min, display_max = self._read_gui_range_mm(joint_name)
                row.set_gui_range_mm(display_min, display_max)
                unit_suffix = " mm"
                motor_target_deg = self._compute_pip_dip_motor_target_deg(joint_name)
                row.set_motor_target_text(f"电机Target: {self._format_motor_angle_text(motor_target_deg)}")
            else:
                display_min = definition.min_deg
                display_max = definition.max_deg
                unit_suffix = " °"

            row.set_display_profile(
                self.PARAHAND_MODE,
                display_min,
                display_max,
                unit_suffix,
                decimals=2,
                step=self.gui_step_spinbox.value(),
                main_adjust_enabled=True,
                zero_jog_visible=False,
            )
            row.set_target_value(self._display_target_for_joint(joint_name))
            if not self.hand.connected:
                row.set_actual_display_value(None, unit_suffix)
            self._hand_target_positions[joint_name] = self._display_value_to_hand_position(
                joint_name,
                row.get_target_value(),
            )

    def _on_control_mode_changed(self, _index: int):
        new_mode = self.control_mode_combo.currentData()
        if new_mode == self.control_mode:
            return
        self.control_mode = self.MOTOR_MODE if new_mode is None else str(new_mode)
        self._apply_control_mode_to_rows()
        if self.hand.connected:
            self._refresh_feedback()

    def _apply_gui_step_to_rows(self, *_args):
        if self.row_widgets:
            self._apply_control_mode_to_rows()

    def _get_gui_step(self, joint_name: Optional[str] = None) -> float:
        step_value = float(self.gui_step_spinbox.value())
        if joint_name and self.control_mode == self.PARAHAND_MODE and self.hand.is_pip_dip_joint(joint_name):
            return step_value * 0.5
        return step_value

    def _apply_step_adjust(self, joint_name: str, direction: int):
        row = self.row_widgets.get(joint_name)
        if row is None:
            return
        current_value = row.get_target_value()
        target_value = current_value + direction * self._get_gui_step(joint_name)
        clamped_value = row.set_target_value(target_value)
        self._set_joint_target(joint_name, clamped_value)

    def _start_step_adjust(self, joint_name: str, direction: int):
        if not self.hand.connected:
            self._set_status("请先连接设备")
            return
        if not self.row_widgets[joint_name].is_joint_enabled():
            self._set_status(f"{joint_name} 未启用，已跳过控制")
            return
        self._active_step_adjust_joint = joint_name
        self._active_step_adjust_direction = int(direction)
        self._apply_step_adjust(joint_name, direction)
        self.step_repeat_timer.start()

    def _repeat_step_adjust(self):
        if not self._active_step_adjust_joint or self._active_step_adjust_direction == 0:
            return
        self._apply_step_adjust(self._active_step_adjust_joint, self._active_step_adjust_direction)

    def _stop_step_adjust(self, joint_name: str):
        if self._active_step_adjust_joint == joint_name:
            self.step_repeat_timer.stop()
            self._active_step_adjust_joint = None
            self._active_step_adjust_direction = 0

    def _set_connection_state(self, connected: bool):
        self.connect_button.setEnabled(not connected)
        self.disconnect_button.setEnabled(connected)
        self.enable_checkbox.setEnabled(connected)
        self._set_connection_config_editable(not connected)
        for row in self.row_widgets.values():
            row.set_motion_enabled(connected)
            row.set_config_editable(not connected)
        if not connected:
            self.enable_checkbox.blockSignals(True)
            self.enable_checkbox.setChecked(False)
            self.enable_checkbox.blockSignals(False)

    def _set_status(self, message: str):
        self.status_label.setText(message)

    def _connect_device(self):
        if self._connection_config_dirty:
            QMessageBox.information(self, "请先保存", "连接前请先保存控制器配置。")
            self._set_status("控制器配置尚未保存")
            return

        self._feedback_initialized = False
        try:
            if not self.hand.connect():
                self._set_status("连接失败")
                QMessageBox.warning(self, "连接失败", "未能连接到电机控制系统。")
                return
            if self.row_widgets:
                self.hand.start_polling()
                self.feedback_timer.start()
                self._refresh_feedback()
            self._set_connection_state(True)
            self._set_status("已连接")
        except Exception as exc:
            self._set_status(f"连接失败: {exc}")
            QMessageBox.critical(self, "连接失败", str(exc))

    def _disconnect_device(self):
        self.feedback_timer.stop()
        self.step_repeat_timer.stop()
        self._active_step_adjust_joint = None
        self._active_step_adjust_direction = 0
        try:
            if self.hand.connected:
                self.hand.disconnect()
        except Exception as exc:
            QMessageBox.critical(self, "断开失败", str(exc))
        finally:
            self._feedback_initialized = False
            self._set_connection_state(False)
            self._apply_control_mode_to_rows()
            self._set_status("已断开")

    def _toggle_enable(self, enabled: bool):
        if not self.hand.connected:
            return
        try:
            if enabled:
                self.hand.enable()
                self._set_status("已发送全局使能")
            else:
                self.hand.disable()
                self._set_status("已发送全局失能")
        except Exception as exc:
            self.enable_checkbox.blockSignals(True)
            self.enable_checkbox.setChecked(not enabled)
            self.enable_checkbox.blockSignals(False)
            QMessageBox.critical(self, "使能失败", str(exc))
            self._set_status(f"使能失败: {exc}")

    def _show_zero_placeholder(self):
        QMessageBox.information(self, "置零", "调零请使用每个关节详情中的 -/+ 按钮。")

    def _set_joint_enabled(self, joint_name: str, enabled: bool):
        previous_definition = self.hand.joint_to_motor.get(joint_name)
        if previous_definition is None:
            return

        try:
            self.hand.set_joint_enabled(joint_name, enabled)
            self.row_widgets[joint_name].definition = self.hand.joint_to_motor[joint_name]
            self.row_widgets[joint_name]._apply_motion_state()
            if self.hand.connected:
                if enabled:
                    if self.enable_checkbox.isChecked():
                        self.hand.enable([joint_name])
                else:
                    self.hand.disable([joint_name])
            state_text = "启用" if enabled else "停用"
            self._set_status(f"{joint_name} 已{state_text}（未保存）")
        except Exception as exc:
            self.hand.set_joint_enabled(joint_name, previous_definition.enabled)
            row = self.row_widgets[joint_name]
            row.active_checkbox.blockSignals(True)
            row.active_checkbox.setChecked(previous_definition.enabled)
            row.active_checkbox.blockSignals(False)
            row.definition = previous_definition
            row._apply_motion_state()
            QMessageBox.critical(self, "更新启用状态失败", str(exc))
            self._set_status(f"更新启用状态失败: {exc}")

    def _save_joint_config(self, joint_name: str, config_values: Any):
        if self.hand.connected:
            QMessageBox.information(self, "请先断开", "修改配置前请先断开设备连接。")
            return

        try:
            gui_range_mm = config_values.get("gui_range_mm") if isinstance(config_values, dict) else None
            if self.hand.is_pip_dip_joint(joint_name):
                self._write_gui_range_mm(joint_name, gui_range_mm)
            self.hand.update_joint_definition(joint_name, config_values)
            self.hand.save_config()
            self.row_widgets[joint_name].update_definition(self.hand.joint_to_motor[joint_name])
            if self.hand.is_pip_dip_joint(joint_name):
                self.row_widgets[joint_name].set_gui_range_mm(*self._read_gui_range_mm(joint_name))
            self._refresh_hand_joint_order()
            self._apply_control_mode_to_rows()
            self._set_status(f"{joint_name} 配置已保存")
        except Exception as exc:
            QMessageBox.critical(self, "保存配置失败", str(exc))
            self._set_status(f"保存配置失败: {exc}")

    def _set_joint_target(self, joint_name: str, display_value: float):
        if not self.hand.connected:
            self._set_status("请先连接设备")
            return
        if not self.row_widgets[joint_name].is_joint_enabled():
            self._set_status(f"{joint_name} 未启用，已跳过控制")
            return

        try:
            if self.control_mode == self.MOTOR_MODE:
                angle_deg = float(display_value)
                resolved_targets = self.hand.resolve_joint_targets({joint_name: angle_deg})
                self.hand._dispatch_joint_positions(resolved_targets)
                self._apply_resolved_joint_targets_to_gui(resolved_targets)
                self._set_status(f"已发送 {joint_name} -> {resolved_targets[joint_name]:.2f} °")
                return

            if not self._feedback_initialized:
                self.row_widgets[joint_name].set_target_value(self._display_target_for_joint(joint_name))
                self._set_status("ParaHand 模式等待反馈初始化完成")
                return

            self._hand_target_positions[joint_name] = self._display_value_to_hand_position(joint_name, display_value)
            requested_targets_deg = self.hand.hand_position_map_to_joint_targets_deg(self._hand_target_positions)
            resolved_targets = self.hand.resolve_joint_targets(requested_targets_deg)
            self._apply_resolved_joint_targets_to_gui(resolved_targets)
            full_positions = [self._hand_target_positions[name] for name in self.hand_joint_order]
            self.hand.set_hand_positions(full_positions)
            unit_suffix = " mm" if self.hand.is_pip_dip_joint(joint_name) else " °"
            self._set_status(f"已发送 {joint_name} -> {self._display_target_for_joint(joint_name):.2f}{unit_suffix}")
        except Exception as exc:
            QMessageBox.critical(self, "发送目标失败", str(exc))
            self._set_status(f"发送目标失败: {exc}")

    def _apply_resolved_joint_targets_to_gui(self, resolved_targets_deg: Dict[str, float]):
        if not resolved_targets_deg:
            return
        self._motor_target_values_deg.update({joint_name: float(value) for joint_name, value in resolved_targets_deg.items()})
        self._sync_hand_targets_from_motor_targets()
        for joint_name in resolved_targets_deg:
            row = self.row_widgets.get(joint_name)
            if row is None:
                continue
            row.set_target_value(self._display_target_for_joint(joint_name))
            if self.control_mode == self.PARAHAND_MODE and self.hand.is_pip_dip_joint(joint_name):
                row.set_motor_target_text(
                    f"电机Target: {self._format_motor_angle_text(self._compute_pip_dip_motor_target_deg(joint_name))}"
                )

    def _start_joint_jog(self, joint_name: str, direction: int):
        if self.control_mode != self.MOTOR_MODE:
            self._set_status("ParaHand 模式下不支持点动")
            return
        if not self.hand.connected:
            self._set_status("请先连接设备")
            return
        if not self.row_widgets[joint_name].is_joint_enabled():
            self._set_status(f"{joint_name} 未启用，已跳过点动")
            return
        try:
            if self.control_mode != self.MOTOR_MODE:
                self._set_status("调零仅在电机模式下可用")
                return
            self.hand.jog_joint(joint_name, direction)
            sign = "+" if direction == 1 else "-"
            self._set_status(f"{joint_name} 调零 {sign}")
        except Exception as exc:
            QMessageBox.critical(self, "点动失败", str(exc))
            self._set_status(f"点动失败: {exc}")

    def _stop_joint_jog(self, joint_name: str):
        if self.control_mode != self.MOTOR_MODE:
            return
        if not self.hand.connected or not self.row_widgets[joint_name].is_joint_enabled():
            return
        try:
            self.hand.jog_joint(joint_name, 0)
            self._set_status(f"{joint_name} 调零停止")
        except Exception as exc:
            QMessageBox.critical(self, "停止点动失败", str(exc))
            self._set_status(f"停止点动失败: {exc}")

    def _all_enabled_hand_feedback_ready(self, hand_feedback: Dict[str, Optional[float]]) -> bool:
        for joint_name, definition in self.hand.joint_to_motor.items():
            if not definition.enabled:
                continue
            if hand_feedback.get(joint_name) is None:
                return False
        return True

    def _refresh_feedback(self):
        if not self.hand.connected or not self.row_widgets:
            return
        try:
            joint_feedback = self.hand.get_joint_feedback()
            hand_feedback = self._get_hand_positions_feedback(joint_feedback)
        except Exception as exc:
            self._set_status(f"读取反馈失败: {exc}")
            return

        if not self._feedback_initialized and self._all_enabled_hand_feedback_ready(hand_feedback):
            for joint_name in self.hand_joint_order:
                position_deg = joint_feedback.get(joint_name, {}).get("position_deg")
                if position_deg is not None:
                    self._motor_target_values_deg[joint_name] = float(position_deg)
                hand_position = hand_feedback.get(joint_name)
                if hand_position is not None:
                    self._hand_target_positions[joint_name] = float(hand_position)
            self._feedback_initialized = True
            self._apply_control_mode_to_rows()

        for joint_name, row in self.row_widgets.items():
            if self.control_mode == self.MOTOR_MODE:
                actual_value = joint_feedback.get(joint_name, {}).get("position_deg")
                actual_unit_suffix = " °"
            else:
                actual_value = self._hand_position_to_display_value(joint_name, hand_feedback.get(joint_name))
                actual_unit_suffix = " mm" if self.hand.is_pip_dip_joint(joint_name) else " °"
                if self.hand.is_pip_dip_joint(joint_name):
                    row.set_motor_target_text(
                        f"电机Target: {self._format_motor_angle_text(self._compute_pip_dip_motor_target_deg(joint_name))}"
                    )
                    row.set_motor_actual_angle_text(
                        f"实际电机角度: {self._format_motor_angle_text(joint_feedback.get(joint_name, {}).get('position_deg'))}"
                    )
            row.update_feedback(
                joint_feedback.get(joint_name, {}),
                actual_display_value=actual_value,
                actual_unit_suffix=actual_unit_suffix,
            )

    def closeEvent(self, event):
        self._disconnect_device()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    try:
        window = MainWindow()
    except Exception as exc:
        QMessageBox.critical(None, "启动失败", str(exc))
        return 1
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
