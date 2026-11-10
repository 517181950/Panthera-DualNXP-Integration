"""Tk tuning application for the dual-arm serial Gello reader."""

from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk
from typing import Callable

from .config import (
    ArmConfig,
    AxisConfig,
    GelloConfig,
    GripperConfig,
    JOINT_NAMES,
    SerialConfig,
    load_config,
    save_config,
)
from .reader import (
    ConnectionState,
    GelloReader,
    GroupSnapshot,
    ReaderSnapshot,
    ReaderStats,
)


GROUP_CHOICES = {
    "仅左臂（ID 1–7）": 0x1,
    "仅右臂（ID 11–17）": 0x2,
    "双臂（ID 1–7、11–17）": 0x3,
}
GROUP_LABEL_BY_MASK = {value: label for label, value in GROUP_CHOICES.items()}


class _ArmPanel(ttk.Frame):
    def __init__(
        self,
        master: tk.Misc,
        *,
        title: str,
        group: int,
        config: ArmConfig,
        capture_zero: Callable[[int], None],
        toggle_calibration: Callable[[int], None],
        apply_calibration: Callable[[int, int, int], None],
    ) -> None:
        super().__init__(master, padding=10)
        self.group = group
        self._capture_zero = capture_zero
        self._toggle_calibration = toggle_calibration
        self._apply_calibration = apply_calibration
        self._raw_vars = [tk.StringVar(value="—") for _ in range(7)]
        self._unwrapped_vars = [tk.StringVar(value="—") for _ in range(7)]
        self._input_vars = [tk.StringVar(value="—") for _ in range(7)]
        self._mapped_vars = [tk.StringVar(value="—") for _ in range(7)]
        self._velocity_vars = [tk.StringVar(value="—") for _ in range(7)]
        self._mapped_velocity_vars = [tk.StringVar(value="—") for _ in range(6)]
        self._sign_vars = [tk.StringVar(value=str(axis.sign)) for axis in config.axes]
        self._scale_vars = [tk.StringVar(value=f"{axis.scale:g}") for axis in config.axes]
        self._offset_vars = [
            tk.StringVar(value=f"{math.degrees(axis.offset_rad):g}")
            for axis in config.axes
        ]
        self._min_raw = tk.StringVar(value=str(config.gripper.min_raw))
        self._max_raw = tk.StringVar(value=str(config.gripper.max_raw))
        self._inverted = tk.BooleanVar(value=config.gripper.inverted)
        self._output_min = tk.StringVar(value=f"{config.gripper.output_min:g}")
        self._output_max = tk.StringVar(value=f"{config.gripper.output_max:g}")
        self._state = tk.StringVar(value="未连接")
        self._calibration = tk.StringVar(value="未标定")
        self._build(title)

    def _build(self, title: str) -> None:
        toolbar = ttk.Frame(self)
        toolbar.pack(fill="x", pady=(0, 10))
        ttk.Label(toolbar, text=title, font=("TkDefaultFont", 12, "bold")).pack(
            side="left"
        )
        ttk.Label(toolbar, textvariable=self._state).pack(side="left", padx=18)
        ttk.Button(
            toolbar,
            text="以当前姿态归零",
            command=lambda: self._capture_zero(self.group),
        ).pack(side="right")

        table = ttk.Frame(self)
        table.pack(fill="x")
        headings = ("轴", "ID", "原始值", "展开值", "输入角度", "方向", "缩放", "偏移", "映射角度", "输入速度 rad/s", "映射速度 rad/s")
        widths = (8, 6, 10, 11, 12, 7, 10, 12, 12, 16, 16)
        for column, (text, width) in enumerate(zip(headings, widths)):
            ttk.Label(table, text=text, anchor="center").grid(
                row=0, column=column, padx=3, pady=4, sticky="ew"
            )
            table.columnconfigure(column, minsize=width * 8)

        servo_offset = self.group * 10
        for row, joint in enumerate(JOINT_NAMES, start=1):
            index = row - 1
            ttk.Label(table, text=f"J{row}  {joint}").grid(
                row=row, column=0, padx=3, pady=3, sticky="w"
            )
            ttk.Label(table, text=str(index + 1 + servo_offset), anchor="center").grid(
                row=row, column=1, sticky="ew"
            )
            ttk.Label(table, textvariable=self._raw_vars[index], anchor="e").grid(
                row=row, column=2, sticky="ew"
            )
            ttk.Label(
                table, textvariable=self._unwrapped_vars[index], anchor="e"
            ).grid(row=row, column=3, sticky="ew")
            ttk.Label(table, textvariable=self._input_vars[index], anchor="e").grid(
                row=row, column=4, sticky="ew"
            )
            ttk.Combobox(
                table,
                textvariable=self._sign_vars[index],
                values=("1", "-1"),
                width=4,
                state="readonly",
            ).grid(row=row, column=5, padx=3)
            ttk.Entry(table, textvariable=self._scale_vars[index], width=9).grid(
                row=row, column=6, padx=3
            )
            ttk.Entry(table, textvariable=self._offset_vars[index], width=10).grid(
                row=row, column=7, padx=3
            )
            ttk.Label(table, textvariable=self._mapped_vars[index], anchor="e").grid(
                row=row, column=8, sticky="ew"
            )

            ttk.Label(table, textvariable=self._velocity_vars[index], anchor="e").grid(
                row=row, column=9, sticky="ew"
            )
            ttk.Label(table, textvariable=self._mapped_velocity_vars[index], anchor="e").grid(
                row=row, column=10, sticky="ew"
            )

        separator = ttk.Separator(self)
        separator.pack(fill="x", pady=14)
        gripper = ttk.LabelFrame(self, text="夹爪", padding=10)
        gripper.pack(fill="x")

        ttk.Label(gripper, text="ID").grid(row=0, column=0, sticky="w")
        ttk.Label(gripper, text=str(7 + servo_offset)).grid(row=0, column=1, padx=(6, 20))
        ttk.Label(gripper, text="原始值").grid(row=0, column=2, sticky="w")
        ttk.Label(gripper, textvariable=self._raw_vars[6], width=8).grid(row=0, column=3)
        ttk.Label(gripper, text="输入角度").grid(row=0, column=4, sticky="w")
        ttk.Label(gripper, textvariable=self._input_vars[6], width=12).grid(row=0, column=5)
        ttk.Label(gripper, text="输出").grid(row=0, column=6, sticky="w")
        ttk.Label(gripper, textvariable=self._mapped_vars[6], width=12).grid(row=0, column=7)

        ttk.Label(gripper, text="最小展开值").grid(row=1, column=0, pady=(10, 0), sticky="w")
        ttk.Entry(gripper, textvariable=self._min_raw, width=8).grid(row=1, column=1, pady=(10, 0))
        ttk.Label(gripper, text="最大展开值").grid(row=1, column=2, pady=(10, 0), sticky="w")
        ttk.Entry(gripper, textvariable=self._max_raw, width=8).grid(row=1, column=3, pady=(10, 0))
        ttk.Checkbutton(gripper, text="反向", variable=self._inverted).grid(
            row=1, column=4, pady=(10, 0), sticky="w"
        )
        ttk.Button(
            gripper,
            text="应用范围",
            command=self._apply_calibration_values,
        ).grid(row=1, column=5, pady=(10, 0), padx=3)
        ttk.Button(
            gripper,
            text="开始/结束标定",
            command=lambda: self._toggle_calibration(self.group),
        ).grid(row=1, column=6, columnspan=2, pady=(10, 0), padx=3)

        ttk.Label(gripper, text="输出最小值").grid(row=2, column=0, pady=(10, 0), sticky="w")
        ttk.Entry(gripper, textvariable=self._output_min, width=8).grid(row=2, column=1, pady=(10, 0))
        ttk.Label(gripper, text="输出最大值").grid(row=2, column=2, pady=(10, 0), sticky="w")
        ttk.Entry(gripper, textvariable=self._output_max, width=8).grid(row=2, column=3, pady=(10, 0))
        ttk.Label(gripper, textvariable=self._calibration).grid(
            row=2, column=4, columnspan=4, pady=(10, 0), sticky="w"
        )

        ttk.Label(gripper, text="舵机轴速度 rad/s").grid(
            row=3, column=0, columnspan=2, pady=(10, 0), sticky="w"
        )
        ttk.Label(gripper, textvariable=self._velocity_vars[6]).grid(
            row=3, column=2, columnspan=2, pady=(10, 0), sticky="w"
        )

    def _apply_calibration_values(self) -> None:
        try:
            minimum = int(self._min_raw.get())
            maximum = int(self._max_raw.get())
            GripperConfig(
                min_raw=minimum,
                max_raw=maximum,
                inverted=self._inverted.get(),
                output_min=float(self._output_min.get()),
                output_max=float(self._output_max.get()),
            ).validate()
            self._apply_calibration(self.group, minimum, maximum)
        except ValueError as error:
            messagebox.showerror("夹爪参数错误", str(error), parent=self)

    def get_config(self) -> ArmConfig:
        config = ArmConfig(axes=self._get_axes(), gripper=self._get_gripper())
        config.validate()
        return config

    def _get_axes(self) -> list[AxisConfig]:
        axes = [
            AxisConfig(
                sign=int(sign.get()),
                scale=float(scale.get()),
                offset_rad=math.radians(float(offset.get())),
            )
            for sign, scale, offset in zip(
                self._sign_vars, self._scale_vars, self._offset_vars
            )
        ]
        for axis in axes:
            axis.validate()
        return axes

    def _get_gripper(self) -> GripperConfig:
        gripper = GripperConfig(
            min_raw=int(self._min_raw.get()),
            max_raw=int(self._max_raw.get()),
            inverted=self._inverted.get(),
            output_min=float(self._output_min.get()),
            output_max=float(self._output_max.get()),
        )
        gripper.validate()
        return gripper

    def set_config(self, config: ArmConfig) -> None:
        for index, axis in enumerate(config.axes):
            self._sign_vars[index].set(str(axis.sign))
            self._scale_vars[index].set(f"{axis.scale:g}")
            self._offset_vars[index].set(f"{math.degrees(axis.offset_rad):g}")
        self._min_raw.set(str(config.gripper.min_raw))
        self._max_raw.set(str(config.gripper.max_raw))
        self._inverted.set(config.gripper.inverted)
        self._output_min.set(f"{config.gripper.output_min:g}")
        self._output_max.set(f"{config.gripper.output_max:g}")

    def update_snapshot(self, snapshot: GroupSnapshot | None) -> None:
        if snapshot is None:
            self._state.set("无数据")
            for values in (
                self._raw_vars,
                self._unwrapped_vars,
                self._input_vars,
                self._mapped_vars,
                self._velocity_vars,
                self._mapped_velocity_vars,
            ):
                for value in values:
                    value.set("—")
            return

        state = "数据正常" if snapshot.data_valid else "本轮数据不完整"
        state += " / 速度完整" if snapshot.velocity_valid else " / 速度缺失或未启用"
        self._state.set(f"{state}    零点：{'是' if snapshot.zeroed else '否'}")
        try:
            axes = self._get_axes()
        except ValueError:
            axes = None
        try:
            gripper_config = self._get_gripper()
        except ValueError:
            gripper_config = None

        for index in range(7):
            velocity = (
                snapshot.arm_velocities_rad_s[index] if index < 6
                else snapshot.gripper_servo_velocity_rad_s
            )
            self._velocity_vars[index].set("—" if velocity is None else f"{velocity:+.4f}")
            if index < 6:
                mapped_velocity = (
                    None if velocity is None or axes is None
                    else axes[index].apply_velocity(velocity)
                )
                self._mapped_velocity_vars[index].set(
                    "—" if mapped_velocity is None else f"{mapped_velocity:+.4f}"
                )
            raw = snapshot.raw_positions[index]
            unwrapped = snapshot.unwrapped_ticks[index]
            self._raw_vars[index].set("—" if raw is None else str(raw))
            self._unwrapped_vars[index].set("—" if unwrapped is None else str(unwrapped))
            radians = (
                snapshot.arm_radians[index]
                if index < 6
                else snapshot.gripper_radians
            )
            self._input_vars[index].set(
                "—" if radians is None else f"{math.degrees(radians):+.2f}°"
            )
            if raw is None:
                mapped = "—"
            elif index < 6 and radians is not None and axes is not None:
                mapped = f"{math.degrees(axes[index].apply(radians)):+.2f}°"
            elif (
                index == 6
                and gripper_config is not None
                and snapshot.gripper_normalized is not None
            ):
                mapped = (
                    f"{gripper_config.apply_fraction(snapshot.gripper_normalized):.4f}"
                )
            else:
                mapped = "—"
            self._mapped_vars[index].set(mapped)

        if snapshot.gripper_calib_active:
            if snapshot.gripper_calib_min_raw is not None:
                self._min_raw.set(str(snapshot.gripper_calib_min_raw))
            if snapshot.gripper_calib_max_raw is not None:
                self._max_raw.set(str(snapshot.gripper_calib_max_raw))
            self._calibration.set("标定进行中")
        elif snapshot.gripper_calib_done:
            self._calibration.set("标定完成")
        else:
            self._calibration.set("未标定")


class GelloTunerApp:
    """Own the Tk widgets and one replaceable GelloReader instance."""

    REFRESH_MS = 50

    def __init__(
        self,
        root: tk.Tk,
        config_path: str | Path,
        config: GelloConfig,
        *,
        reader_factory: Callable[..., GelloReader] = GelloReader,
    ) -> None:
        self.root = root
        self.config_path = Path(config_path)
        self._reader_factory = reader_factory
        self._reader: GelloReader | None = None
        self._closed = False
        self._active_config = deepcopy(config)

        self._port = tk.StringVar(value=config.serial.port)
        self._baud = tk.StringVar(value=str(config.serial.baud))
        self._groups = tk.StringVar(value=GROUP_LABEL_BY_MASK[config.serial.group_mask])
        self._timeout_ms = tk.StringVar(value=f"{config.serial.timeout_ms:g}")
        self._scan_interval_ms = tk.StringVar(
            value=f"{config.serial.scan_interval_ms:g}"
        )
        self._max_delta = tk.StringVar(value=str(config.serial.max_delta_ticks))
        self._auto_reconnect = tk.BooleanVar(value=config.serial.auto_reconnect)
        self._swap_arms = tk.BooleanVar(value=config.swap_arms)
        self._link_loss_ms = tk.StringVar(value=f"{config.serial.link_loss_ms:g}")
        self._reconnect_initial_ms = tk.StringVar(
            value=f"{config.serial.reconnect_initial_ms:g}"
        )
        self._reconnect_max_ms = tk.StringVar(
            value=f"{config.serial.reconnect_max_ms:g}"
        )
        self._read_velocity = tk.BooleanVar(value=config.serial.read_velocity)
        self._velocity_rpm_per_unit = tk.StringVar(value=f"{config.serial.velocity_rpm_per_unit:g}")
        self._connection = tk.StringVar(value="未连接")
        self._stats = tk.StringVar(value="尚未采集")
        self._node_stats = tk.StringVar(value="")
        self._config_status = tk.StringVar(value="映射参数已应用")

        self._build(config)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(self.REFRESH_MS, self._refresh)

    def _build(self, config: GelloConfig) -> None:
        self.root.title("Gello 双臂调参")
        self.root.geometry("1380x800")
        self.root.minsize(1280, 760)

        top = ttk.LabelFrame(self.root, text="Gello USB", padding=10)
        top.pack(fill="x", padx=10, pady=(10, 5))

        ttk.Label(top, text="串口").grid(row=0, column=0, sticky="w")
        self._port_entry = ttk.Combobox(
            top,
            textvariable=self._port,
            values=("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"),
            width=24,
        )
        self._port_entry.grid(row=0, column=1, padx=(5, 14))
        ttk.Label(top, text="波特率").grid(row=0, column=2, sticky="w")
        self._baud_entry = ttk.Combobox(
            top,
            textvariable=self._baud,
            values=("1000000", "921600", "115200"),
            width=11,
        )
        self._baud_entry.grid(row=0, column=3, padx=(5, 14))
        ttk.Label(top, text="扫描").grid(row=0, column=4, sticky="w")
        self._group_entry = ttk.Combobox(
            top,
            textvariable=self._groups,
            values=tuple(GROUP_CHOICES),
            state="readonly",
            width=27,
        )
        self._group_entry.grid(row=0, column=5, padx=(5, 14))
        self._connect_button = ttk.Button(top, text="连接", command=self.connect)
        self._connect_button.grid(row=0, column=6, padx=4)
        self._disconnect_button = ttk.Button(
            top, text="断开", command=self.disconnect, state="disabled"
        )
        self._disconnect_button.grid(row=0, column=7, padx=4)

        ttk.Label(top, text="节点超时").grid(row=1, column=0, pady=(10, 0), sticky="w")
        ttk.Entry(top, textvariable=self._timeout_ms, width=10).grid(
            row=1, column=1, pady=(10, 0), sticky="w"
        )
        ttk.Label(top, text="ms    扫描间隔").grid(row=1, column=2, pady=(10, 0), sticky="e")
        ttk.Entry(top, textvariable=self._scan_interval_ms, width=10).grid(
            row=1, column=3, pady=(10, 0), sticky="w"
        )
        ttk.Label(top, text="ms    跳变阈值").grid(row=1, column=4, pady=(10, 0), sticky="e")
        ttk.Entry(top, textvariable=self._max_delta, width=10).grid(
            row=1, column=5, pady=(10, 0), sticky="w"
        )
        ttk.Label(top, textvariable=self._connection).grid(
            row=1, column=6, columnspan=2, pady=(10, 0), sticky="w"
        )

        ttk.Checkbutton(
            top, text="自动重连", variable=self._auto_reconnect
        ).grid(row=2, column=0, columnspan=2, pady=(10, 0), sticky="w")
        ttk.Label(top, text="总线静默").grid(
            row=2, column=2, pady=(10, 0), sticky="e"
        )
        ttk.Entry(top, textvariable=self._link_loss_ms, width=10).grid(
            row=2, column=3, pady=(10, 0), sticky="w"
        )
        ttk.Label(top, text="ms    重连间隔").grid(
            row=2, column=4, pady=(10, 0), sticky="e"
        )
        reconnect_range = ttk.Frame(top)
        reconnect_range.grid(row=2, column=5, pady=(10, 0), sticky="w")
        ttk.Entry(
            reconnect_range, textvariable=self._reconnect_initial_ms, width=7
        ).pack(side="left")
        ttk.Label(reconnect_range, text=" – ").pack(side="left")
        ttk.Entry(
            reconnect_range, textvariable=self._reconnect_max_ms, width=7
        ).pack(side="left")
        ttk.Label(reconnect_range, text="ms").pack(side="left", padx=(4, 0))
        ttk.Checkbutton(
            top,
            text="交换左右臂（ID 1–7 → 右臂）",
            variable=self._swap_arms,
        ).grid(row=2, column=6, columnspan=2, pady=(10, 0), sticky="w")

        ttk.Checkbutton(top, text="读取主手速度", variable=self._read_velocity).grid(
            row=3, column=0, columnspan=2, pady=(10, 0), sticky="w"
        )
        ttk.Label(top, text="速度单位 RPM/原始值").grid(
            row=3, column=2, pady=(10, 0), sticky="e"
        )
        ttk.Combobox(top, textvariable=self._velocity_rpm_per_unit,
                     values=("0.732", "0.0146"), width=10).grid(
            row=3, column=3, pady=(10, 0), sticky="w"
        )
        ttk.Label(top, text="按舵机相位设置选择；修改后重新连接").grid(
            row=3, column=4, columnspan=4, pady=(10, 0), sticky="w"
        )

        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=10, pady=5)
        self._left_panel = _ArmPanel(
            notebook,
            title="左臂 · ID 1–7",
            group=0,
            config=config.left,
            capture_zero=self.capture_zero,
            toggle_calibration=self.toggle_calibration,
            apply_calibration=self.apply_calibration,
        )
        self._right_panel = _ArmPanel(
            notebook,
            title="右臂 · ID 11–17",
            group=1,
            config=config.right,
            capture_zero=self.capture_zero,
            toggle_calibration=self.toggle_calibration,
            apply_calibration=self.apply_calibration,
        )
        diagnostics = ttk.Frame(notebook, padding=12)
        ttk.Label(diagnostics, textvariable=self._stats, justify="left").pack(
            anchor="w"
        )
        ttk.Separator(diagnostics).pack(fill="x", pady=10)
        ttk.Label(diagnostics, textvariable=self._node_stats, justify="left").pack(
            anchor="w"
        )
        notebook.add(self._left_panel, text="左臂")
        notebook.add(self._right_panel, text="右臂")
        notebook.add(diagnostics, text="串口统计")

        bottom = ttk.Frame(self.root, padding=(10, 5, 10, 10))
        bottom.pack(fill="x")
        ttk.Label(bottom, text=f"配置：{self.config_path}").pack(side="left")
        ttk.Label(bottom, textvariable=self._config_status).pack(
            side="left", padx=18
        )
        ttk.Button(bottom, text="重新加载", command=self.reload_config).pack(
            side="right", padx=4
        )
        ttk.Button(bottom, text="保存参数", command=self.save_config).pack(
            side="right", padx=4
        )
        ttk.Button(bottom, text="应用到 MuJoCo", command=self.apply_config).pack(
            side="right", padx=4
        )

    def _serial_config(self) -> SerialConfig:
        config = SerialConfig(
            port=self._port.get().strip(),
            baud=int(self._baud.get()),
            group_mask=GROUP_CHOICES[self._groups.get()],
            timeout_ms=float(self._timeout_ms.get()),
            scan_interval_ms=float(self._scan_interval_ms.get()),
            max_delta_ticks=int(self._max_delta.get()),
            auto_reconnect=self._auto_reconnect.get(),
            link_loss_ms=float(self._link_loss_ms.get()),
            reconnect_initial_ms=float(self._reconnect_initial_ms.get()),
            reconnect_max_ms=float(self._reconnect_max_ms.get()),
            read_velocity=self._read_velocity.get(),
            velocity_rpm_per_unit=float(self._velocity_rpm_per_unit.get()),
        )
        config.validate()
        return config

    def collect_config(self) -> GelloConfig:
        config = GelloConfig(
            serial=self._serial_config(),
            swap_arms=self._swap_arms.get(),
            left=self._left_panel.get_config(),
            right=self._right_panel.get_config(),
        )
        config.validate()
        return config

    @property
    def active_config(self) -> GelloConfig:
        """Return the last validated configuration explicitly applied by the UI."""
        return self._active_config

    @property
    def is_closed(self) -> bool:
        return self._closed

    @property
    def fatal_error(self) -> BaseException | None:
        reader = self._reader
        return reader.fatal_error if reader is not None else None

    def latest_snapshot(self) -> ReaderSnapshot | None:
        reader = self._reader
        return reader.latest_snapshot() if reader is not None else None

    def pump_events(self) -> bool:
        """Process pending Tk work once; return false after the window closes."""
        if self._closed:
            return False
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            self._closed = True
            return False
        return not self._closed

    def apply_config(self) -> bool:
        try:
            config = self.collect_config()
        except (KeyError, ValueError) as error:
            messagebox.showerror("参数错误", str(error), parent=self.root)
            return False

        self._active_config = deepcopy(config)
        reader = self._reader
        if reader is not None:
            if config.serial.group_mask & 0x1:
                reader.set_gripper_calibration(
                    0, config.left.gripper.min_raw, config.left.gripper.max_raw
                )
            if config.serial.group_mask & 0x2:
                reader.set_gripper_calibration(
                    1, config.right.gripper.min_raw, config.right.gripper.max_raw
                )
        self._config_status.set("映射参数已应用")
        return True

    def connect(self) -> None:
        if self._reader is not None:
            return
        try:
            config = self.collect_config()
            serial_config = config.serial
            reader = self._reader_factory(
                serial_config.port,
                baudrate=serial_config.baud,
                group_mask=serial_config.group_mask,
                timeout_s=serial_config.timeout_ms / 1000.0,
                scan_interval_s=serial_config.scan_interval_ms / 1000.0,
                max_delta_ticks=serial_config.max_delta_ticks,
                auto_reconnect=serial_config.auto_reconnect,
                link_loss_s=serial_config.link_loss_ms / 1000.0,
                reconnect_initial_s=serial_config.reconnect_initial_ms / 1000.0,
                reconnect_max_s=serial_config.reconnect_max_ms / 1000.0,
                read_velocity=serial_config.read_velocity,
                velocity_rpm_per_unit=serial_config.velocity_rpm_per_unit,
            )
            if serial_config.group_mask & 0x1:
                reader.set_gripper_calibration(
                    0, config.left.gripper.min_raw, config.left.gripper.max_raw
                )
            if serial_config.group_mask & 0x2:
                reader.set_gripper_calibration(
                    1, config.right.gripper.min_raw, config.right.gripper.max_raw
                )
            reader.start()
            self._active_config = deepcopy(config)
            self._reader = reader
        except (OSError, ValueError) as error:
            messagebox.showerror("连接失败", str(error), parent=self.root)
            return

        self._connection.set(f"连接中：{serial_config.port}")
        self._connect_button.configure(state="disabled")
        self._disconnect_button.configure(state="normal")

    def disconnect(self) -> None:
        reader = self._reader
        self._reader = None
        if reader is not None:
            reader.stop()
        if not self._closed:
            self._connection.set("未连接")
            self._connect_button.configure(state="normal")
            self._disconnect_button.configure(state="disabled")

    def capture_zero(self, group: int) -> None:
        if self._reader is None:
            messagebox.showinfo("尚未连接", "请先连接 Gello 串口。", parent=self.root)
            return
        self._reader.capture_zero(group)

    def toggle_calibration(self, group: int) -> None:
        if self._reader is None:
            messagebox.showinfo("尚未连接", "请先连接 Gello 串口。", parent=self.root)
            return
        self._reader.toggle_gripper_calibration(group)

    def apply_calibration(self, group: int, minimum: int, maximum: int) -> None:
        if self._reader is not None:
            self._reader.set_gripper_calibration(group, minimum, maximum)

    def save_config(self) -> None:
        if not self.apply_config():
            return
        try:
            save_config(self._active_config, self.config_path)
        except (OSError, ValueError) as error:
            messagebox.showerror("保存失败", str(error), parent=self.root)
            return
        self._connection.set(f"参数已保存：{self.config_path.name}")

    def reload_config(self) -> None:
        try:
            config = load_config(self.config_path)
        except (OSError, ValueError) as error:
            messagebox.showerror("加载失败", str(error), parent=self.root)
            return
        self._read_velocity.set(config.serial.read_velocity)
        self._velocity_rpm_per_unit.set(f"{config.serial.velocity_rpm_per_unit:g}")
        self._port.set(config.serial.port)
        self._baud.set(str(config.serial.baud))
        self._groups.set(GROUP_LABEL_BY_MASK[config.serial.group_mask])
        self._swap_arms.set(config.swap_arms)
        self._timeout_ms.set(f"{config.serial.timeout_ms:g}")
        self._scan_interval_ms.set(f"{config.serial.scan_interval_ms:g}")
        self._max_delta.set(str(config.serial.max_delta_ticks))
        self._auto_reconnect.set(config.serial.auto_reconnect)
        self._link_loss_ms.set(f"{config.serial.link_loss_ms:g}")
        self._reconnect_initial_ms.set(
            f"{config.serial.reconnect_initial_ms:g}"
        )
        self._reconnect_max_ms.set(f"{config.serial.reconnect_max_ms:g}")
        self._left_panel.set_config(config.left)
        self._right_panel.set_config(config.right)
        self._active_config = deepcopy(config)
        self._config_status.set("映射参数已重新加载")
        self._connection.set(f"已加载：{self.config_path.name}")

    def _format_stats(self, stats: ReaderStats) -> None:
        self._stats.set(
            f"扫描频率：{stats.scan_hz:.1f} Hz\n"
            f"最近扫描：{stats.last_scan_s * 1000.0:.2f} ms\n"
            f"扫描耗时 p95/p99：{stats.p95_scan_s * 1000.0:.2f} / "
            f"{stats.p99_scan_s * 1000.0:.2f} ms\n"
            f"扫描：{stats.scans}    完整：{stats.complete_scans}    "
            f"不完整：{stats.incomplete_scans}\n"
            f"超时：{stats.servo_timeouts}    checksum：{stats.checksum_errors}    "
            f"帧错误：{stats.frame_errors}    "
            f"舵机状态告警（位置仍采用）：{stats.status_errors}\n"
            f"意外 ID：{stats.unexpected_ids}    跳变复读：{stats.jump_retries}    "
            f"跳变拒绝：{stats.jump_rejects}\n"
            f"重连尝试：{stats.reconnect_attempts}    "
            f"重连成功：{stats.successful_reconnects}"
        )
        self._node_stats.set(
            "各节点超时：\n"
            + "    ".join(f"ID {servo_id}: {count}" for servo_id, count in stats.node_timeouts)
        )

    def _refresh(self) -> None:
        if self._closed:
            return
        reader = self._reader
        if reader is not None:
            snapshot = reader.latest_snapshot()
            self._left_panel.update_snapshot(
                snapshot.group(0) if snapshot is not None else None
            )
            self._right_panel.update_snapshot(
                snapshot.group(1) if snapshot is not None else None
            )
            self._format_stats(reader.stats)
            connection = reader.connection_status
            if connection.state == ConnectionState.CONNECTING:
                self._connection.set(
                    f"连接中（尝试 {connection.reconnect_attempt}）：{reader.port}"
                )
            elif connection.state == ConnectionState.CONNECTED:
                self._connection.set(f"已连接：{reader.port}")
            elif connection.state == ConnectionState.RECONNECT_WAIT:
                self._connection.set(
                    f"等待第 {connection.reconnect_attempt} 次重连 "
                    f"({connection.retry_in_s:.1f}s)：{connection.last_error}"
                )
            elif connection.state == ConnectionState.FAILED:
                self._connection.set(f"串口线程已停止：{connection.last_error}")
                self._connect_button.configure(state="disabled")
                self._disconnect_button.configure(state="normal")
        self.root.after(self.REFRESH_MS, self._refresh)

    def close(self) -> None:
        was_closed = self._closed
        self._closed = True
        reader = self._reader
        self._reader = None
        if reader is not None:
            reader.stop()
        if not was_closed:
            self.root.destroy()
