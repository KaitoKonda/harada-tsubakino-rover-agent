import os
import signal
import socket
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

import yaml
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

BASE_DIR = Path(__file__).resolve().parent
CONFIG_FILE = BASE_DIR / "launcherConfig.yaml"


def default_group():
    """Use the short Raspberry Pi hostname as the default ROS namespace."""
    return socket.gethostname().split(".", 1)[0]


def default_ros_hostname():
    """Advertise this rover through its Avahi .local name."""
    return f"{default_group()}.local"


@dataclass
class LaunchConfig:
    ros_master_uri: str = "http://CONTROL_PC_IP:11311"
    ros_hostname: str = field(default_factory=default_ros_hostname)
    package: str = "harada-tsubakino"
    launch_file: str = "harada-tsubakino.launch"
    args: dict = field(default_factory=lambda: {"group": default_group()})
    extra_script: str = ""

    @classmethod
    def from_dict(cls, data):
        data = data or {}
        args = dict(data.get("args", {}))
        # Always refresh the per-rover namespace from the local hostname.
        # This also migrates old copied configs such as `group: pi`.
        args["group"] = default_group()
        return cls(
            ros_master_uri=data.get("ros_master_uri", cls.ros_master_uri),
            # Ignore legacy ros_ip values: the rover is advertised by Avahi.
            ros_hostname=default_ros_hostname(),
            package=data.get("package", cls.package),
            launch_file=data.get("launch_file", cls.launch_file),
            args=args,
            extra_script=data.get("extra_script", cls.extra_script),
        )

    def to_dict(self):
        return {
            "ros_master_uri": self.ros_master_uri,
            "ros_hostname": "auto",
            "package": self.package,
            "launch_file": self.launch_file,
            "args": self.args,
            "extra_script": self.extra_script,
        }


class ROSProcess:
    def __init__(self):
        self.proc = None
        self.extra_proc = None

    def is_running(self):
        return any(proc and proc.poll() is None for proc in (self.proc, self.extra_proc))

    def start(self, config: LaunchConfig):
        self.stop()

        env = os.environ.copy()
        env["ROS_MASTER_URI"] = config.ros_master_uri
        env.pop("ROS_IP", None)
        env["ROS_HOSTNAME"] = config.ros_hostname

        ros_command = [
            "roslaunch",
            config.package,
            config.launch_file,
            *[f"{key}:={value}" for key, value in config.args.items()],
        ]

        self.proc = subprocess.Popen(
            ros_command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
            text=True,
            bufsize=1,
        )

        if config.extra_script:
            self.extra_proc = subprocess.Popen(
                [sys.executable, config.extra_script],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                text=True,
                bufsize=1,
            )

    def stop(self):
        self.proc = self._stop_process(self.proc, signal.SIGINT)
        self.extra_proc = self._stop_process(self.extra_proc)

    @staticmethod
    def _stop_process(process, stop_signal=None):
        if not process or process.poll() is not None:
            return None

        try:
            if stop_signal is not None:
                process.send_signal(stop_signal)
            else:
                process.terminate()
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)

        return None


class LogThread(QThread):
    log_signal = pyqtSignal(str)

    def __init__(self, process):
        super().__init__()
        self.process = process

    def run(self):
        if not self.process or not self.process.stdout:
            return

        for line in self.process.stdout:
            self.log_signal.emit(line.rstrip())


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS Launcher GUI")

        self.ros = ROSProcess()
        self.log_threads = []

        self.master_uri = QLineEdit()
        self.ros_hostname = QLineEdit()
        self.ros_hostname.setReadOnly(True)
        self.package = QLineEdit()
        self.launch_file = QLineEdit()
        self.args = QLineEdit()
        self.extra_script = QLineEdit()

        self.launch_btn = QPushButton("Launch")
        self.stop_btn = QPushButton("Stop")
        self.save_btn = QPushButton("Save Config")
        self.load_btn = QPushButton("Load Config")
        self.file_btn = QPushButton("Browse Script")

        self.log = QTextEdit()
        self.log.setReadOnly(True)

        self._build_layout()
        self._connect_signals()
        self.load_initial_config()
        self.update_button_state()

    def _build_layout(self):
        layout = QVBoxLayout()

        layout.addWidget(QLabel("ROS_MASTER_URI"))
        layout.addWidget(self.master_uri)

        layout.addWidget(QLabel("ROS_HOSTNAME (auto)"))
        layout.addWidget(self.ros_hostname)

        layout.addWidget(QLabel("Package"))
        layout.addWidget(self.package)

        layout.addWidget(QLabel("Launch File"))
        layout.addWidget(self.launch_file)

        layout.addWidget(QLabel("Args (key:=value space separated)"))
        layout.addWidget(self.args)

        layout.addWidget(QLabel("Extra Python Script"))
        extra_script_row = QHBoxLayout()
        extra_script_row.addWidget(self.extra_script)
        extra_script_row.addWidget(self.file_btn)
        layout.addLayout(extra_script_row)

        layout.addWidget(self.launch_btn)
        layout.addWidget(self.stop_btn)
        layout.addWidget(self.save_btn)
        layout.addWidget(self.load_btn)

        layout.addWidget(QLabel("Log"))
        layout.addWidget(self.log)

        self.setLayout(layout)

    def _connect_signals(self):
        self.launch_btn.clicked.connect(self.launch)
        self.stop_btn.clicked.connect(self.stop)
        self.save_btn.clicked.connect(self.save_config)
        self.load_btn.clicked.connect(self.load_config)
        self.file_btn.clicked.connect(self.browse_file)

    def parse_args(self):
        arg_dict = {}
        for part in self.args.text().split():
            if ":=" in part:
                key, value = part.split(":=", 1)
                arg_dict[key] = value
        return arg_dict

    def format_args(self, args):
        return " ".join(f"{key}:={value}" for key, value in args.items())

    def get_config(self):
        return LaunchConfig(
            ros_master_uri=self.master_uri.text().strip(),
            ros_hostname=self.ros_hostname.text().strip(),
            package=self.package.text().strip(),
            launch_file=self.launch_file.text().strip(),
            args=self.parse_args(),
            extra_script=self.extra_script.text().strip(),
        )

    def apply_config(self, config: LaunchConfig):
        self.master_uri.setText(config.ros_master_uri)
        self.ros_hostname.setText(config.ros_hostname)
        self.package.setText(config.package)
        self.launch_file.setText(config.launch_file)
        self.args.setText(self.format_args(config.args))
        self.extra_script.setText(config.extra_script)

    def load_initial_config(self):
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, encoding="utf-8") as file_obj:
                    loaded = yaml.safe_load(file_obj)
                if loaded is not None and not isinstance(loaded, dict):
                    raise ValueError("The top-level YAML value must be a mapping.")
                config = LaunchConfig.from_dict(loaded)
            except (OSError, UnicodeError, ValueError, yaml.YAMLError) as exc:
                self.apply_config(LaunchConfig())
                message = (
                    f"Could not read {CONFIG_FILE}:\n{exc}\n\n"
                    "Run 'python3 update.py -g' from the repository to repair "
                    "the configuration."
                )
                self.append_log(f"ERROR: {message}")
                QMessageBox.critical(self, "Invalid Launcher Configuration", message)
                return
            self.apply_config(config)
            self.append_log(f"Loaded config from {CONFIG_FILE}.")
            return

        self.apply_config(LaunchConfig())

    def validate_config(self, config: LaunchConfig):
        if not config.ros_master_uri or "CONTROL_PC_IP" in config.ros_master_uri:
            return "Replace CONTROL_PC_IP with the control PC's IP address."
        if not config.ros_hostname or not config.ros_hostname.endswith(".local"):
            return "The automatically generated ROS_HOSTNAME is invalid."
        if not config.package:
            return "Package is required."
        if not config.launch_file:
            return "Launch file is required."
        if config.extra_script and not os.path.isfile(config.extra_script):
            return "Extra Python script path does not exist."
        return ""

    def launch(self):
        config = self.get_config()
        error_message = self.validate_config(config)
        if error_message:
            QMessageBox.warning(self, "Invalid Configuration", error_message)
            return

        try:
            self.ros.start(config)
        except Exception as exc:
            QMessageBox.critical(self, "Launch Failed", str(exc))
            self.append_log(f"ERROR: {exc}")
            self.update_button_state()
            return

        self.log.clear()
        self.append_log("Started ROS launch process.")
        self.append_log(f"ROS_MASTER_URI={config.ros_master_uri}")
        self.append_log(f"ROS_HOSTNAME={config.ros_hostname}")
        self._start_log_thread(self.ros.proc)

        if self.ros.extra_proc:
            self.append_log("Started extra Python script.")
            self._start_log_thread(self.ros.extra_proc)

        self.update_button_state()

    def _start_log_thread(self, process):
        if not process or not process.stdout:
            return

        thread = LogThread(process)
        thread.log_signal.connect(self.append_log)
        thread.finished.connect(self.cleanup_threads)
        thread.start()
        self.log_threads.append(thread)

    def stop(self):
        self.ros.stop()
        self.cleanup_threads()
        self.append_log("Stopped running processes.")
        self.update_button_state()

    def cleanup_threads(self):
        alive_threads = []
        for thread in self.log_threads:
            if thread.isRunning():
                alive_threads.append(thread)
            else:
                thread.wait(100)
        self.log_threads = alive_threads

    def append_log(self, text):
        if not text:
            return

        if "ERROR" in text:
            color = QColor("red")
        elif "WARN" in text:
            color = QColor("darkGoldenrod")
        else:
            color = QColor("black")

        self.log.setTextColor(color)
        self.log.append(text)
        self.update_button_state()

    def update_button_state(self):
        running = self.ros.is_running()
        self.launch_btn.setEnabled(not running)
        self.stop_btn.setEnabled(running)

    def save_config(self):
        config = self.get_config()
        reply = QMessageBox.question(
            self,
            "Confirm Save",
            f"Overwrite {CONFIG_FILE} with the current settings?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            self.append_log("Save cancelled.")
            return

        with open(CONFIG_FILE, "w", encoding="utf-8") as file_obj:
            yaml.safe_dump(config.to_dict(), file_obj, sort_keys=False)
        self.append_log(f"Saved config to {CONFIG_FILE}.")

    def load_config(self):
        if not os.path.exists(CONFIG_FILE):
            QMessageBox.information(self, "Load Config", f"{CONFIG_FILE} was not found.")
            return

        with open(CONFIG_FILE, encoding="utf-8") as file_obj:
            config = LaunchConfig.from_dict(yaml.safe_load(file_obj))

        self.apply_config(config)
        self.append_log(f"Loaded config from {CONFIG_FILE}.")

    def browse_file(self):
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Select Python Script",
            "",
            "Python Files (*.py);;All Files (*)",
        )
        if filename:
            self.extra_script.setText(filename)

    def closeEvent(self, event):
        self.stop()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(600, 800)
    window.show()
    sys.exit(app.exec_())
