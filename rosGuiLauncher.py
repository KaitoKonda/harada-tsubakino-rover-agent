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

baseDir = Path(__file__).resolve().parent
configFile = baseDir / "launcherConfig.yaml"


def defaultGroup():
    """Use the short Raspberry Pi hostname as the default ROS namespace."""
    return socket.gethostname().split(".", 1)[0]


def defaultRosHostname():
    """Advertise this rover through its Avahi .local name."""
    return f"{defaultGroup()}.local"


@dataclass
class LaunchConfig:
    rosMasterUri: str = "http://CONTROL_PC_IP:11311"
    rosHostname: str = field(default_factory=defaultRosHostname)
    package: str = "harada-tsubakino"
    launchFile: str = "harada-tsubakino.launch"
    args: dict = field(default_factory=lambda: {"group": defaultGroup()})
    extraScript: str = ""

    @classmethod
    def fromDict(cls, data):
        if data is None:
            data = {}
        elif not isinstance(data, dict):
            raise ValueError("The top-level YAML value must be a mapping.")
        argsData = data.get("args")
        if argsData is None:
            args = {}
        elif isinstance(argsData, dict):
            args = dict(argsData)
        else:
            raise ValueError("The args setting must be a mapping or null.")
        # Always refresh the per-rover namespace from the local hostname.
        # This also migrates old copied configs such as `group: pi`.
        args["group"] = defaultGroup()
        launchFile = data.get("launch_file", cls.launchFile)
        if launchFile == "haradaTsubakino.launch":
            launchFile = cls.launchFile
        return cls(
            rosMasterUri=data.get("ros_master_uri", cls.rosMasterUri),
            # Ignore legacy ros_ip values: the rover is advertised by Avahi.
            rosHostname=defaultRosHostname(),
            package=data.get("package", cls.package),
            launchFile=launchFile,
            args=args,
            extraScript=data.get("extra_script", cls.extraScript),
        )

    def toDict(self):
        return {
            "ros_master_uri": self.rosMasterUri,
            "ros_hostname": "auto",
            "package": self.package,
            "launch_file": self.launchFile,
            "args": self.args,
            "extra_script": self.extraScript,
        }


class RosProcess:
    def __init__(self):
        self.proc = None
        self.extraProc = None

    def isRunning(self):
        return any(proc and proc.poll() is None for proc in (self.proc, self.extraProc))

    def start(self, config: LaunchConfig):
        self.stop()

        env = os.environ.copy()
        env["ROS_MASTER_URI"] = config.rosMasterUri
        env.pop("ROS_IP", None)
        env["ROS_HOSTNAME"] = config.rosHostname

        rosCommand = [
            "roslaunch",
            config.package,
            config.launchFile,
            *[f"{key}:={value}" for key, value in config.args.items()],
        ]

        self.proc = subprocess.Popen(
            rosCommand,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
            text=True,
            bufsize=1,
        )

        if config.extraScript:
            self.extraProc = subprocess.Popen(
                [sys.executable, config.extraScript],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=env,
                text=True,
                bufsize=1,
            )

    def stop(self):
        self.proc = self._stopProcess(self.proc, signal.SIGINT)
        self.extraProc = self._stopProcess(self.extraProc)

    @staticmethod
    def _stopProcess(process, stopSignal=None):
        if not process or process.poll() is not None:
            return None

        try:
            if stopSignal is not None:
                process.send_signal(stopSignal)
            else:
                process.terminate()
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)

        return None


class LogThread(QThread):
    logSignal = pyqtSignal(str)

    def __init__(self, process):
        super().__init__()
        self.process = process

    def run(self):
        if not self.process or not self.process.stdout:
            return

        for line in self.process.stdout:
            self.logSignal.emit(line.rstrip())


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS Launcher GUI")

        self.ros = RosProcess()
        self.logThreads = []

        self.masterUri = QLineEdit()
        self.rosHostname = QLineEdit()
        self.rosHostname.setReadOnly(True)
        self.package = QLineEdit()
        self.launchFile = QLineEdit()
        self.args = QLineEdit()
        self.extraScript = QLineEdit()

        self.launchButton = QPushButton("Launch")
        self.stopButton = QPushButton("Stop")
        self.saveButton = QPushButton("Save Config")
        self.loadButton = QPushButton("Load Config")
        self.fileButton = QPushButton("Browse Script")

        self.log = QTextEdit()
        self.log.setReadOnly(True)

        self._buildLayout()
        self._connectSignals()
        self.loadInitialConfig()
        self.updateButtonState()

    def _buildLayout(self):
        layout = QVBoxLayout()

        layout.addWidget(QLabel("ROS_MASTER_URI"))
        layout.addWidget(self.masterUri)

        layout.addWidget(QLabel("ROS_HOSTNAME (auto)"))
        layout.addWidget(self.rosHostname)

        layout.addWidget(QLabel("Package"))
        layout.addWidget(self.package)

        layout.addWidget(QLabel("Launch File"))
        layout.addWidget(self.launchFile)

        layout.addWidget(QLabel("Args (key:=value space separated)"))
        layout.addWidget(self.args)

        layout.addWidget(QLabel("Extra Python Script"))
        extraScriptRow = QHBoxLayout()
        extraScriptRow.addWidget(self.extraScript)
        extraScriptRow.addWidget(self.fileButton)
        layout.addLayout(extraScriptRow)

        layout.addWidget(self.launchButton)
        layout.addWidget(self.stopButton)
        layout.addWidget(self.saveButton)
        layout.addWidget(self.loadButton)

        layout.addWidget(QLabel("Log"))
        layout.addWidget(self.log)

        self.setLayout(layout)

    def _connectSignals(self):
        self.launchButton.clicked.connect(self.launch)
        self.stopButton.clicked.connect(self.stop)
        self.saveButton.clicked.connect(self.saveConfig)
        self.loadButton.clicked.connect(self.loadConfig)
        self.fileButton.clicked.connect(self.browseFile)

    def parseArgs(self):
        argDict = {}
        for part in self.args.text().split():
            if ":=" in part:
                key, value = part.split(":=", 1)
                argDict[key] = value
        return argDict

    def formatArgs(self, args):
        return " ".join(f"{key}:={value}" for key, value in args.items())

    def getConfig(self):
        return LaunchConfig(
            rosMasterUri=self.masterUri.text().strip(),
            rosHostname=self.rosHostname.text().strip(),
            package=self.package.text().strip(),
            launchFile=self.launchFile.text().strip(),
            args=self.parseArgs(),
            extraScript=self.extraScript.text().strip(),
        )

    def applyConfig(self, config: LaunchConfig):
        self.masterUri.setText(config.rosMasterUri)
        self.rosHostname.setText(config.rosHostname)
        self.package.setText(config.package)
        self.launchFile.setText(config.launchFile)
        self.args.setText(self.formatArgs(config.args))
        self.extraScript.setText(config.extraScript)

    def loadInitialConfig(self):
        if os.path.exists(configFile):
            try:
                with open(configFile, encoding="utf-8") as fileObj:
                    loaded = yaml.safe_load(fileObj)
                config = LaunchConfig.fromDict(loaded)
            except (OSError, UnicodeError, ValueError, yaml.YAMLError) as exc:
                self.applyConfig(LaunchConfig())
                message = (
                    f"Could not read {configFile}:\n{exc}\n\n"
                    "Run 'python3 update.py -g' from the repository to repair "
                    "the configuration."
                )
                self.appendLog(f"ERROR: {message}")
                QMessageBox.critical(self, "Invalid Launcher Configuration", message)
                return
            self.applyConfig(config)
            self.appendLog(f"Loaded config from {configFile}.")
            return

        self.applyConfig(LaunchConfig())

    def validateConfig(self, config: LaunchConfig):
        if not config.rosMasterUri or "CONTROL_PC_IP" in config.rosMasterUri:
            return "Replace CONTROL_PC_IP with the control PC's IP address."
        if not config.rosHostname or not config.rosHostname.endswith(".local"):
            return "The automatically generated ROS_HOSTNAME is invalid."
        if not config.package:
            return "Package is required."
        if not config.launchFile:
            return "Launch file is required."
        if config.extraScript and not os.path.isfile(config.extraScript):
            return "Extra Python script path does not exist."
        return ""

    def launch(self):
        config = self.getConfig()
        errorMessage = self.validateConfig(config)
        if errorMessage:
            QMessageBox.warning(self, "Invalid Configuration", errorMessage)
            return

        try:
            self.ros.start(config)
        except Exception as exc:
            QMessageBox.critical(self, "Launch Failed", str(exc))
            self.appendLog(f"ERROR: {exc}")
            self.updateButtonState()
            return

        self.log.clear()
        self.appendLog("Started ROS launch process.")
        self.appendLog(f"ROS_MASTER_URI={config.rosMasterUri}")
        self.appendLog(f"ROS_HOSTNAME={config.rosHostname}")
        self._startLogThread(self.ros.proc)

        if self.ros.extraProc:
            self.appendLog("Started extra Python script.")
            self._startLogThread(self.ros.extraProc)

        self.updateButtonState()

    def _startLogThread(self, process):
        if not process or not process.stdout:
            return

        thread = LogThread(process)
        thread.logSignal.connect(self.appendLog)
        thread.finished.connect(self.cleanupThreads)
        thread.start()
        self.logThreads.append(thread)

    def stop(self):
        self.ros.stop()
        self.cleanupThreads()
        self.appendLog("Stopped running processes.")
        self.updateButtonState()

    def cleanupThreads(self):
        aliveThreads = []
        for thread in self.logThreads:
            if thread.isRunning():
                aliveThreads.append(thread)
            else:
                thread.wait(100)
        self.logThreads = aliveThreads

    def appendLog(self, text):
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
        self.updateButtonState()

    def updateButtonState(self):
        running = self.ros.isRunning()
        self.launchButton.setEnabled(not running)
        self.stopButton.setEnabled(running)

    def saveConfig(self):
        config = self.getConfig()
        reply = QMessageBox.question(
            self,
            "Confirm Save",
            f"Overwrite {configFile} with the current settings?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            self.appendLog("Save cancelled.")
            return

        with open(configFile, "w", encoding="utf-8") as fileObj:
            yaml.safe_dump(config.toDict(), fileObj, sort_keys=False)
        self.appendLog(f"Saved config to {configFile}.")

    def loadConfig(self):
        if not os.path.exists(configFile):
            QMessageBox.information(self, "Load Config", f"{configFile} was not found.")
            return

        try:
            with open(configFile, encoding="utf-8") as fileObj:
                config = LaunchConfig.fromDict(yaml.safe_load(fileObj))
        except (OSError, UnicodeError, ValueError, yaml.YAMLError) as exc:
            message = f"Could not read {configFile}:\n{exc}"
            self.appendLog(f"ERROR: {message}")
            QMessageBox.critical(self, "Invalid Launcher Configuration", message)
            return

        self.applyConfig(config)
        self.appendLog(f"Loaded config from {configFile}.")

    def browseFile(self):
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Select Python Script",
            "",
            "Python Files (*.py);;All Files (*)",
        )
        if filename:
            self.extraScript.setText(filename)

    def closeEvent(self, event):
        self.stop()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(600, 800)
    window.show()
    sys.exit(app.exec_())
