#!/usr/bin/env python3
"""Korean Qt Quick parameter editor; serial I/O belongs to one worker thread."""

from datetime import datetime
from pathlib import Path
import sys

from PySide6.QtCore import (QAbstractListModel, QModelIndex, QObject, Property,
                           QMetaObject, Qt, QThread, QTimer, QUrl, Signal, Slot)
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine
from PySide6.QtQuickControls2 import QQuickStyle
from serial.tools import list_ports

from keya.controller import KeyaController
from keya.parameters import PARAMETERS, encode_value, format_value, prepare_writes
from keya.parameter_help import HELP, CATEGORIES, physical_text
from keya.protocol import make_ky, read_ky


class ParameterModel(QAbstractListModel):
    changed = Signal()
    ROLES = {Qt.UserRole + i + 1: name.encode() for i, name in enumerate((
        "paramId", "code", "parameterName", "sectionName", "tooltip", "paramValue",
        "ramValue", "romValue", "editorValue", "dirty", "unsaved", "editable",
        "inputError", "selected", "ramMeaning", "romMeaning"))}

    def __init__(self, parent=None):
        super().__init__(parent)
        self.snapshot = {}
        self.drafts = {}
        self.baselines = {}
        self.advanced = False
        self.group = CATEGORIES[0]
        self.selected = 13

    def roleNames(self):
        return self.ROLES

    def indices(self):
        return [i for i in HELP if HELP[i][1] == self.group]

    def rowCount(self, parent=QModelIndex()):
        return 0 if parent.isValid() else len(self.indices())

    def row(self, i):
        spec, info = PARAMETERS[i], HELP[i]
        raw = [self.snapshot.get((i, bank)) for bank in range(3)]
        text = ["—" if v is None else format_value(i, v) for v in raw]
        draft = self.drafts.get(i, "" if raw[1] is None else text[1])
        error = ""
        try:
            target = encode_value(i, draft) if draft else None
        except ValueError:
            target = None
            error = "소수 셋째 자리까지, 0–65.535를 입력하세요." if spec.kind == "fp3" else "0–65535의 정수를 입력하세요."
        if i in self.drafts and not draft:
            error = "값을 입력하세요."
        dirty = i in self.drafts and (bool(error) or target != raw[1])
        return dict(paramId=i, code=f"{i:04d}", parameterName=info[0], sectionName=info[2],
                    tooltip=info[3] + "\n" + info[4], paramValue=text[0], ramValue=text[1],
                    romValue=text[2], editorValue=draft, dirty=dirty,
                    unsaved=raw[1] is not None and raw[2] is not None and raw[1] != raw[2],
                    editable=not spec.read_only and (spec.normal_edit or self.advanced) and raw[1] is not None,
                    inputError=error, selected=i == self.selected,
                    ramMeaning=physical_text(i, raw[1]), romMeaning=physical_text(i, raw[2]))

    def data(self, index, role=Qt.DisplayRole):
        indices = self.indices()
        if not index.isValid() or index.row() >= len(indices) or role not in self.ROLES:
            return None
        return self.row(indices[index.row()])[self.ROLES[role].decode()]

    def notify_rows(self, roles=None):
        if self.rowCount():
            self.dataChanged.emit(self.index(0), self.index(self.rowCount() - 1), roles or list(self.ROLES))
        self.changed.emit()

    def set_snapshot(self, snapshot, clear_edits=False):
        # Preserve typed text and its original RAM baseline during background refresh.
        previous = {i: self.row(i) for i in self.indices()}
        self.snapshot = dict(snapshot)
        if clear_edits:
            self.drafts.clear()
            self.baselines.clear()
        for n, i in enumerate(self.indices()):
            current = self.row(i)
            roles = [r for r, name in self.ROLES.items() if previous[i][name.decode()] != current[name.decode()]]
            if roles:
                self.dataChanged.emit(self.index(n), self.index(n), roles)
        self.changed.emit()

    def set_group(self, group):
        self.beginResetModel()
        self.group = group
        if self.selected not in self.indices():
            self.selected = self.indices()[0]
        self.endResetModel()
        self.changed.emit()

    def select(self, i):
        self.selected = i
        self.notify_rows([r for r, name in self.ROLES.items() if name == b"selected"])

    def edit(self, i, text):
        if not self.row(i)["editable"]:
            return
        if i not in self.drafts:
            self.baselines[i] = self.snapshot.get((i, 1))
        self.drafts[i] = text
        self.selected = i
        self.notify_rows()

    def pending(self):
        return {i: text for i, text in self.drafts.items() if self.row(i)["dirty"]}

    def displayed_for_write(self):
        result = dict(self.snapshot)
        result.update({(i, 1): raw for i, raw in self.baselines.items()})
        return result

    def reset_edits(self):
        self.drafts.clear()
        self.baselines.clear()
        self.notify_rows()

    def import_values(self, values):
        unknown = set(values) - set(PARAMETERS)
        if unknown:
            raise ValueError("지원하지 않는 파라미터: " + ", ".join(f"{i:04d}" for i in sorted(unknown)))
        changes = {i: raw for i, raw in values.items()
                   if not PARAMETERS[i].read_only and self.snapshot.get((i, 1)) != raw}
        for i in changes:
            if self.snapshot.get((i, 1)) is None:
                raise ValueError(f"{i:04d}: 장치에서 RAM 값을 받지 못했습니다. 새로고침 후 다시 불러오세요.")
            if not PARAMETERS[i].normal_edit and not self.advanced:
                raise ValueError("파일에 고급 설정의 변경값이 있습니다. 고급 편집을 허용한 뒤 불러오세요.")
        for i, raw in changes.items():
            self.baselines[i] = self.snapshot[(i, 1)]
            self.drafts[i] = format_value(i, raw)
        self.notify_rows()
        return len(changes)

    @Property(int, notify=changed)
    def dirtyCount(self): return len(self.pending())

    @Property(int, notify=changed)
    def invalidCount(self): return sum(bool(self.row(i)["inputError"]) for i in self.drafts)

    @Property(int, notify=changed)
    def unsavedCount(self): return sum(self.row(i)["unsaved"] for i in HELP)

    @Property("QVariantList", notify=changed)
    def burnChanges(self):
        return [self.row(i) for i in HELP if self.row(i)["unsaved"]]

    @Property("QVariantMap", notify=changed)
    def detail(self):
        i = self.selected
        row, info, spec = self.row(i), HELP[i], PARAMETERS[i]
        raw = self.snapshot.get((i, 1))
        editor_meaning = ""
        if not row["inputError"] and row["editorValue"]:
            editor_meaning = physical_text(i, encode_value(i, row["editorValue"]))
        return dict(row, original=spec.name, description=info[3], reference=info[4], note=info[5],
                    resolution="0.001" if spec.kind == "fp3" else "정수 1",
                    editorMeaning=editor_meaning,
                    access="읽기 전용" if spec.read_only else ("일반 편집" if spec.normal_edit else "고급 편집"),
                    conflict=i in self.drafts and self.baselines.get(i) != raw and row["dirty"])


class SerialWorker(QObject):
    finished = Signal(str, object, str)
    failed = Signal(str, str)
    progress = Signal(str)

    def __init__(self):
        super().__init__()
        self.controller = None
        self.timer = QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.poll)

    @Slot(str)
    def connect_port(self, port):
        try:
            controller = KeyaController(port)
            self.progress.emit("포트 열림 · 설정 응답 기다리는 중")
            try:
                snapshot = controller.connect()
                if not any(bank == 1 and i in PARAMETERS for i, bank in snapshot):
                    raise ValueError("설정 데이터에서 RAM 값을 찾지 못했습니다.")
            except BaseException:
                try:
                    controller.close()
                except OSError:
                    pass
                raise
            self.controller = controller
            self.finished.emit("connect", snapshot, "설정 응답을 확인했습니다. 값을 선택해 편집하세요.")
            self.timer.start()
        except (ValueError, OSError) as exc:
            self.failed.emit("connect", str(exc))

    @Slot()
    def poll(self):
        self._operate("poll", self.controller.read, "")

    @Slot()
    def read(self):
        self._operate("read", self.controller.read, "설정값을 새로 읽었습니다. 편집 중인 값은 유지했습니다.")

    @Slot(object, object)
    def apply(self, values, displayed):
        self._operate("apply", lambda: self.controller.apply_ram(values, displayed=displayed),
                      f"RAM {len(values)}개 적용 후 재조회로 확인했습니다. 영구 저장은 별도입니다.")

    @Slot(object)
    def burn(self, displayed):
        self._operate("burn", lambda: self.controller.burn(displayed=displayed),
                      "ROM 재조회 확인 완료. 연결 종료 후 전원을 껐다 켜고 유지 여부를 확인하세요.")

    def _operate(self, action, function, message):
        self.timer.stop()
        try:
            snapshot = function()
            self.finished.emit(action, snapshot, message)
            self.timer.start()
        except (ValueError, OSError) as exc:
            # Pause after a failed read; the user explicitly refreshes or reconnects.
            self.failed.emit(action, str(exc))

    @Slot()
    def disconnect(self):
        self.timer.stop()
        device, self.controller = self.controller, None
        try:
            if device:
                device.close()
            self.finished.emit("disconnect", {}, "연결을 종료했습니다. 읽기만 했어도 컨트롤러를 재부팅하세요.")
        except OSError as exc:
            self.failed.emit("disconnect", str(exc))

    @Slot()
    def shutdown(self):
        self.timer.stop()
        if self.controller:
            try:
                self.controller.close()
            except OSError:
                pass
            finally:
                self.controller = None


class Backend(QObject):
    connectRequested = Signal(str)
    readRequested = Signal()
    applyRequested = Signal(object, object)
    burnRequested = Signal(object)
    disconnectRequested = Signal()
    stateChanged = Signal()
    portsChanged = Signal()
    advancedChanged = Signal()

    def __init__(self):
        super().__init__()
        self.model = ParameterModel(self)
        self._connected = self._live = self._busy = self._error = False
        self._advanced = False
        self._port = self._last_read = self._error_detail = ""
        self._phase = "연결 안 됨"
        self._status = "RS-232 포트를 선택한 뒤 연결하세요."
        self._ports = []
        self._burn_preview = None
        self.thread = QThread(self)
        self.worker = SerialWorker()
        self.worker.moveToThread(self.thread)
        self.thread.finished.connect(self.worker.deleteLater)
        self.connectRequested.connect(self.worker.connect_port)
        self.readRequested.connect(self.worker.read)
        self.applyRequested.connect(self.worker.apply)
        self.burnRequested.connect(self.worker.burn)
        self.disconnectRequested.connect(self.worker.disconnect)
        self.worker.progress.connect(self._progress)
        self.worker.finished.connect(self._finished)
        self.worker.failed.connect(self._failed)
        self.thread.start()
        self.refreshPorts()

    @Property(bool, notify=stateChanged)
    def connected(self): return self._connected

    @Property(bool, notify=stateChanged)
    def live(self): return self._live

    @Property(bool, notify=stateChanged)
    def busy(self): return self._busy

    @Property(str, notify=stateChanged)
    def phase(self): return self._phase

    @Property(str, notify=stateChanged)
    def currentPort(self): return self._port

    @Property(str, notify=stateChanged)
    def status(self): return self._status

    @Property(str, notify=stateChanged)
    def lastRead(self): return self._last_read or "수신 이력 없음"

    @Property(str, notify=stateChanged)
    def mode(self):
        return physical_text(19, self.model.snapshot.get((19, 0))) + " · " + physical_text(20, self.model.snapshot.get((20, 0)))

    @Property(bool, notify=stateChanged)
    def hasError(self): return self._error

    @Property(str, notify=stateChanged)
    def errorDetail(self): return self._error_detail

    @Property("QVariantList", notify=portsChanged)
    def ports(self): return self._ports

    @Property("QStringList", constant=True)
    def categories(self): return CATEGORIES

    @Property(bool, notify=advancedChanged)
    def advancedUnlocked(self): return self._advanced

    @advancedUnlocked.setter
    def advancedUnlocked(self, enabled):
        self._advanced = bool(enabled)
        self.model.advanced = self._advanced
        self.model.notify_rows()
        self.advancedChanged.emit()

    def _start(self, phase, message):
        if self._busy:
            return False
        self._busy, self._error = True, False
        self._phase, self._status, self._error_detail = phase, message, ""
        self.stateChanged.emit()
        return True

    @Slot()
    def refreshPorts(self):
        devices = sorted(list_ports.comports(), key=lambda p: (p.vid is None, p.device))
        self._ports = [{"device": p.device, "label": f"{p.device} · {p.description if p.description != 'n/a' else '설명 없음'}"} for p in devices]
        self.portsChanged.emit()

    @Slot(str)
    def connectPort(self, port):
        if self._connected or not port.strip() or not self._start("포트 여는 중", "포트를 열고 설정 응답을 확인합니다."):
            return
        self._port, self._last_read = port.strip(), ""
        self.model.set_snapshot({}, clear_edits=True)
        self.connectRequested.emit(self._port)

    @Slot()
    def disconnect(self):
        if self._connected and self._start("연결 종료 중", "설정 세션을 닫고 포트를 해제합니다."):
            self.disconnectRequested.emit()

    @Slot()
    def refresh(self):
        if self._connected and self._start("새로 읽는 중", "장치 값을 새로 읽습니다. 편집값은 유지됩니다."):
            self.readRequested.emit()

    @Slot(int, str)
    def editParameter(self, i, text):
        if not self._busy and self._live:
            self.model.edit(i, text)

    @Slot(int)
    def selectParameter(self, i): self.model.select(i)

    @Slot(str)
    def selectGroup(self, group): self.model.set_group(group)

    @Slot()
    def resetEdits(self): self.model.reset_edits()

    @Slot()
    def applyRam(self):
        if not self._live or self.model.invalidCount or not self.model.dirtyCount:
            return
        try:
            values = prepare_writes(self.model.pending(), self._advanced)
        except ValueError as exc:
            self._status, self._error, self._error_detail = "입력값 또는 고급 편집 허용 여부를 확인하세요.", True, str(exc)
            self.stateChanged.emit()
            return
        if self._start("RAM 적용 중", f"{len(values)}개를 적용하고 재조회합니다."):
            self.applyRequested.emit(values, self.model.displayed_for_write())

    @Slot(result="QVariantList")
    def prepareBurn(self):
        # Bind the confirmation dialog to the values it actually showed.
        self._burn_preview = dict(self.model.snapshot)
        return self.model.burnChanges

    @Slot()
    def burn(self):
        if self._live and not self.model.dirtyCount and self._burn_preview is not None:
            if self._start("EEPROM 저장 중", "전체 RAM을 저장합니다. 5초 대기 후 ROM을 재조회합니다."):
                self.burnRequested.emit(self._burn_preview)
                self._burn_preview = None

    @Slot(QUrl)
    def importKy(self, url):
        if self._busy or not self._live:
            return
        try:
            count = self.model.import_values(read_ky(Path(url.toLocalFile()).read_bytes()))
            self._status, self._error = f"파일에서 {count}개 변경값을 불러왔습니다. 목록을 확인한 뒤 RAM에 적용하세요.", False
            self._error_detail = ""
        except (OSError, ValueError) as exc:
            self._status, self._error, self._error_detail = "설정 파일을 불러오지 못했습니다. 오류 상세를 확인하세요.", True, str(exc)
        self.stateChanged.emit()

    @Slot(QUrl, str)
    def exportKy(self, url, bank_name):
        if self._busy or not self._live:
            return
        try:
            path = Path(url.toLocalFile())
            if path.suffix.lower() != ".ky":
                path = path.with_suffix(".ky")
            payload = make_ky(self.model.snapshot, 1 if bank_name == "RAM" else 2)
            with path.open("xb") as stream:
                stream.write(payload)
            self._status, self._error, self._error_detail = f"{bank_name} 백업 저장: {path.name}", False, ""
        except FileExistsError:
            self._status, self._error = "같은 이름의 파일이 있습니다. 다른 이름으로 저장하세요.", True
        except (OSError, ValueError) as exc:
            self._status, self._error, self._error_detail = "백업 저장에 실패했습니다. 오류 상세를 확인하세요.", True, str(exc)
        self.stateChanged.emit()

    @Slot(str)
    def _progress(self, phase):
        self._phase = phase
        self.stateChanged.emit()

    @Slot(str, object, str)
    def _finished(self, action, snapshot, message):
        self.model.set_snapshot(snapshot, clear_edits=action in ("apply", "connect", "disconnect"))
        if action == "disconnect":
            self._connected = self._live = False
            self._phase = "연결 종료"
        else:
            self._connected = self._live = True
            self._last_read = datetime.now().strftime("%H:%M:%S")
        if action != "poll":
            self._busy, self._error, self._error_detail = False, False, ""
            self._status = message
        if not self._busy and self._live:
            self._phase = "설정 응답 수신 중"
        self.stateChanged.emit()

    @Slot(str, str)
    def _failed(self, action, message):
        self._live = False
        if action != "poll":
            self._busy = False
        self._error, self._error_detail = True, message
        if action in ("connect", "disconnect"):
            self._connected = False
            self._phase = "연결 실패" if action == "connect" else "연결 종료"
            self._status = "포트 경로·접근 권한·RS-232 배선·정상 기동 여부를 확인한 뒤 다시 연결하세요."
        else:
            self._phase = "응답 확인 필요"
            self._status = "마지막 수신값을 표시합니다. 새로고침하거나 연결을 종료한 뒤 다시 연결하세요."
            if action in ("apply", "burn"):
                self._status = "작업 결과를 확인하지 못했습니다. 일부 값이 변경됐을 수 있으니 먼저 새로고침하세요."
        self.stateChanged.emit()

    def shutdown_and_wait(self):
        if self.thread.isRunning():
            QMetaObject.invokeMethod(self.worker, "shutdown", Qt.BlockingQueuedConnection)
            self.thread.quit()
            self.thread.wait()


def main():
    QQuickStyle.setStyle("Basic")
    app = QGuiApplication(sys.argv)
    app.setApplicationName("KEYA 설정")
    backend = Backend()
    engine = QQmlApplicationEngine()
    engine.rootContext().setContextProperty("backend", backend)
    engine.rootContext().setContextProperty("parameterModel", backend.model)
    engine.load(QUrl.fromLocalFile(str(Path(__file__).with_name("qml") / "Main.qml")))
    try:
        if not engine.rootObjects():
            return 1
        app.aboutToQuit.connect(backend.shutdown_and_wait)
        return app.exec()
    finally:
        backend.shutdown_and_wait()
        del engine


if __name__ == "__main__":
    raise SystemExit(main())
