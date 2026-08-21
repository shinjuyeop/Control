#!/usr/bin/env python3
"""Team KAI vcan simulator 실행 진입점.

이 파일은 실행 옵션만 해석하고,
실제 CAN runtime과 GUI는 분리된 모듈에 위임한다.
"""
from __future__ import annotations

import argparse
import signal
import time

from .app import VcanSimRuntime


def parse_args() -> argparse.Namespace:
    """실행 모드와 CAN 인터페이스를 명령줄에서 읽는다."""
    parser = argparse.ArgumentParser(description="Team KAI vcan simulator")
    parser.add_argument("--motor-iface", default="vcan0", help="motor CAN interface")
    parser.add_argument("--steer-iface", default="vcan1", help="steering CAN interface")
    parser.add_argument("--no-gui", action="store_true", help="run without Tk dashboard")
    parser.add_argument(
        "--ros-no-gui", choices=("true", "false"), default="false",
        help="launch-file compatible form of --no-gui",
    )
    parser.add_argument("--headless-demo", action="store_true", help="send example commands in headless mode")
    parser.add_argument("--duration", type=float, default=5.0, help="headless run duration")
    return parser.parse_args()


def main() -> int:
    """headless 또는 GUI 모드로 lab runtime을 실행한다."""
    args = parse_args()
    runtime = VcanSimRuntime(args.motor_iface, args.steer_iface)
    try:
        if args.no_gui or args.ros_no_gui == "true":
            # GUI 없이도 vcan 위에서 동일한 runtime을 돌린다.
            try:
                if args.headless_demo:
                    runtime.run_headless_demo(args.duration)
                else:
                    time.sleep(args.duration)
            except KeyboardInterrupt:
                # Ctrl+C 종료 시 traceback 없이 빠져나온다.
                pass
        else:
            # Tk 루프는 여기서만 열고, 실제 상태 갱신은 runtime이 담당한다.
            import tkinter as tk
            from .hmi import DashboardApp

            root = tk.Tk()
            app = DashboardApp(root, runtime)

            old_sigint = signal.getsignal(signal.SIGINT)

            def _request_gui_shutdown(signum, frame) -> None:
                """Ctrl+C를 예외 대신 정상 종료 요청으로 바꾼다."""
                app.request_close()

            signal.signal(signal.SIGINT, _request_gui_shutdown)
            try:
                root.mainloop()
            except KeyboardInterrupt:
                # 예외 기반 SIGINT가 들어와도 조용히 닫는다.
                app.request_close()
            finally:
                signal.signal(signal.SIGINT, old_sigint)
                try:
                    if root.winfo_exists():
                        root.destroy()
                except tk.TclError:
                    pass
    finally:
        runtime.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
