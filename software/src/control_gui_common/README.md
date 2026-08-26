# control_gui_common

Control 저장소의 Qt GUI가 공유하는 최소 runtime helper 패키지다. GUI 화면이나
ROS topic 자체를 소유하지 않는다.

`qt_runtime.ensure_bundled_qt_first()`는 PyPI PyQt5에 포함된 Qt와 시스템 Qt가
`LD_LIBRARY_PATH`에서 섞여 xcb plugin 로딩이 실패하는 환경에서 bundled Qt
library 경로를 앞에 둔 채 프로세스를 한 번만 재실행한다. 시스템 PyQt5처럼
bundled Qt가 없으면 아무 동작도 하지 않는다.

현재 사용처:

- `kaiev26_hil.hil_gui`
- `visualization.vehicle_dashboard`

이 패키지는 두 GUI 사이의 역방향 의존성을 없애기 위한 중립 경계다. 새로운 GUI
공통 동작이 실제로 두 소비자에게 필요할 때만 여기에 추가한다.
