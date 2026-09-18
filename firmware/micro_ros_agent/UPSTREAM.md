# Upstream source

This directory vendors `micro-ROS/micro-ROS-Agent` from its Humble branch at
commit `4f363a79ae96aed39dcc5269f53f73f6a70d3a2c`.

It also vendors the required `micro-ROS/micro_ros_msgs` Humble branch at commit
`100bf269e78da3fe0a58f6531ebe47d6991bd9ab`.

The packages discovered by colcon are `micro_ros_agent/` and
`micro_ros_msgs/` below this directory.

`micro_ros_agent/cmake/SuperBuild.cmake` is locally adjusted so its
Micro-XRCE-DDS Agent dependency links against the Fast DDS selected by the
active ROS workspace rather than an unrelated `/usr/local` installation.
`micro_ros_agent/CMakeLists.txt` selects Fast DDS/Fast CDR from the active ROS
workspace and gives the executable an `$ORIGIN/..` runtime path so it loads the
packaged Micro-XRCE-DDS library before a global copy with the same SONAME.

`micro_ros_agent/src/agent/Agent.cpp` applies `ROS_LOCALHOST_ONLY=1` before
starting the XRCE worker: default DDS participants use loopback UDP and shared
memory, matching ROS Humble's localhost transports. This covers the STM_A/B
binary participant creation path and the Agent's ROS graph participants for
both direct and launch-based execution. Unset or `0` keeps the upstream behavior.

## 저장소 문서 관리 범위

Control의 Agent 빌드·serial 실행은 [상위 firmware 문서](../README.md)를 따른다.
이 디렉터리의 upstream README·CONTRIBUTING·issue template은 해당 commit의
원문으로 유지한다. 원문에 남은 `micro_ros_agent/README.md`의 `TODO` 링크와
`micro_ros_msgs/CONTRIBUTING.md`의 미포함 `NOTICE` 참조는 로컬 운용 경로가 아니다.
