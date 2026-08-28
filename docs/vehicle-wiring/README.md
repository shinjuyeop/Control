# Vehicle Wiring Draw.io

이 디렉터리는 자작차 배선도, 부품별 Draw.io 커스텀 라이브러리, 팀 공용
VS Code 설정을 함께 관리합니다.

## 처음 한 번만 설치

저장소를 clone 또는 pull한 다음 아래 명령을 그대로 실행합니다.

```bash
cd docs/vehicle-wiring
code --install-extension tools/vscode-drawio-wiring.vsix --force
code vehicle-wiring.code-workspace
```

VS Code에서 `Ctrl+Shift+P`를 누르고 `Developer: Reload Window`를 한 번
실행합니다. 그다음 `vehicle.drawio`를 엽니다.

VSCodium을 쓰는 경우 위 명령의 `code`만 `codium`으로 바꿉니다.

## 정상 화면

Draw.io 왼쪽 도형 패널에 다음 드롭다운이 자동으로 나타납니다.

```text
Development Boards
Connectors
```

`File > Open Library`로 XML을 수동 Import하지 않습니다. 수동 Import는 파일의
현재 내용을 복사한 스냅샷이라 이후 XML 변경을 따라가지 않습니다.

예전에 수동으로 연 `nucleo-g474re` 패널이 남아 있으면 패널 제목 오른쪽의
`×`로 한 번 닫습니다.

## 기본 Draw.io 팔레트 추가

왼쪽 아래 `More Shapes`를 누르고 `Electrical`처럼 필요한 기본 팔레트를 체크한
뒤 `Apply`를 누릅니다. 선택한 기본 팔레트는 `vehicle-wiring` 팔레트와 함께
저장되며 Window를 리로드해도 유지됩니다.

## 부품 XML 추가

부품 하나당 XML 파일 하나를 해당 카테고리 디렉터리에 넣습니다.

```text
libraries/
├── development-boards/
│   ├── nucleo-g474re.xml
│   └── arduino-uno-r4-minima.xml
└── connectors/
    ├── ms-14s-6-6pin.xml
    ├── ms-18-1-10pin.xml
    ├── ms-20-27-14pin.xml
    └── ...
```

파일을 저장하면 열린 Draw.io 패널이 자동으로 다시 로드됩니다.

- 기존 XML 수정: 같은 도형이 최신 내용으로 갱신됩니다.
- 새 XML 생성: 해당 카테고리 드롭다운에 자동으로 추가됩니다.
- XML 삭제: 해당 카테고리 드롭다운에서도 제거됩니다.
- 설정 파일 수정이나 수동 재Import는 필요 없습니다.

이미 도면에 배치한 도형은 라이브러리 원본과 연결된 인스턴스가 아니므로 자동
교체되지 않습니다. 자동 갱신되는 대상은 왼쪽 라이브러리 패널의 원본입니다.

## 새 카테고리 추가

예를 들어 CAN transceiver 모듈을 별도로 모으려면 다음 디렉터리를 만듭니다.

```text
libraries/interface-modules/
```

그리고 `vehicle-wiring.code-workspace`의
`hediet.vscode-drawio.customLibraries` 배열에 한 항목만 추가합니다.

```json
{
  "entryId": "vehicle-wiring",
  "libName": "Interface Modules",
  "directory": "${workspaceFolder}/libraries/interface-modules"
}
```

이 설정과 새 디렉터리를 Git에 같이 올리면 다른 팀원도 pull 후 같은 메뉴를
봅니다.

## 팀원이 pull한 뒤

일반적인 부품 XML 추가·수정만 있었다면 별도 설치 작업 없이 파일을 열면 됩니다.
열려 있던 VS Code에서 pull했다면 XML 변경 이벤트를 받아 패널이 자동
갱신됩니다.

`tools/vscode-drawio-wiring.vsix`가 변경된 경우에만 다시 설치하고 Window를 한 번
리로드합니다.

```bash
cd docs/vehicle-wiring
code --install-extension tools/vscode-drawio-wiring.vsix --force
```

## 확인 및 문제 해결

설치된 watcher 확장 버전은 다음 명령으로 확인합니다.

```bash
code --list-extensions --show-versions | grep '^hediet.vscode-drawio@'
```

정상 설치 결과는 다음과 같습니다.

```text
hediet.vscode-drawio@999.0.0-alpha
```

드롭다운이 보이지 않으면 다음 순서만 확인합니다.

1. `vehicle-wiring.code-workspace`를 열었는지 확인합니다.
2. 위 명령으로 watcher 확장 버전을 확인합니다.
3. VSIX를 `--force`로 다시 설치합니다.
4. `Developer: Reload Window`를 한 번 실행합니다.

공식 draw.io MCP나 localhost 웹 에디터는 필요하지 않습니다. `.drawio`와
`libraries/**/*.xml`이 설계 원본이며 모두 Git으로 관리합니다.
