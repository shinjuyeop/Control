pragma ComponentBehavior: Bound

import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs

ApplicationWindow {
    id: root
    width: 1270
    height: 800
    minimumWidth: 1080
    minimumHeight: 660
    visible: true
    title: "KEYA 설정 · V4.3"
    color: "#ffffff"
    font.family: "Noto Sans CJK KR"
    font.pixelSize: 13
    palette.window: "#ffffff"
    palette.base: "#ffffff"
    palette.text: "#20252b"
    palette.windowText: "#20252b"
    palette.buttonText: "#20252b"
    palette.highlight: "#245c91"
    palette.highlightedText: "#ffffff"

    readonly property color ink: "#20252b"
    readonly property color muted: "#606974"
    readonly property color line: "#d9dee4"
    readonly property color accent: "#245c91"
    readonly property color warning: "#87520f"
    readonly property color danger: "#aa2929"
    property string selectedGroup: backend.categories[0]
    property var detail: parameterModel.detail
    property bool closeApproved: false

    onClosing: function(event) {
        if (!closeApproved && (backend.connected || backend.busy || parameterModel.dirtyCount)) {
            event.accepted = false
            closeDialog.open()
        }
    }

    component Tip: ToolTip {
        id: tip
        delay: 450
        timeout: 9000
        padding: 8
        background: Rectangle { color: "#fffff4"; border.color: "#b8b8a5" }
        contentItem: Text { text: tip.text; font.pixelSize: 12; color: root.ink; wrapMode: Text.Wrap; width: Math.min(360, implicitWidth) }
    }

    component Action: Button {
        id: button
        property string help: ""
        property bool primary: false
        implicitHeight: 32
        leftPadding: 12
        rightPadding: 12
        hoverEnabled: true
        Accessible.name: text
        contentItem: Text {
            text: button.text; font: button.font
            color: !button.enabled ? "#9299a1" : (button.primary ? "#ffffff" : root.ink)
            horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
        }
        background: Rectangle {
            color: !button.enabled ? "#f1f3f5" : button.primary ? (button.down ? "#19456d" : root.accent) : (button.hovered ? "#eaf0f6" : "#ffffff")
            border.color: button.activeFocus ? root.accent : button.enabled ? "#9da9b6" : root.line
            border.width: button.activeFocus ? 2 : 1
        }
        Tip { visible: button.hovered && button.help.length > 0; text: button.help }
    }

    component Note: Text {
        id: note
        property string help: ""
        color: root.muted
        font.pixelSize: 12
        wrapMode: Text.Wrap
        HoverHandler { id: noteHover }
        Tip { visible: noteHover.hovered && note.help.length > 0; text: note.help }
    }

    component Heading: Text {
        color: root.ink
        font.pixelSize: 13
        font.weight: Font.DemiBold
        Layout.topMargin: 8
    }

    component PanelDialog: Dialog {
        id: dialog
        anchors.centerIn: parent
        width: 540
        padding: 20
        modal: true
        focus: true
        background: Rectangle { color: "#ffffff"; border.color: "#8c99a8" }
        header: Text { text: dialog.title; padding: 20; bottomPadding: 12; font.pixelSize: 17; font.weight: Font.DemiBold; color: root.ink }
    }

    PanelDialog {
        id: startupDialog
        objectName: "startupDialog"
        title: "설정 모드 연결"
        contentItem: ColumnLayout {
            spacing: 14
            Note { Layout.fillWidth: true; text: "컨트롤러가 정상 기동한 뒤 연결하세요. 제조사는 전원 투입 순간 설정을 읽으면 파라미터가 초기값으로 리셋될 수 있다고 경고합니다." }
            Note { Layout.fillWidth: true; text: "읽기만 했더라도 연결을 종료한 뒤 전원을 껐다 켜야 합니다. 모터를 정차한 상태에서 작업하세요."; color: root.warning }
            RowLayout {
                Layout.alignment: Qt.AlignRight
                Action { text: "취소"; onClicked: startupDialog.close() }
                Action { text: "정상 기동 확인 · 연결"; primary: true; onClicked: { startupDialog.close(); backend.connectPort(portBox.chosenPort()) } }
            }
        }
    }

    PanelDialog {
        id: closeDialog
        title: "설정 도구 종료"
        contentItem: ColumnLayout {
            spacing: 14
            Note { Layout.fillWidth: true; text: backend.busy ? "진행 중인 작업과 포트 정리가 끝난 뒤 종료합니다." : "연결을 종료하고 도구를 닫습니다. 적용하지 않은 편집값은 사라집니다." }
            Note { Layout.fillWidth: true; text: "설정 세션을 사용했다면 컨트롤러 전원을 껐다 켜세요." }
            RowLayout { Layout.alignment: Qt.AlignRight
                Action { text: "계속 작업"; onClicked: closeDialog.close() }
                Action { text: "종료"; onClicked: { root.closeApproved = true; root.close() } }
            }
        }
    }

    PanelDialog {
        id: burnDialog
        objectName: "burnDialog"
        property var changes: []
        width: 640
        title: "전체 RAM을 EEPROM에 저장"
        contentItem: ColumnLayout {
            spacing: 12
            Note { Layout.fillWidth: true; text: "ROM과 다른 항목은 아래와 같습니다. 명령은 이 항목들을 포함한 전체 RAM을 영구 저장합니다." }
            RowLayout {
                Layout.fillWidth: true
                Text { Layout.fillWidth: true; text: "항목"; font.weight: Font.DemiBold }
                Text { Layout.preferredWidth: 120; text: "저장된 ROM"; font.weight: Font.DemiBold }
                Text { Layout.preferredWidth: 120; text: "저장할 RAM"; font.weight: Font.DemiBold }
            }
            ListView {
                Layout.fillWidth: true
                Layout.preferredHeight: Math.min(300, contentHeight)
                clip: true
                model: burnDialog.changes
                ScrollBar.vertical: ScrollBar {}
                delegate: Rectangle {
                    required property var modelData
                    width: ListView.view.width; height: 44; color: "#f5f7f9"
                    RowLayout { anchors.fill: parent; anchors.margins: 6
                        Text { Layout.fillWidth: true; text: modelData.code + "  " + modelData.parameterName; elide: Text.ElideRight }
                        Column { Layout.preferredWidth: 120
                            Text { text: modelData.romValue }
                            Note { text: modelData.romMeaning; font.pixelSize: 10 }
                        }
                        Column { Layout.preferredWidth: 120
                            Text { text: modelData.ramValue; color: root.accent }
                            Note { text: modelData.ramMeaning; font.pixelSize: 10 }
                        }
                    }
                }
            }
            Note { Layout.fillWidth: true; text: "5초 대기 후 ROM을 다시 읽어 확인합니다. 작업 후 재부팅하고, 다시 연결해 값이 유지되는지 확인하세요." }
            RowLayout { Layout.alignment: Qt.AlignRight
                Action { text: "취소"; onClicked: burnDialog.close() }
                Action { text: "전체 RAM 영구 저장"; primary: true; enabled: backend.live && !backend.busy; onClicked: { burnDialog.close(); backend.burn() } }
            }
        }
    }

    PanelDialog {
        id: capabilityDialog
        width: 650
        title: "원본 프로그램과 지원 범위"
        contentItem: ColumnLayout {
            spacing: 12
            Heading { text: "현재 지원 · 8바이트 Setup" }
            Note { Layout.fillWidth: true; text: "0000–0032, 0041, 0063의 35개 항목 조회와 TAG를 제외한 34개 항목 편집. RAM 재조회, EEPROM 저장·재조회, RAM/ROM 백업과 .ky 불러오기." }
            Heading { text: "원본에는 있지만 아직 없는 기능" }
            Note { Layout.fillWidth: true; text: "운전 활성화·비활성화, 속도·토크·위치 직접 명령, 전류·속도·위치·온도·고장 상태 모니터, 170 3세대 4바이트 프로토콜, 듀얼 모터와 검사 화면." }
            Heading { text: "동작 차이" }
            Note { Layout.fillWidth: true; text: "원본의 파일 불러오기는 RAM 전송 뒤 Burn까지 실행합니다. 이 도구는 편집 목록에서 차이를 검토하고 RAM 적용과 EEPROM 저장을 각각 실행합니다. 설정 응답만으로 모터 모델이나 구동 가능 상태를 판별하지 않습니다." }
            Action { Layout.alignment: Qt.AlignRight; text: "닫기"; onClicked: capabilityDialog.close() }
        }
    }

    PanelDialog {
        id: errorDialog
        title: "오류 상세"
        contentItem: ColumnLayout {
            Note { Layout.fillWidth: true; text: backend.status }
            TextArea { Layout.fillWidth: true; Layout.preferredHeight: 160; readOnly: true; selectByMouse: true; text: backend.errorDetail; wrapMode: TextEdit.Wrap }
            Action { Layout.alignment: Qt.AlignRight; text: "닫기"; onClicked: errorDialog.close() }
        }
    }

    FileDialog {
        id: importDialog
        title: "설정 파일 불러오기"
        fileMode: FileDialog.OpenFile
        nameFilters: ["KEYA 설정 (*.ky)", "모든 파일 (*)"]
        acceptLabel: "불러오기"
        rejectLabel: "취소"
        onAccepted: backend.importKy(selectedFile)
    }
    FileDialog {
        id: exportDialog
        property string bank: "ROM"
        title: bank + " 백업 저장"
        fileMode: FileDialog.SaveFile
        nameFilters: ["KEYA 설정 (*.ky)"]
        defaultSuffix: "ky"
        acceptLabel: "저장"
        rejectLabel: "취소"
        onAccepted: backend.exportKy(selectedFile, bank)
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0
        Rectangle {
            Layout.fillWidth: true; implicitHeight: 106; color: "#f4f6f8"
            ColumnLayout { anchors.fill: parent; anchors.margins: 14; spacing: 8
                RowLayout {
                    Text { text: "KEYA 설정"; color: root.ink; font.pixelSize: 19; font.weight: Font.DemiBold }
                    Note { text: "V4.3 · 8바이트 Setup"; help: "원본의 ‘Not 170 Three generations’ 설정 프로토콜입니다." }
                    Item { Layout.fillWidth: true }
                    Note { text: "마지막 수신  " + backend.lastRead; help: "설정 스트림의 한 주기를 마지막으로 읽은 시각입니다. 모터 구동 상태는 뜻하지 않습니다." }
                    Text { text: backend.phase; color: backend.hasError ? root.danger : backend.live ? "#236545" : root.muted; font.weight: Font.DemiBold }
                }
                RowLayout {
                    spacing: 8
                    Text { text: "RS-232 포트" }
                    ComboBox {
                        id: portBox
                        objectName: "portBox"
                        Layout.preferredWidth: 330; implicitHeight: 32
                        model: backend.ports
                        textRole: "label"
                        valueRole: "device"
                        editable: true
                        enabled: !backend.connected && !backend.busy
                        function chosenPort() { return editText.split(" · ")[0].trim() }
                        contentItem: TextInput {
                            text: backend.connected ? backend.currentPort : portBox.editText
                            font: portBox.font; color: root.ink; padding: 6; rightPadding: 22
                            enabled: portBox.enabled; selectByMouse: true
                            onTextEdited: portBox.editText = text
                            Text { visible: !parent.text.length; anchors.verticalCenter: parent.verticalCenter; x: 6; text: "포트 선택 또는 경로 입력"; color: root.muted; font.pixelSize: 12 }
                        }
                        background: Rectangle { color: "#ffffff"; border.color: root.line }
                        popup: Popup {
                            y: portBox.height
                            width: portBox.width; padding: 1
                            implicitHeight: Math.min(240, portsList.contentHeight + 2)
                            background: Rectangle { color: "#ffffff"; border.color: root.line }
                            contentItem: ListView { id: portsList; clip: true; model: portBox.popup.visible ? portBox.delegateModel : null; currentIndex: portBox.highlightedIndex; ScrollIndicator.vertical: ScrollIndicator {} }
                        }
                        Tip { visible: portHover.hovered; text: "USB 어댑터 이름과 장치 경로입니다. /dev/serial/by-id/ 경로를 직접 입력할 수도 있습니다." }
                        HoverHandler { id: portHover }
                    }
                    Action { text: "포트 찾기"; help: "현재 연결된 시리얼 포트 목록을 다시 검색합니다."; enabled: !backend.connected && !backend.busy; onClicked: backend.refreshPorts() }
                    Action { text: backend.connected ? "연결 종료" : "연결"; primary: !backend.connected; enabled: !backend.busy && (backend.connected || portBox.chosenPort().length > 0); onClicked: backend.connected ? backend.disconnect() : startupDialog.open() }
                    Item { Layout.fillWidth: true }
                    Note { text: "115200 · 8N1"; help: "115200 bit/s, 데이터 8비트, 패리티 없음, 정지 1비트. TTL UART가 아닌 RS-232 레벨 어댑터를 사용하세요." }
                    Action { text: "새로고침"; help: "장치 값을 다시 읽습니다. 편집값은 유지됩니다. 실패로 멈춘 자동 갱신도 다시 시작합니다."; enabled: backend.connected && !backend.busy; onClicked: backend.refresh() }
                }
            }
        }
        Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: root.line }

        RowLayout {
            Layout.fillWidth: true; Layout.fillHeight: true; spacing: 0
            Rectangle {
                Layout.preferredWidth: 154; Layout.fillHeight: true; color: "#f8f9fb"
                ColumnLayout {
                    anchors.fill: parent; anchors.margins: 10; spacing: 4
                    Heading { text: "설정 분류"; Layout.bottomMargin: 8 }
                    Repeater {
                        model: backend.categories
                        delegate: Button {
                            id: nav
                            required property string modelData
                            Layout.fillWidth: true; implicitHeight: 36
                            text: modelData; hoverEnabled: true
                            contentItem: Text { text: nav.text; color: root.selectedGroup === nav.text ? root.accent : root.ink; font.weight: root.selectedGroup === nav.text ? Font.DemiBold : Font.Normal; verticalAlignment: Text.AlignVCenter; leftPadding: 9 }
                            background: Rectangle { color: root.selectedGroup === nav.text ? "#e5edf5" : nav.hovered ? "#eef1f4" : "transparent"
                                Rectangle { width: 3; height: parent.height; color: root.accent; visible: root.selectedGroup === nav.text }
                            }
                            onClicked: { root.selectedGroup = modelData; backend.selectGroup(modelData) }
                        }
                    }
                    Item { Layout.fillHeight: true }
                    Action { Layout.fillWidth: true; text: "지원 기능 확인"; help: "Windows 원본 EXE와 현재 도구의 기능 차이를 확인합니다."; onClicked: capabilityDialog.open() }
                }
            }
            Rectangle { Layout.fillHeight: true; implicitWidth: 1; color: root.line }

            ColumnLayout {
                Layout.fillWidth: true; Layout.fillHeight: true
                Layout.margins: 14; spacing: 10
                RowLayout {
                    Text { text: root.selectedGroup; font.pixelSize: 17; font.weight: Font.DemiBold; color: root.ink }
                    Item { Layout.fillWidth: true }
                    Action { text: "파일 불러오기"; help: "Windows .ky 파일의 변경값을 편집란으로 불러옵니다. RAM 적용은 별도로 실행합니다."; enabled: backend.live && !backend.busy; onClicked: importDialog.open() }
                    Action { id: backup; text: "백업 ▾"; help: "마지막 수신한 RAM 또는 ROM을 .ky 파일로 저장합니다."; enabled: backend.live && !backend.busy; onClicked: backupMenu.open()
                        Menu { id: backupMenu; y: backup.height
                            background: Rectangle { color: "#ffffff"; border.color: root.line }
                            MenuItem { text: "ROM 백업 저장"; onTriggered: { exportDialog.bank = "ROM"; exportDialog.open() } }
                            MenuItem { text: "RAM 백업 저장"; onTriggered: { exportDialog.bank = "RAM"; exportDialog.open() } }
                        }
                    }
                }
                RowLayout {
                    Note { Layout.fillWidth: true; text: backend.live ? "수신값 자동 갱신 중 · 편집값은 별도 유지" : backend.connected ? "수신 확인 필요 · 마지막 값 표시" : "연결하면 장치에서 읽은 값이 표시됩니다." }
                    CheckBox {
                        id: advanced
                        hoverEnabled: true
                        text: "고급 편집 허용"; checked: backend.advancedUnlocked; enabled: !backend.busy
                        onToggled: backend.advancedUnlocked = checked
                        indicator: Rectangle { implicitWidth: 15; implicitHeight: 15; x: 0; y: (advanced.height - height)/2; border.color: "#8895a4"; color: advanced.checked ? root.accent : "#ffffff"
                            Text { anchors.centerIn: parent; text: "✓"; color: "white"; visible: advanced.checked; font.pixelSize: 12 }
                        }
                        contentItem: Text { text: advanced.text; font.pixelSize: 12; leftPadding: 21; color: root.muted; verticalAlignment: Text.AlignVCenter }
                        Tip { visible: advanced.hovered; text: "보호·통신·모터·예비 설정의 편집을 허용합니다. TAG는 읽기 전용입니다." }
                    }
                }
                Rectangle {
                    Layout.fillWidth: true; implicitHeight: 30; color: "#eef1f4"
                    RowLayout { anchors.fill: parent; anchors.leftMargin: 8; anchors.rightMargin: 8; spacing: 8
                        Note { Layout.preferredWidth: 39; text: "번호" }
                        Note { Layout.fillWidth: true; text: "파라미터"; help: "선택하면 오른쪽에 상세 설명을 표시합니다." }
                        Note { Layout.preferredWidth: 64; text: "적용값"; help: "Param: 컨트롤러가 보고하는 적용 파라미터입니다." }
                        Note { Layout.preferredWidth: 64; text: "RAM"; help: "장치에서 수신한 작업 메모리 값. 주황색은 ROM과 다른 값입니다." }
                        Note { Layout.preferredWidth: 64; text: "ROM"; help: "장치에서 수신한 EEPROM 저장값입니다." }
                        Note { Layout.preferredWidth: 82; text: "편집값"; help: "이 PC의 입력값입니다. ‘RAM에 적용’을 눌러야 전송됩니다." }
                    }
                }
                ListView {
                    id: parameterList
                    objectName: "parameterList"
                    Layout.fillWidth: true; Layout.fillHeight: true
                    clip: true; model: parameterModel
                    boundsBehavior: Flickable.StopAtBounds
                    ScrollBar.vertical: ScrollBar {}
                    section.property: "sectionName"
                    section.criteria: ViewSection.FullString
                    section.delegate: Rectangle {
                        required property string section
                        width: parameterList.width; height: 28; color: "#ffffff"
                        Text { anchors.left: parent.left; anchors.leftMargin: 8; anchors.verticalCenter: parent.verticalCenter; text: section; font.pixelSize: 11; font.weight: Font.DemiBold; color: root.muted }
                    }
                    delegate: Rectangle {
                        id: row
                        required property int paramId
                        required property string code
                        required property string parameterName
                        required property string tooltip
                        required property string paramValue
                        required property string ramValue
                        required property string romValue
                        required property string editorValue
                        required property string ramMeaning
                        required property string romMeaning
                        required property string inputError
                        required property bool editable
                        required property bool selected
                        required property bool dirty
                        required property bool unsaved
                        width: parameterList.width; height: 38
                        color: selected ? "#eaf1f8" : hover.hovered ? "#f6f8fa" : "#ffffff"
                        HoverHandler { id: hover }
                        TapHandler { onTapped: backend.selectParameter(row.paramId) }
                        Rectangle { width: parent.width; height: 1; anchors.bottom: parent.bottom; color: "#edf0f3" }
                        RowLayout {
                            anchors.fill: parent; anchors.leftMargin: 8; anchors.rightMargin: 8; spacing: 8
                            Note { Layout.preferredWidth: 39; text: row.code; font.family: "monospace" }
                            Text { Layout.fillWidth: true; text: row.parameterName; color: root.ink; font.pixelSize: 12; elide: Text.ElideRight
                                HoverHandler { id: nameHover }
                                Tip { visible: nameHover.hovered; text: row.tooltip }
                            }
                            Note { Layout.preferredWidth: 64; text: row.paramValue; color: root.ink; font.family: "monospace" }
                            Note { Layout.preferredWidth: 64; text: row.ramValue; color: row.unsaved ? root.warning : root.ink; font.family: "monospace"; help: row.ramMeaning + (row.unsaved ? "\nROM과 다릅니다. 아직 영구 저장되지 않았습니다." : "") }
                            Note { Layout.preferredWidth: 64; text: row.romValue; color: root.ink; font.family: "monospace"; help: row.romMeaning }
                            TextField {
                                id: editor
                                hoverEnabled: true
                                objectName: "edit_" + row.paramId
                                Layout.preferredWidth: 82; implicitHeight: 28
                                text: row.editorValue
                                enabled: backend.live && row.editable && !backend.busy
                                selectByMouse: true; leftPadding: 6; rightPadding: 6
                                color: !row.inputError.length ? root.ink : root.danger
                                font.family: "monospace"; font.pixelSize: 12
                                Accessible.name: row.parameterName + " 편집값"
                                onTextEdited: backend.editParameter(row.paramId, text)
                                onActiveFocusChanged: if (activeFocus) backend.selectParameter(row.paramId)
                                background: Rectangle { color: !editor.enabled ? "#f3f5f7" : row.dirty ? "#fff4df" : "#ffffff"; border.color: row.inputError.length ? root.danger : editor.activeFocus ? root.accent : "#bfc7d0" }
                                Tip { visible: editor.hovered; text: row.inputError || (row.editable ? "로컬 편집값입니다. RAM 적용 전에는 장치 값이 바뀌지 않습니다." : "읽기 전용 또는 고급 편집 잠금 상태입니다.") }
                            }
                        }
                    }
                }
                Note { Layout.fillWidth: true; text: backend.connected ? "항목 선택: 상세 설명  ·  입력 후 RAM 적용  ·  ROM과 비교 후 영구 저장" : "기본값을 자동 입력하지 않습니다. 먼저 정상 기동을 확인하고 연결하세요."; font.pixelSize: 11 }
            }
            Rectangle { Layout.fillHeight: true; implicitWidth: 1; color: root.line }

            ScrollView {
                Layout.preferredWidth: 294; Layout.fillHeight: true
                clip: true; contentWidth: availableWidth
                ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
                ColumnLayout {
                    width: parent.width - 28; x: 14; y: 14
                    spacing: 9
                    Note { text: "선택한 파라미터 · " + root.detail.code; font.pixelSize: 11 }
                    Text { Layout.fillWidth: true; text: root.detail.parameterName; wrapMode: Text.Wrap; font.pixelSize: 18; font.weight: Font.DemiBold; color: root.ink }
                    Note { Layout.fillWidth: true; text: "원문: " + root.detail.original; font.pixelSize: 11; help: "원본 Windows 프로그램과 항목을 대조할 때 사용하는 이름입니다." }
                    Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: root.line; Layout.topMargin: 4; Layout.bottomMargin: 4 }
                    Note { Layout.fillWidth: true; text: root.detail.description; color: root.ink; font.pixelSize: 13; lineHeight: 1.25 }
                    Heading { text: "값의 의미" }
                    GridLayout {
                        columns: 2; rowSpacing: 6; columnSpacing: 10; Layout.fillWidth: true
                        Note { text: "장치 RAM" }
                        Text { Layout.fillWidth: true; wrapMode: Text.Wrap; text: root.detail.ramValue + (root.detail.ramMeaning ? " · " + root.detail.ramMeaning : ""); color: root.ink }
                        Note { text: "저장 ROM" }
                        Text { Layout.fillWidth: true; wrapMode: Text.Wrap; text: root.detail.romValue + (root.detail.romMeaning ? " · " + root.detail.romMeaning : ""); color: root.ink }
                        Note { text: "편집값" }
                        Text { Layout.fillWidth: true; text: (root.detail.editorValue || "—") + (root.detail.editorMeaning ? " · " + root.detail.editorMeaning : ""); wrapMode: Text.Wrap; color: root.detail.dirty ? root.warning : root.ink }
                    }
                    Note { Layout.fillWidth: true; visible: root.detail.inputError.length > 0; text: root.detail.inputError; color: root.danger }
                    Note { Layout.fillWidth: true; visible: root.detail.conflict; text: "편집을 시작한 뒤 장치 RAM이 바뀌었습니다. 편집 취소 후 현재 값을 기준으로 다시 입력하세요."; color: root.danger }
                    Heading { text: "입력 안내" }
                    Note { Layout.fillWidth: true; text: root.detail.reference; color: root.ink }
                    Note { Layout.fillWidth: true; text: root.detail.access + " · 입력 분해능 " + root.detail.resolution; font.pixelSize: 11 }
                    Heading { text: "참고" }
                    Note { Layout.fillWidth: true; text: root.detail.note; lineHeight: 1.25 }
                    Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: root.line; Layout.topMargin: 12 }
                    Heading { text: "장치 응답" }
                    Note { Layout.fillWidth: true; text: "마지막 수신  " + backend.lastRead }
                    Note { Layout.fillWidth: true; text: "보고된 적용 모드  " + backend.mode }
                    Note { Layout.fillWidth: true; text: "설정 응답은 모터 구동 가능 여부나 모델 식별 결과가 아닙니다."; font.pixelSize: 11; Layout.bottomMargin: 20 }
                }
            }
        }
        Rectangle { Layout.fillWidth: true; implicitHeight: 1; color: root.line }
        Rectangle {
            Layout.fillWidth: true; implicitHeight: 88; color: "#f5f7f9"
            ColumnLayout { anchors.fill: parent; anchors.margins: 12; spacing: 8
                RowLayout {
                    Note { Layout.fillWidth: true; text: backend.status; color: backend.hasError ? root.danger : root.ink }
                    Action { visible: backend.errorDetail.length > 0; text: "오류 상세"; onClicked: errorDialog.open() }
                }
                RowLayout {
                    spacing: 8
                    Text { text: "편집 " + parameterModel.dirtyCount + "개"; color: root.ink }
                    Text { text: "· ROM과 다름 " + parameterModel.unsavedCount + "개"; color: parameterModel.unsavedCount ? root.warning : root.muted }
                    Item { Layout.fillWidth: true }
                    Action { text: "편집 취소"; help: "PC에서 입력한 값만 취소합니다. 장치 RAM을 변경하지 않습니다."; enabled: parameterModel.dirtyCount > 0 && !backend.busy; onClicked: backend.resetEdits() }
                    Action { objectName: "applyButton"; text: "RAM에 적용"; primary: true; help: "모든 분류의 편집값을 RAM에 적용하고 재조회합니다. EEPROM에는 저장하지 않습니다."; enabled: backend.live && !backend.busy && parameterModel.dirtyCount > 0 && parameterModel.invalidCount === 0; onClicked: backend.applyRam() }
                    Action { objectName: "burnButton"; text: "EEPROM에 저장…"; help: "ROM과 RAM의 차이를 확인한 뒤 전체 RAM을 영구 저장합니다. 편집값을 먼저 RAM에 적용하세요."; enabled: backend.live && !backend.busy && parameterModel.dirtyCount === 0 && parameterModel.unsavedCount > 0; onClicked: { burnDialog.changes = backend.prepareBurn(); burnDialog.open() } }
                }
            }
        }
    }
}
