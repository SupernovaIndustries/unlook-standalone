import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

// Unlook 3D Scanner Main Window - Qt Design Studio Compatible
ApplicationWindow {
    id: unlookMainWindow
    title: "Unlook 3D Scanner v1.0"
    width: 1824
    height: 1026
    visible: true
    color: UnlookTheme.colors.background

    // Design System - Supernova-tech Theme
    QtObject {
        id: UnlookTheme
        
        readonly property QtObject colors: QtObject {
            readonly property color primary: "#00E5CC"
            readonly property color secondary: "#00B8A3"
            readonly property color accent: "#FF6B35"
            readonly property color background: "#000000"
            readonly property color surface: "#1A2B2A"
            readonly property color textPrimary: "#FFFFFF"
            readonly property color textSecondary: "#B3B3B3"
            readonly property color success: "#4CAF50"
            readonly property color warning: "#FF9800"
            readonly property color error: "#F44336"
            readonly property color info: "#2196F3"
        }
        
        readonly property QtObject typography: QtObject {
            readonly property string fontFamily: "Inter"
            readonly property int large: 24
            readonly property int heading: 18
            readonly property int body: 14
            readonly property int small: 12
        }
        
        readonly property QtObject spacing: QtObject {
            readonly property int xs: 4
            readonly property int sm: 8
            readonly property int md: 16
            readonly property int lg: 24
            readonly property int xl: 32
        }
        
        readonly property QtObject radius: QtObject {
            readonly property int sm: 4
            readonly property int md: 8
            readonly property int lg: 16
            readonly property int round: 32
        }
        
        readonly property QtObject buttonSizes: QtObject {
            readonly property size main: Qt.size(312, 86)
            readonly property size secondary: Qt.size(150, 60)
            readonly property size compact: Qt.size(120, 48)
        }
    }

    // Custom Touch Button Component
    component TouchButton: Rectangle {
        id: touchButton
        property alias text: buttonText.text
        property string buttonType: "primary"
        property bool enabled: true
        signal clicked()
        
        width: UnlookTheme.buttonSizes.main.width
        height: UnlookTheme.buttonSizes.main.height
        radius: UnlookTheme.radius.md
        
        color: {
            switch(buttonType) {
                case "primary": return UnlookTheme.colors.primary
                case "secondary": return UnlookTheme.colors.secondary  
                case "success": return UnlookTheme.colors.success
                case "warning": return UnlookTheme.colors.warning
                case "error": return UnlookTheme.colors.error
                default: return UnlookTheme.colors.primary
            }
        }
        
        opacity: enabled ? 1.0 : 0.6
        
        // Visual feedback
        scale: mouseArea.pressed ? 0.95 : 1.0
        Behavior on scale { NumberAnimation { duration: 100 } }
        
        Text {
            id: buttonText
            anchors.centerIn: parent
            font.family: UnlookTheme.typography.fontFamily
            font.pixelSize: UnlookTheme.typography.body
            font.weight: Font.Bold
            color: buttonType === "primary" || buttonType === "warning" ? 
                   UnlookTheme.colors.background : UnlookTheme.colors.textPrimary
        }
        
        MouseArea {
            id: mouseArea
            anchors.fill: parent
            enabled: touchButton.enabled
            onClicked: touchButton.clicked()
        }
    }

    // Custom Status Display Component  
    component StatusDisplay: Rectangle {
        id: statusDisplay
        property alias text: statusText.text
        property string statusType: "info"
        property bool pulsing: false
        
        height: 32
        radius: height / 2
        
        color: {
            switch(statusType) {
                case "success": return UnlookTheme.colors.success
                case "warning": return UnlookTheme.colors.warning
                case "error": return UnlookTheme.colors.error
                case "processing": return UnlookTheme.colors.accent
                default: return UnlookTheme.colors.info
            }
        }
        
        // Pulsing animation for processing states
        opacity: pulsing ? pulseAnimation.running ? 0.6 : 1.0 : 1.0
        
        SequentialAnimation {
            id: pulseAnimation
            running: statusDisplay.pulsing
            loops: Animation.Infinite
            NumberAnimation { target: statusDisplay; property: "opacity"; to: 0.6; duration: 500 }
            NumberAnimation { target: statusDisplay; property: "opacity"; to: 1.0; duration: 500 }
        }
        
        Text {
            id: statusText
            anchors.centerIn: parent
            font.family: UnlookTheme.typography.fontFamily
            font.pixelSize: UnlookTheme.typography.small
            color: statusType === "warning" ? UnlookTheme.colors.background : UnlookTheme.colors.textPrimary
        }
    }

    // Main Application Layout
    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // Title Bar
        Rectangle {
            id: titleBar
            Layout.fillWidth: true
            Layout.preferredHeight: 84
            color: UnlookTheme.colors.surface
            
            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: UnlookTheme.spacing.lg
                anchors.rightMargin: UnlookTheme.spacing.lg
                spacing: UnlookTheme.spacing.md
                
                Text {
                    id: titleText
                    text: "UNLOOK 3D SCANNER"
                    font.family: UnlookTheme.typography.fontFamily
                    font.pixelSize: UnlookTheme.typography.large
                    font.weight: Font.Bold
                    color: UnlookTheme.colors.primary
                }
                
                Text {
                    id: versionText
                    text: "v1.0 Professional"
                    font.family: UnlookTheme.typography.fontFamily
                    font.pixelSize: UnlookTheme.typography.small
                    color: UnlookTheme.colors.textSecondary
                }
                
                Item {
                    Layout.fillWidth: true
                }
                
                TouchButton {
                    id: backButton
                    text: "← BACK"
                    buttonType: "secondary"
                    width: UnlookTheme.buttonSizes.compact.width
                    height: UnlookTheme.buttonSizes.compact.height
                    visible: screenStack.currentIndex !== 0
                    onClicked: screenStack.currentIndex = 0
                }
                
                TouchButton {
                    id: fullscreenButton
                    text: "⛶ FULLSCREEN"
                    buttonType: "secondary"
                    width: UnlookTheme.buttonSizes.compact.width
                    height: UnlookTheme.buttonSizes.compact.height
                    onClicked: {
                        if (unlookMainWindow.visibility === Window.FullScreen) {
                            unlookMainWindow.showNormal()
                        } else {
                            unlookMainWindow.showFullScreen()
                        }
                    }
                }
            }
        }

        // Main Content Area with Stack View
        StackLayout {
            id: screenStack
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: 0

            // Main Menu Screen
            Rectangle {
                id: mainMenuScreen
                color: UnlookTheme.colors.background
                
                GridLayout {
                    anchors.centerIn: parent
                    columns: 2
                    rows: 2
                    columnSpacing: UnlookTheme.spacing.lg
                    rowSpacing: UnlookTheme.spacing.lg
                    
                    TouchButton {
                        id: cameraPreviewButton
                        text: "📹 CAMERA PREVIEW"
                        buttonType: "primary"
                        onClicked: screenStack.currentIndex = 1
                    }
                    
                    TouchButton {
                        id: depthTestButton
                        text: "🔍 DEPTH TEST"
                        buttonType: "primary"
                        onClicked: screenStack.currentIndex = 2
                    }
                    
                    TouchButton {
                        id: optionsButton
                        text: "⚙️ OPTIONS"
                        buttonType: "secondary"
                        onClicked: screenStack.currentIndex = 3
                    }
                    
                    TouchButton {
                        id: exitButton
                        text: "🚪 EXIT"
                        buttonType: "warning"
                        onClicked: Qt.quit()
                    }
                }
            }

            // Camera Preview Screen
            Rectangle {
                id: cameraPreviewScreen
                color: UnlookTheme.colors.background
                
                RowLayout {
                    anchors.fill: parent
                    anchors.margins: UnlookTheme.spacing.md
                    spacing: UnlookTheme.spacing.md
                    
                    // Camera Feed Area
                    ColumnLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: UnlookTheme.spacing.md
                        
                        // Camera Grid
                        GridLayout {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            columns: 2
                            columnSpacing: UnlookTheme.spacing.md
                            
                            Rectangle {
                                id: leftCameraPreview
                                Layout.preferredWidth: 400
                                Layout.preferredHeight: 300
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                color: UnlookTheme.colors.background
                                border.color: UnlookTheme.colors.primary
                                border.width: 2
                                radius: UnlookTheme.radius.md
                                
                                Text {
                                    anchors.centerIn: parent
                                    text: "LEFT Camera Preview"
                                    font.family: UnlookTheme.typography.fontFamily
                                    font.pixelSize: UnlookTheme.typography.body
                                    color: UnlookTheme.colors.textPrimary
                                }
                            }
                            
                            Rectangle {
                                id: rightCameraPreview
                                Layout.preferredWidth: 400
                                Layout.preferredHeight: 300
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                color: UnlookTheme.colors.background
                                border.color: UnlookTheme.colors.secondary
                                border.width: 2
                                radius: UnlookTheme.radius.md
                                
                                Text {
                                    anchors.centerIn: parent
                                    text: "RIGHT Camera Preview"
                                    font.family: UnlookTheme.typography.fontFamily
                                    font.pixelSize: UnlookTheme.typography.body
                                    color: UnlookTheme.colors.textPrimary
                                }
                            }
                        }
                        
                        // Status Row
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: UnlookTheme.spacing.md
                            
                            StatusDisplay {
                                id: fpsStatus
                                text: "31 FPS"
                                statusType: "info"
                            }
                            
                            StatusDisplay {
                                id: syncStatus
                                text: "Hardware Sync: OK"
                                statusType: "success"
                            }
                            
                            Item { Layout.fillWidth: true }
                        }
                    }
                    
                    // Control Panel
                    Rectangle {
                        Layout.preferredWidth: 350
                        Layout.fillHeight: true
                        color: UnlookTheme.colors.surface
                        radius: UnlookTheme.radius.md
                        
                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: UnlookTheme.spacing.lg
                            spacing: UnlookTheme.spacing.md
                            
                            Text {
                                text: "Camera Controls"
                                font.family: UnlookTheme.typography.fontFamily
                                font.pixelSize: UnlookTheme.typography.heading
                                font.weight: Font.Bold
                                color: UnlookTheme.colors.primary
                            }
                            
                            TouchButton {
                                Layout.fillWidth: true
                                text: "▶ START CAPTURE"
                                buttonType: "success"
                                height: 60
                            }
                            
                            TouchButton {
                                Layout.fillWidth: true
                                text: "⏹ STOP CAPTURE"
                                buttonType: "warning"
                                height: 60
                            }
                            
                            Item { Layout.fillHeight: true }
                        }
                    }
                }
            }

            // Depth Test Screen
            Rectangle {
                id: depthTestScreen
                color: UnlookTheme.colors.background
                
                RowLayout {
                    anchors.fill: parent
                    anchors.margins: UnlookTheme.spacing.md
                    spacing: UnlookTheme.spacing.md
                    
                    // Visualization Area
                    ColumnLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: UnlookTheme.spacing.md
                        
                        Rectangle {
                            id: depthMapDisplay
                            Layout.preferredWidth: 500
                            Layout.preferredHeight: 375
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            color: UnlookTheme.colors.background
                            border.color: UnlookTheme.colors.primary
                            border.width: 2
                            radius: UnlookTheme.radius.md
                            
                            Text {
                                anchors.centerIn: parent
                                text: "No depth map available"
                                font.family: UnlookTheme.typography.fontFamily
                                font.pixelSize: UnlookTheme.typography.body
                                color: UnlookTheme.colors.textPrimary
                            }
                        }
                        
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 60
                            color: UnlookTheme.colors.surface
                            radius: UnlookTheme.radius.sm
                            
                            Text {
                                anchors.centerIn: parent
                                text: "Quality metrics will appear here"
                                font.family: UnlookTheme.typography.fontFamily
                                font.pixelSize: UnlookTheme.typography.body
                                color: UnlookTheme.colors.textSecondary
                                wrapMode: Text.WordWrap
                            }
                        }
                    }
                    
                    // Control Panel
                    Rectangle {
                        Layout.preferredWidth: 350
                        Layout.fillHeight: true
                        color: UnlookTheme.colors.surface
                        radius: UnlookTheme.radius.md
                        
                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: UnlookTheme.spacing.lg
                            spacing: UnlookTheme.spacing.md
                            
                            Text {
                                text: "Depth Processing"
                                font.family: UnlookTheme.typography.fontFamily
                                font.pixelSize: UnlookTheme.typography.heading
                                font.weight: Font.Bold
                                color: UnlookTheme.colors.primary
                            }
                            
                            TouchButton {
                                Layout.fillWidth: true
                                text: "📸 CAPTURE STEREO FRAME"
                                buttonType: "primary"
                                height: 60
                            }
                            
                            TouchButton {
                                Layout.fillWidth: true
                                text: "💾 EXPORT PLY"
                                buttonType: "success"
                                height: 60
                            }
                            
                            // Preset Buttons Grid
                            GridLayout {
                                Layout.fillWidth: true
                                columns: 2
                                columnSpacing: UnlookTheme.spacing.sm
                                rowSpacing: UnlookTheme.spacing.sm
                                
                                TouchButton {
                                    text: "Fast"
                                    buttonType: "warning"
                                    width: UnlookTheme.buttonSizes.secondary.width
                                    height: UnlookTheme.buttonSizes.secondary.height
                                }
                                
                                TouchButton {
                                    text: "Balanced"
                                    buttonType: "primary"
                                    width: UnlookTheme.buttonSizes.secondary.width
                                    height: UnlookTheme.buttonSizes.secondary.height
                                }
                                
                                TouchButton {
                                    Layout.columnSpan: 2
                                    Layout.fillWidth: true
                                    text: "Quality"
                                    buttonType: "success"
                                    height: UnlookTheme.buttonSizes.secondary.height
                                }
                            }
                            
                            Item { Layout.fillHeight: true }
                        }
                    }
                }
            }

            // Options Screen
            Rectangle {
                id: optionsScreen
                color: UnlookTheme.colors.background
                
                ScrollView {
                    anchors.fill: parent
                    anchors.margins: UnlookTheme.spacing.md
                    
                    ColumnLayout {
                        width: optionsScreen.width - 2 * UnlookTheme.spacing.md
                        spacing: UnlookTheme.spacing.lg
                        
                        // System Status Panel
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 150
                            color: UnlookTheme.colors.surface
                            radius: UnlookTheme.radius.md
                            
                            ColumnLayout {
                                anchors.fill: parent
                                anchors.margins: UnlookTheme.spacing.lg
                                spacing: UnlookTheme.spacing.md
                                
                                Text {
                                    text: "System Status"
                                    font.family: UnlookTheme.typography.fontFamily
                                    font.pixelSize: UnlookTheme.typography.heading
                                    font.weight: Font.Bold
                                    color: UnlookTheme.colors.primary
                                }
                                
                                StatusDisplay {
                                    Layout.fillWidth: true
                                    text: "System operational"
                                    statusType: "success"
                                }
                                
                                StatusDisplay {
                                    Layout.fillWidth: true
                                    text: "Hardware Sync ENABLED"
                                    statusType: "success"
                                }
                            }
                        }
                        
                        Item { Layout.fillHeight: true }
                    }
                }
            }
        }

        // Status Bar
        Rectangle {
            id: statusBar
            Layout.fillWidth: true
            Layout.preferredHeight: 54
            color: UnlookTheme.colors.surface
            
            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: UnlookTheme.spacing.lg
                anchors.rightMargin: UnlookTheme.spacing.lg
                spacing: UnlookTheme.spacing.md
                
                StatusDisplay {
                    text: "System: OK"
                    statusType: "success"
                }
                
                StatusDisplay {
                    text: "Cameras: Ready"
                    statusType: "info"
                }
                
                Item { Layout.fillWidth: true }
                
                Text {
                    text: Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
                    font.family: UnlookTheme.typography.fontFamily
                    font.pixelSize: UnlookTheme.typography.small
                    color: UnlookTheme.colors.textSecondary
                }
            }
        }
    }
}