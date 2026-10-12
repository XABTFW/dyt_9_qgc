/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts

import QtLocation
import QtPositioning
import QtQuick.Window
import QtQml.Models

import QGroundControl
import QGroundControl.Controls
import QGroundControl.Controllers
import QGroundControl.DytSerialBridge
import QGroundControl.Controls
import QGroundControl.FactSystem
import QGroundControl.FlightDisplay
import QGroundControl.FlightMap
import QGroundControl.Palette
import QGroundControl.ScreenTools
import QGroundControl.Vehicle

// This is the ui overlay layer for the widgets/tools for Fly View
Item {
    id: _root

    property var    parentToolInsets
    property var    totalToolInsets:        _totalToolInsets
    property var    mapControl
    property bool   isViewer3DOpen:         false

    property var    _activeVehicle:         QGroundControl.multiVehicleManager.activeVehicle
    property var    _sdm50Group:            _activeVehicle ? _activeVehicle.distanceSensors : null
    property var    _sdm50DistanceFact:     _sdm50Group ? _sdm50Group.rotationNone : null
    property real   _sdm50DistanceM:        _sdm50DistanceFact ? Number(_sdm50DistanceFact.rawValue) : NaN
    property real   _sdm50MaxDistanceM:     _sdm50Group ? Number(_sdm50Group.maxDistance.rawValue) : NaN
    property bool   _sdm50Available:        _sdm50Group && _sdm50Group.telemetryAvailable
    property bool   _sdm50Valid:            _sdm50Available && isFinite(_sdm50DistanceM) &&
                                             isFinite(_sdm50MaxDistanceM) && _sdm50DistanceM >= 0.05 &&
                                             _sdm50DistanceM < _sdm50MaxDistanceM
    property real   _sdm50ClosingSpeedMps:  _activeVehicle ? Number(_activeVehicle.sdm50ClosingSpeedMps) : NaN
    property bool   _sdm50SpeedValid:       isFinite(_sdm50ClosingSpeedMps)
    property var    _planMasterController:  globals.planMasterControllerFlyView
    property var    _missionController:     _planMasterController.missionController
    property var    _geoFenceController:    _planMasterController.geoFenceController
    property var    _rallyPointController:  _planMasterController.rallyPointController
    property var    _guidedController:      globals.guidedControllerFlyView
    property real   _margins:               ScreenTools.defaultFontPixelWidth / 2
    property real   _toolsMargin:           ScreenTools.defaultFontPixelWidth * 0.75
    property rect   _centerViewport:        Qt.rect(0, 0, width, height)
    property real   _rightPanelWidth:       ScreenTools.defaultFontPixelWidth * 30
    property alias  _gripperMenu:           gripperOptions
    property real   _layoutMargin:          ScreenTools.defaultFontPixelWidth * 0.75
    property bool   _layoutSpacing:         ScreenTools.defaultFontPixelWidth
    property bool   _showSingleVehicleUI:   true
    property int    _pendingDytPhase:       0
    property bool   _dytDetailsExpanded:    false

    property bool utmspActTrigger

    function enableSdm50LiveUpdates() {
        if (_sdm50Group) {
            _sdm50Group.setLiveUpdates(true)
        }
    }

    on_ActiveVehicleChanged: enableSdm50LiveUpdates()
    Component.onCompleted: {
        enableSdm50LiveUpdates()
        DytSerialBridge.refreshPorts()
    }

    QGCToolInsets {
        id:                     _totalToolInsets
        leftEdgeTopInset:       toolStrip.leftEdgeTopInset
        leftEdgeCenterInset:    toolStrip.leftEdgeCenterInset
        leftEdgeBottomInset:    virtualJoystickMultiTouch.visible ? virtualJoystickMultiTouch.leftEdgeBottomInset : parentToolInsets.leftEdgeBottomInset
        rightEdgeTopInset:      topRightPanel.rightEdgeTopInset
        rightEdgeCenterInset:   topRightPanel.rightEdgeCenterInset
        rightEdgeBottomInset:   bottomRightRowLayout.rightEdgeBottomInset
        topEdgeLeftInset:       toolStrip.topEdgeLeftInset
        topEdgeCenterInset:     mapScale.topEdgeCenterInset
        topEdgeRightInset:      topRightPanel.topEdgeRightInset
        bottomEdgeLeftInset:    virtualJoystickMultiTouch.visible ? virtualJoystickMultiTouch.bottomEdgeLeftInset : parentToolInsets.bottomEdgeLeftInset
        bottomEdgeCenterInset:  bottomRightRowLayout.bottomEdgeCenterInset
        bottomEdgeRightInset:   virtualJoystickMultiTouch.visible ? virtualJoystickMultiTouch.bottomEdgeRightInset : bottomRightRowLayout.bottomEdgeRightInset
    }

    FlyViewTopRightPanel {
        id:                     topRightPanel
        anchors.top:            parent.top
        anchors.right:          parent.right
        anchors.topMargin:      _layoutMargin
        anchors.rightMargin:    _layoutMargin
        maximumHeight:          parent.height - (bottomRightRowLayout.height + _margins * 5)

        property real topEdgeRightInset:    height + _layoutMargin
        property real rightEdgeTopInset:    width + _layoutMargin
        property real rightEdgeCenterInset: rightEdgeTopInset
    }

    FlyViewTopRightColumnLayout {
        id:                 topRightColumnLayout
        anchors.margins:    _layoutMargin
        anchors.top:        parent.top
        anchors.bottom:     bottomRightRowLayout.top
        anchors.right:      parent.right
        spacing:            _layoutSpacing
        visible:           !topRightPanel.visible

        property real topEdgeRightInset:    childrenRect.height + _layoutMargin
        property real rightEdgeTopInset:    width + _layoutMargin
        property real rightEdgeCenterInset: rightEdgeTopInset
    }

    FlyViewBottomRightRowLayout {
        id:                 bottomRightRowLayout
        anchors.margins:    _layoutMargin
        anchors.bottom:     parent.bottom
        anchors.right:      parent.right
        spacing:            _layoutSpacing

        property real bottomEdgeRightInset:     height + _layoutMargin
        property real bottomEdgeCenterInset:    bottomEdgeRightInset
        property real rightEdgeBottomInset:     width + _layoutMargin
    }

    FlyViewMissionCompleteDialog {
        missionController:      _missionController
        geoFenceController:     _geoFenceController
        rallyPointController:   _rallyPointController
    }

    GuidedActionConfirm {
        anchors.margins:            _toolsMargin
        anchors.top:                parent.top
        anchors.horizontalCenter:   parent.horizontalCenter
        z:                          QGroundControl.zOrderTopMost
        guidedController:           _guidedController
        guidedValueSlider:          _guidedValueSlider
        utmspSliderTrigger:         utmspActTrigger
    }

    //-- Virtual Joystick
    Loader {
        id:                         virtualJoystickMultiTouch
        z:                          QGroundControl.zOrderTopMost + 1
        anchors.right:              parent.right
        anchors.rightMargin:        anchors.leftMargin
        height:                     Math.min(parent.height * 0.25, ScreenTools.defaultFontPixelWidth * 16)
        visible:                    _virtualJoystickEnabled && !QGroundControl.videoManager.fullScreen && !(_activeVehicle ? _activeVehicle.usingHighLatencyLink : false)
        anchors.bottom:             parent.bottom
        anchors.bottomMargin:       bottomLoaderMargin
        anchors.left:               parent.left   
        anchors.leftMargin:         ( y > toolStrip.y + toolStrip.height ? toolStrip.width / 2 : toolStrip.width * 1.05 + toolStrip.x) 
        source:                     "qrc:/qml/VirtualJoystick.qml"
        active:                     _virtualJoystickEnabled && !(_activeVehicle ? _activeVehicle.usingHighLatencyLink : false)

        property real bottomEdgeLeftInset:     parent.height-y
        property bool autoCenterThrottle:      QGroundControl.settingsManager.appSettings.virtualJoystickAutoCenterThrottle.rawValue
        property bool leftHandedMode:          QGroundControl.settingsManager.appSettings.virtualJoystickLeftHandedMode.rawValue
        property bool _virtualJoystickEnabled: QGroundControl.settingsManager.appSettings.virtualJoystick.rawValue
        property real bottomEdgeRightInset:    parent.height-y
        property var  _pipViewMargin:          _pipView.visible ? parentToolInsets.bottomEdgeLeftInset + ScreenTools.defaultFontPixelHeight * 2 : 
                                               bottomRightRowLayout.height + ScreenTools.defaultFontPixelHeight * 1.5

        property var  bottomLoaderMargin:      _pipViewMargin >= parent.height / 2 ? parent.height / 2 : _pipViewMargin

        // Width is difficult to access directly hence this hack which may not work in all circumstances
        property real leftEdgeBottomInset:  visible ? bottomEdgeLeftInset + width/18 - ScreenTools.defaultFontPixelHeight*2 : 0
        property real rightEdgeBottomInset: visible ? bottomEdgeRightInset + width/18 - ScreenTools.defaultFontPixelHeight*2 : 0
        property real rootWidth:            _root.width
        property var  itemX:                virtualJoystickMultiTouch.x   // real X on screen

        onRootWidthChanged: virtualJoystickMultiTouch.status == Loader.Ready && visible ? virtualJoystickMultiTouch.item.uiTotalWidth = rootWidth : undefined
        onItemXChanged:     virtualJoystickMultiTouch.status == Loader.Ready && visible ? virtualJoystickMultiTouch.item.uiRealX = itemX : undefined

        //Loader status logic
        onLoaded: {
            if (virtualJoystickMultiTouch.visible) {
                virtualJoystickMultiTouch.item.calibration = true 
                virtualJoystickMultiTouch.item.uiTotalWidth = rootWidth
                virtualJoystickMultiTouch.item.uiRealX = itemX
            } else {
                virtualJoystickMultiTouch.item.calibration = false
            }
        }
    }

    FlyViewToolStrip {
        id:                     toolStrip
        anchors.leftMargin:     _toolsMargin + parentToolInsets.leftEdgeCenterInset
        anchors.topMargin:      _toolsMargin + parentToolInsets.topEdgeLeftInset
        anchors.left:           parent.left
        anchors.top:            parent.top
        z:                      QGroundControl.zOrderWidgets
        maxHeight:              parent.height - y - parentToolInsets.bottomEdgeLeftInset - _toolsMargin
        visible:                !QGroundControl.videoManager.fullScreen

        onDisplayPreFlightChecklist: {
            if (!preFlightChecklistLoader.active) {
                preFlightChecklistLoader.active = true
            }
            preFlightChecklistLoader.item.open()
        }

        property real topEdgeLeftInset:     visible ? y + height : 0
        property real leftEdgeTopInset:     visible ? x + width : 0
        property real leftEdgeCenterInset:  leftEdgeTopInset
    }

    GripperMenu {
        id: gripperOptions
    }

    VehicleWarnings {
        anchors.centerIn:   parent
        z:                  QGroundControl.zOrderTopMost
    }

    MapScale {
        id:                 mapScale
        anchors.margins:    _toolsMargin
        anchors.left:       toolStrip.right
        anchors.top:        parent.top
        mapControl:         _mapControl
        buttonsOnLeft:      true
        visible:            !ScreenTools.isTinyScreen && QGroundControl.corePlugin.options.flyView.showMapScale && !isViewer3DOpen && mapControl.pipState.state === mapControl.pipState.fullState

        property real topEdgeCenterInset: visible ? y + height : 0
    }

    Rectangle {
        id:                     sdm50Panel
        anchors.left:           toolStrip.right
        anchors.top:            mapScale.bottom
        anchors.leftMargin:     _toolsMargin
        anchors.topMargin:      _toolsMargin
        width:                  ScreenTools.defaultFontPixelWidth * 24
        height:                 sdm50Column.implicitHeight + _toolsMargin * 2
        radius:                 ScreenTools.defaultFontPixelWidth * 0.5
        color:                  sdm50Palette.window
        border.width:           2
        border.color:           _sdm50Valid ? "#36c96f" : sdm50Palette.text
        opacity:                0.94
        visible:                _activeVehicle
        z:                      QGroundControl.zOrderWidgets + 2

        QGCPalette { id: sdm50Palette; colorGroupEnabled: true }

        ColumnLayout {
            id:                 sdm50Column
            anchors.fill:       parent
            anchors.margins:    _toolsMargin
            spacing:            _margins / 2

            QGCLabel {
                text:           qsTr("SDM50 Laser Range")
                font.bold:      true
                Layout.fillWidth: true
            }

            QGCLabel {
                text:           _sdm50Valid ? _sdm50DistanceM.toFixed(2) + qsTr(" m") : qsTr("-- m")
                font.bold:      true
                font.pointSize: ScreenTools.largeFontPointSize
                Layout.fillWidth: true
            }

            QGCLabel {
                text: !_sdm50Available ? qsTr("Status: waiting for data")
                                      : (_sdm50Valid ? qsTr("Status: valid") : qsTr("Status: no target"))
                Layout.fillWidth: true
            }

            QGCLabel {
                text: _sdm50SpeedValid
                      ? qsTr("Closing speed: %1 m/s").arg(_sdm50ClosingSpeedMps.toFixed(2))
                      : qsTr("Closing speed: -- m/s")
                Layout.fillWidth: true
            }
        }
    }

    Rectangle {
        id:                     dytPanel
        anchors.left:           toolStrip.right
        anchors.top:            sdm50Panel.bottom
        anchors.leftMargin:     _toolsMargin
        anchors.topMargin:      _toolsMargin
        width:                  Math.min(parent.width * 0.55, ScreenTools.defaultFontPixelWidth * 58)
        height:                 dytPanelColumn.implicitHeight + _toolsMargin * 2
        radius:                 ScreenTools.defaultFontPixelWidth * 0.5
        color:                  qgcPal.window
        border.color:           qgcPal.text
        opacity:                0.94
        visible:                _activeVehicle
        z:                      QGroundControl.zOrderWidgets + 2

        QGCPalette { id: qgcPal; colorGroupEnabled: true }

        ColumnLayout {
            id:                 dytPanelColumn
            anchors.margins:    _toolsMargin
            anchors.left:       parent.left
            anchors.right:      parent.right
            anchors.top:        parent.top
            spacing:            _margins

            RowLayout {
                Layout.fillWidth: true
                QGCLabel {
                    text: qsTr("DYT  %1  |  %2").arg(_activeVehicle ? _activeVehicle.dytVehicleTypeText : "")
                                                       .arg(_activeVehicle ? _activeVehicle.dytGuidancePhaseText : "")
                    font.bold: true
                    Layout.fillWidth: true
                }
                QGCButton {
                    text: _dytDetailsExpanded ? qsTr("Hide details") : qsTr("Details")
                    onClicked: _dytDetailsExpanded = !_dytDetailsExpanded
                }
            }

            RowLayout {
                Layout.fillWidth: true
                QGCButton {
                    text: qsTr("Enter midcourse")
                    enabled: _activeVehicle && _activeVehicle.armed
                    onClicked: {
                        _pendingDytPhase = 2
                        dytConfirmDialog.open()
                    }
                }
                QGCButton {
                    text: qsTr("Enter terminal")
                    enabled: _activeVehicle && _activeVehicle.armed
                    onClicked: {
                        _pendingDytPhase = 3
                        dytConfirmDialog.open()
                    }
                }
                QGCLabel {
                    Layout.fillWidth: true
                    wrapMode: Text.WordWrap
                    text: _activeVehicle ? _activeVehicle.dytCommandResultText : ""
                }
            }

            RowLayout {
                Layout.fillWidth: true

                QGCLabel { text: qsTr("TCS virtual port") }

                QGCComboBox {
                    id:                 dytSerialPortCombo
                    Layout.fillWidth:   true
                    model:              DytSerialBridge.availablePorts
                    enabled:            !DytSerialBridge.connected
                }

                QGCButton {
                    text:               qsTr("Refresh")
                    enabled:            !DytSerialBridge.connected
                    onClicked:          DytSerialBridge.refreshPorts()
                }

                QGCButton {
                    text:               DytSerialBridge.connected ? qsTr("Disconnect") : qsTr("Connect")
                    enabled:            DytSerialBridge.connected || dytSerialPortCombo.currentText.length > 0
                    onClicked: {
                        if (DytSerialBridge.connected) {
                            DytSerialBridge.closePort()
                        } else {
                            DytSerialBridge.openPort(dytSerialPortCombo.currentText,
                                                     parseInt(dytImageWidth.text),
                                                     parseInt(dytImageHeight.text))
                        }
                    }
                }
            }

            RowLayout {
                Layout.fillWidth: true

                QGCLabel { text: qsTr("Image width") }
                QGCTextField {
                    id:                 dytImageWidth
                    Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 8
                    text:               DytSerialBridge.imageWidth.toString()
                    numericValuesOnly:  true
                    enabled:            !DytSerialBridge.connected
                }

                QGCLabel { text: qsTr("height") }
                QGCTextField {
                    id:                 dytImageHeight
                    Layout.preferredWidth: ScreenTools.defaultFontPixelWidth * 8
                    text:               DytSerialBridge.imageHeight.toString()
                    numericValuesOnly:  true
                    enabled:            !DytSerialBridge.connected
                }

                QGCButton {
                    text:               qsTr("Resend last")
                    enabled:            DytSerialBridge.connected
                    onClicked:          DytSerialBridge.resendLastPoint()
                }
            }

            QGCLabel {
                Layout.fillWidth: true
                wrapMode:        Text.WordWrap
                text:            qsTr("Serial: %1 | Last TCS target: %2")
                                     .arg(DytSerialBridge.statusText)
                                     .arg(DytSerialBridge.lastPointText)
            }

            QGCLabel {
                Layout.fillWidth: true
                wrapMode:        Text.WordWrap
                text:            _activeVehicle ? qsTr("Aircraft ACK: %1").arg(_activeVehicle.dytTrackPointResultText) : ""
            }

            QGCLabel {
                Layout.fillWidth: true
                text: _activeVehicle ? qsTr("Target: %1  Range: %2 m  Net trigger: %3 (%4)")
                                           .arg(_activeVehicle.dytTargetValid ? qsTr("valid") : qsTr("invalid"))
                                           .arg(Number(_activeVehicle.dytRangeM).toFixed(2))
                                           .arg(_activeVehicle.dytNetTriggerSent ? qsTr("sent") : qsTr("not sent"))
                                           .arg(_activeVehicle.dytNetTriggerCount) : ""
            }

            ScrollView {
                Layout.fillWidth: true
                Layout.preferredHeight: Math.min(ScreenTools.defaultFontPixelHeight * 24, _root.height * 0.48)
                visible: _dytDetailsExpanded
                clip: true

                QGCLabel {
                    width: dytPanel.width - _toolsMargin * 4
                    wrapMode: Text.WrapAnywhere
                    text: _activeVehicle ? _activeVehicle.dytGuidanceDetails + "\n\n" +
                                           _activeVehicle.dytTargetDetails + "\n\n" +
                                           _activeVehicle.dytReplyDetails : ""
                }
            }
        }
    }

    MessageDialog {
        id: dytConfirmDialog
        title: qsTr("Confirm guidance switch")
        text: _pendingDytPhase === 2
              ? qsTr("Command the aircraft to enter midcourse guidance?")
              : qsTr("Command the aircraft to enter terminal guidance?")
        buttons: MessageDialog.Yes | MessageDialog.No
        onAccepted: {
            if (_activeVehicle) {
                _activeVehicle.sendDytGuidanceCommand(_pendingDytPhase)
            }
        }
    }

    Loader {
        id: preFlightChecklistLoader
        sourceComponent: preFlightChecklistPopup
        active: false
    }

    Component {
        id: preFlightChecklistPopup
        FlyViewPreFlightChecklistPopup {
        }
    }
}
