
import QtQuick 2.2
import QtGraphicalEffects 1.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Dialogs 1.0
import QtQuick.Extras 1.4
import QtQuick.Layouts 1.0
import QtQuick.Window 2.1

//import QtMultimedia 5.0

Window
{
    id: root
    objectName: "window"
    visible: true
    width: 480
    height: 800

    color: "#161616"
    title: "Home Control"

    function toPixels(percentage)
    {
        return percentage * Math.min(root.width, root.height);
    }

    property bool isScreenPortrait: height > width
    property color lightFontColor: "#222"
    property color darkFontColor: "#e7e7e7"
    readonly property color lightBackgroundColor: "#cccccc"
    readonly property color darkBackgroundColor: "#161616"
    property real customizerPropertySpacing: 10
    property real colorPickerRowSpacing: 8

    Text
    {
        id: textSingleton
    }

    //layer MOnitorização máquina lavar roupa
    property Component washingMachine: WashingMachineView {}

    //layer controlo Temperature
     property Component temperature: Temperature {}

   // property Component temperature: ControlView
    /*{
        darkBackground: false

        control: Column {
            id: dialColumn
            width: controlBounds.width
            height: controlBounds.height - spacing
            spacing: root.toPixels(0.05)

           RowLayout
            {
                id: volumeColumn
                width: parent.width
                height: (dialColumn.height - dialColumn.spacing) / 2
                spacing: height * 0.025

                Dial
                {
                    id: minTempDial
                    anchors.left: parent.left
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    stepSize: 1
                    maximumValue: 10

                    property bool animate: customizerItem.animate

                    Behavior on value
                    {
                        enabled: minTempDial.animate && !minTempDial.pressed
                        NumberAnimation
                        {
                            duration: 300
                            easing.type: Easing.OutSine
                        }
                    }
                }

                ControlLabel
                {
                    id: minTempText
                    text: "Min."
                    anchors.horizontalCenter: minTempDial.horizontalCenter
                }

                Dial
                {
                    id: maxTempDial
                    anchors.right: parent.right
                    Layout.fillWidth: true
                    Layout.fillHeight: true

                    stepSize: 1
                    maximumValue: 10

                    style: DialStyle
                    {
                        labelInset: outerRadius * 0
                    }
                }

                ControlLabel {
                    id: maxTempText
                    text: "Máx."
                    anchors.horizontalCenter: maxTempDial.horizontalCenter
                }
            }

        }

        customizer: Column
        {
            spacing: customizerPropertySpacing

            Item
            {
                id: stylePickerBottomSpacing
                width: parent.width
                height: stylePicker.height + textSingleton.implicitHeight


                StylePicker {
                    id: stylePickerModeTemp
                    currentIndex: 1

                    model: ListModel {
                        ListElement {
                            name: "Low"
                            //path: "CircularGaugeDefaultStyle.qml"
                            //dark: true
                        }
                        ListElement {
                            name: "Auto"
                            //path: "CircularGaugeDarkStyle.qml"
                            //dark: true
                        }
                        ListElement {
                            name: "High"
                            //path: "CircularGaugeLightStyle.qml"
                            //dark: false
                        }
                    }
                }
            }

            /*CustomizerLabel
            {
                text: "Minimum angle"

            }*/ /*

        }
  */
    //layer controlo luz
    property Component light: LightView {} //Go to Lightview.qml


    property Component windows: ControlView
    {
        id: windowsView
        control: Gauge
        {
            id: windows
            width: orientation === Qt.Vertical ? implicitWidth : windowsView.controlBounds.width
            height: orientation === Qt.Vertical ? windowsView.controlBounds.height : implicitHeight
            anchors.centerIn: parent

            minimumValue: 0
            value: customizerItem.value
            maximumValue: 100
            orientation:  Qt.Horizontal
                //talvez colocar aqui uma imagem mais ilustrativa??????

        }

        customizer: Column
        {
            spacing: customizerPropertySpacing

            property alias value: valueSlider.value

            CustomizerSlider {
                id: valueSlider
                minimumValue: 0
                value: 50
                maximumValue: 100
            }

        }
    }

    property Component toggleButton: ControlView
    {
        darkBackground: false
        Item
        {
            width: 400;
            height: 300;

            //volume da playlist
            Slider
            {
                x:70
                y:10
                id: volumeSlider
                height: 10
                property real volume: QtMultimedia.convertVolume(volumeSlider.value,
                                                                 QtMultimedia.LogarithmicVolumeScale,
                                                                QtMultimedia.LinearVolumeScale)

                 /*LinearGradient {
                        anchors.fill: parent
                        start: Qt.point(0, 0)
                        end: Qt.point(0, 300)
                        gradient: Gradient {
                            GradientStop {
                                 position: 0.000
                                 color: Qt.rgba(1, 0, 0, 1)
                              }
                              GradientStop {
                                 position: 0.167
                                 color: Qt.rgba(1, 1, 0, 1)
                              }
                              GradientStop {
                                 position: 0.333
                                 color: Qt.rgba(0, 1, 0, 1)
                              }
                              GradientStop {
                                 position: 0.500
                                 color: Qt.rgba(0, 1, 1, 1)
                              }
                              GradientStop {
                                 position: 0.667
                                 color: Qt.rgba(0, 0, 1, 1)
                              }
                              GradientStop {
                                 position: 0.833
                                 color: Qt.rgba(1, 0, 1, 1)
                              }
                              GradientStop {
                                 position: 1.000
                                 color: Qt.rgba(1, 0, 0, 1)
                              }
                        }

                    }

            }

            Audio
            {

                id: player;
                volume:volumeSlider.volume

               /* playlist: Playlist
                {

                    id: playlist
                    //volume:volumeSlider.volume
                    PlaylistItem { source: "music.ogg";}
                    PlaylistItem { source: "music1.ogg"; }
                   // PlaylistItem { source: "music2.ogg"; }
                }*/
            }

            ListView
            {
                model: playlist;
                delegate: Text {
                    font.pixelSize: 16;
                    text: source;
                }
            }

            //colocar uma linha ON--PLAY-STOP-NEXT

            Button
            {
                x:10
                y:30
                text:"ON"
                onClicked:  player.play();
            }

            Button
            {
                x:10
                y:60
                text:"next"
                onClicked:  //player.pause();
                            playlist.next();

            }

           }
    }

    property Component pieMenu: PieMenuControlView {}

    property Component camera: ControlView
    {
        id: cameraView
        darkBackground: false

        Timer
        {
            id: recordingFlashTimer
            running: true
            repeat: true
            interval: 1000
        }

        ColumnLayout
        {
            id: indicatorLayout
            width: cameraView.controlBounds.width * 0.25
            height: cameraView.controlBounds.height * 0.75
            anchors.centerIn: parent

            Repeater
            {
                model: ListModel
                {
                    id: indicatorModel
                    ListElement
                    {
                        name: "Power"
                        indicatorColor: "#35e02f"
                    }
                    ListElement
                    {

                        name: "Recording"
                        indicatorColor: "red"
                    }
                }

                ColumnLayout
                {
                    Layout.preferredWidth: indicatorLayout.width
                    spacing: 0

                    StatusIndicator {
                        id: indicator
                        color: indicatorColor
                        Layout.preferredWidth: cameraView.controlBounds.width * 0.07
                        Layout.preferredHeight: Layout.preferredWidth
                        Layout.alignment: Qt.AlignHCenter
                        on: true

                        Connections {
                            target: recordingFlashTimer
                            onTriggered: if (name == "Recording") indicator.active = !indicator.active
                        }
                    }
                    ControlLabel
                    {
                        id: indicatorLabel
                        text: name
                        Layout.alignment: Qt.AlignHCenter
                        Layout.maximumWidth: parent.width
                        horizontalAlignment: Text.AlignHCenter
                    }
                }
            }
        }


    }

    property Component tumbler: ControlView {
        id: tumblerView
        darkBackground: false

        Tumbler {
            id: tumbler
            anchors.centerIn: parent

            // TODO: Use FontMetrics with 5.4
            Label {
                id: characterMetrics
                font.bold: true
                font.pixelSize: textSingleton.font.pixelSize * 1.25
                font.family: openSans.name
                visible: false
                text: "M"
            }

            readonly property real delegateTextMargins: characterMetrics.width * 1.5
            readonly property var days: [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]

            TumblerColumn {
                id: tumblerDayColumn

                function updateModel() {
                    var previousIndex = tumblerDayColumn.currentIndex;
                    var newDays = tumbler.days[monthColumn.currentIndex];

                    if (!model) {
                        var array = [];
                        for (var i = 0; i < newDays; ++i) {
                            array.push(i + 1);
                        }
                        model = array;
                    } else {
                        // If we've already got days in the model, just add or remove
                        // the minimum amount necessary to make spinning the month column fast.
                        var difference = model.length - newDays;
                        if (model.length > newDays) {
                            model.splice(model.length - 1, difference);
                        } else {
                            var lastDay = model[model.length - 1];
                            for (i = lastDay; i < lastDay + difference; ++i) {
                                model.push(i + 1);
                            }
                        }
                    }

                    tumbler.setCurrentIndexAt(0, Math.min(newDays - 1, previousIndex));
                }
            }
            TumblerColumn {
                id: monthColumn
                width: characterMetrics.width * 3 + tumbler.delegateTextMargins
                model: ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"]
                onCurrentIndexChanged: tumblerDayColumn.updateModel()
            }
            TumblerColumn {
                width: characterMetrics.width * 4 + tumbler.delegateTextMargins
                model: ListModel {
                    Component.onCompleted: {
                        for (var i = 2000; i < 2100; ++i) {
                            append({value: i.toString()});
                        }
                    }
                }
            }
        }
    }


    FontLoader {
        id: openSans
        source: "qrc:/fonts/OpenSans-Regular.ttf"
     }

    property var componentMap: {           //this is a map for corresponding names
        "WashingMachine": washingMachine,
        "Light": light,
        "Temperature": temperature,
        "Windows": windows,
        "Camera": camera,
        "Settings": pieMenu,
        "Sound": toggleButton,
        "Power": tumbler
    }

    StackView                              //This creat a new element in toobar


    {
        id: stackView
        anchors.fill: parent

        initialItem: ListView {
            model: ListModel
            {
                ListElement {
                    title: "WashingMachine"

                }
                ListElement {
                    title: "Light"

                }
                ListElement {
                    title: "Temperature"
                }
                ListElement {
                    title: "Windows"
                }
                ListElement {
                    title: "Camera"
                }
                ListElement {
                    title: "Sound"
                }
                ListElement {
                    title: "Power"
                }
                ListElement {
                    title: "Settings"



                }
            }

            //change layer
            delegate: Button
            {
                width: stackView.width
                height: root.height * 0.125   //the size of toolbar icons
                text: title
                iconSource:title+".png"   // do corresponding in the layer name and a picture

                style: BlackButtonStyle
                {
                    fontColor: root.darkFontColor
                    rightAlignedIconSource: "icon-go.png"

                }

                onClicked:
                {
                    if (stackView.depth == 1) {
                        // Only push the control view if we haven't already pushed it...
                        stackView.push({item: componentMap[title]});

                        stackView.currentItem.forceActiveFocus();

                        BackEnd.onPageChange(title, stackView.depth);
                    }
                }
            }
        }
    }
}

