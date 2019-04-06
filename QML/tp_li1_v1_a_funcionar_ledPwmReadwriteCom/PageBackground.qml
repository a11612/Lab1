import QtQuick 2.0

Rectangle
{

    width: 900
    height:800
    color: "#100f0f"

    Text {
        id: text1
        x: 21
        y: 35
        width: 87
        height: 27
        text: qsTr("Text")
        font.pixelSize: 12
    }

    Rectangle {
        id: rectangle
        x: 673
        y: -38
        width: 347
        height: 174
        color: "#1b1f1b"
        visible: true
        clip: true
        rotation: 47
    }
}

/*##^## Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
 ##^##*/
