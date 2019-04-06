/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/
import QtQuick 2.10
import QtQuick.Window 2.10
import QtQuick.Controls 2.3
import QtQuick 2.2
import QtQuick.Controls 1.1
import QtQuick.Extras 1.4


import QtGraphicalEffects 1.0
import QtQuick.Controls 1.4
import QtQuick.Controls.Styles 1.4
import QtQuick.Dialogs 1.0

import QtQuick.Layouts 1.0
import QtQuick.Window 2.1


ControlView {

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

        }*/

    }
}
