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
import QtQuick.Controls 1.6


ControlView {




        Button {
            id: buttonOn
            height: 100
            width: 100
            text: "ON"
            onClicked: BackEnd.onPinOn()
        }

        Button {

            height: 100
            width: 100
            text: "OFF"

            anchors.top: buttonOn.bottom
            anchors.topMargin: 10

            onClicked:  BackEnd.onPinOff()

        }



        Rectangle{
            height: 200
            width: 400
            color: "white"
            anchors.left:buttonOn.right

            TextEdit{

                objectName: "textBox1"

                text: "valores"

                anchors.fill: parent
            }
        }



property alias value: valueSlider.value

            //volume da playlist




                CustomizerSlider {
                    x:140
                    y:250

                    id: valueSlider
                    minimumValue: 0
                    value:0
                    maximumValue: 1024

                    onValueChanged: BackEnd.valueSlider(valueSlider.value)
                }


//comunication write

                Text {
                    id: tempslideValue
                    text: tempValueSlider.value
                    anchors.right: tempValueSlider.left
                    anchors.bottom: tempValueSlider.bottom
                }

                CustomizerSlider{
                    x:149
                    y:300

                    width: 500

                    id:tempValueSlider
                    minimumValue: 16
                    maximumValue: 30
                    value:18
                    stepSize: 1

                    onPressedChanged:{
                    if(! pressed)
                        BackEnd.tempChange(tempValueSlider.value)

                    }
                }
                








}
