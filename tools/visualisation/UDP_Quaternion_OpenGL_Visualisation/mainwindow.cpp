/****************************************************************************
**
** BSD License
**
** Copyright (C) 2023 Fritz Webering <fritz.webering@ims.uni-hannover.de>
**
** Redistribution and use in source and binary forms, with or without modification,
** are permitted provided that the following conditions are met:
** 
** 1. Redistributions of source code must retain the above copyright notice, this
**    list of conditions and the following disclaimer.
** 
** 2. Redistributions in binary form must reproduce the above copyright notice,
**    this list of conditions and the following disclaimer in the documentation
**    and/or other materials provided with the distribution.
** 
** 3. Neither the name of the copyright holder nor the names of its contributors
**    may be used to endorse or promote products derived from this software without
**    specific prior written permission.
** 
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND
** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
** ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
** ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
****************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "glwidget.h"
#include "mesh.h"
#include <QString>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QDebug>

#define UDP_PORT 1024

Mesh createImuMesh()
{
    QVector3D blue(0.3, 0.3, 1.0);
    QVector3D green(0.3, 1.0, 0.3);
    QVector3D red(1.0, 0.3, 0.3);
    QVector3D yellow(1.0, 1.0, 0);
    QVector3D darker_yellow(0.8, 0.8, 0);

    Mesh imu;
    imu.setDrawMode(GL_TRIANGLES);

    // Rectangular part of the case, top then bottom
    imu.addFaceXY(0.06f, -0.09f, 0.06f, 0.12f, -0.06f, 0.12f, -0.06f, -0.09f, 0.025f, blue);
    imu.addFaceXY(0.06f, -0.09f, 0.06f, 0.12f, -0.06f, 0.12f, -0.06f, -0.09f, -0.025f, yellow);
    // Rounded part of the case (trapeze-shaped)
    imu.addFaceXY(0.06f, -0.09f, 0.03f, -0.12f, -0.03f, -0.12f, -0.06f, -0.09f, 0.025f, blue);
    imu.addFaceXY(0.06f, -0.09f, 0.03f, -0.12f, -0.03f, -0.12f, -0.06f, -0.09f, -0.025f, yellow);

    // Side walls of rectangular part of the case
    imu.extrudeLineXY(-0.06f, 0.12f, 0.06f, 0.12f, 0.025f, -0.025f, green);
    imu.extrudeLineXY(-0.06f, 0.12f, -0.06f, -0.09f, 0.025f, -0.025f, darker_yellow);
    imu.extrudeLineXY(0.06f, -0.09f, 0.06f, 0.12f, 0.025f, -0.025f, red);
    // Side walls of trapezoid part of the case
    imu.extrudeLineXY(0.03f, -0.12f, -0.03f, -0.12f, 0.025f, -0.025f, darker_yellow);
    imu.extrudeLineXY(0.06f, -0.09f, 0.03f, -0.12f, 0.025f, -0.025f, darker_yellow);
    imu.extrudeLineXY(-0.03f, -0.12f, -0.06f, -0.09f, 0.025f, -0.025f, darker_yellow);

    return imu;
}

Mesh createAxesMesh()
{
    QVector3D blue(0.3, 0.3, 1.0);
    QVector3D green(0.3, 1.0, 0.3);
    QVector3D red(1.0, 0.3, 0.3);

    Mesh axes;
    axes.setDrawMode(GL_LINES);

    // Axis lines: X = red, Y = green, Z = blue
    axes.addVertex(QVector3D(0.0f, 0.0f, 0.0f), red);
    axes.addVertex(QVector3D(0.3f, 0.0f, 0.0f), red);
    axes.addVertex(QVector3D(0.0f, 0.0f, 0.0f), red);
    axes.addVertex(QVector3D(0.0f, 0.0f, 0.0f), green);
    axes.addVertex(QVector3D(0.0f, 0.3f, 0.0f), green);
    axes.addVertex(QVector3D(0.0f, 0.0f, 0.0f), green);
    axes.addVertex(QVector3D(0.0f, 0.0f, 0.0f), blue);
    axes.addVertex(QVector3D(0.0f, 0.0f, 0.3f), blue);
    axes.addVertex(QVector3D(0.0f, 0.0f, 0.0f), blue);

    return axes;
}



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    glWidget = new GLWidget;
    glWidget->addMesh(createImuMesh());
    glWidget->addMesh(createAxesMesh());

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(glWidget);

    // Add Interfaces
    viewTopButton = new QPushButton;
    viewTopButton->setText("Top View");
    connect(viewTopButton, SIGNAL(clicked(bool)), glWidget, SLOT(setViewFromTop()));

    viewFrontButton = new QPushButton;
    viewFrontButton->setText("Front View");
    connect(viewFrontButton, SIGNAL(clicked(bool)), glWidget, SLOT(setViewFromFront()));

    QHBoxLayout *interfaceLayout = new QHBoxLayout;

    interfaceLayout->addWidget(viewTopButton);
    interfaceLayout->addWidget(viewFrontButton);

    mainLayout->addLayout(interfaceLayout);

    socket = new QUdpSocket(this);
    bool success = socket->bind(QHostAddress::AnyIPv4, UDP_PORT);
    if(success) {
        qDebug() << "UDP port " << UDP_PORT << " opened successfully";
        connect(socket, SIGNAL(readyRead()), this, SLOT(processPendingDatagrams()),Qt::DirectConnection);   //QueuedConnection fkt nicht kontinuierlich
    }
    else {
        qDebug() << "Failed to open UDP port " << UDP_PORT;
    }

    ui->centralWidget->setLayout(mainLayout);
}

MainWindow::~MainWindow()
{
    delete ui;
}

static inline int16_t as_signed(char low, char high)
{
    return (static_cast<uint8_t>(high) << 8) | static_cast<uint8_t>(low);
}

static inline QString hex(uint16_t u)
{
    return QString::number( u, 16 ).toUpper();
}

void MainWindow::processPendingDatagrams()
{
    QHostAddress sender;
    u_int16_t port;
    while (socket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(socket->pendingDatagramSize());
        socket->readDatagram(datagram.data(),datagram.size(),&sender,&port);

        //extract data
        unsigned short index = datagram[1] + (datagram[0] << 8);
        int16_t w_int = as_signed(datagram[3], datagram[2]);
        int16_t x_int = as_signed(datagram[5], datagram[4]);
        int16_t y_int = as_signed(datagram[7], datagram[6]);
        int16_t z_int = as_signed(datagram[9], datagram[8]);

        float w = static_cast<float>(w_int) / 32767.0f;
        float x = static_cast<float>(x_int) / 32767.0f;
        float y = static_cast<float>(y_int) / 32767.0f;
        float z = static_cast<float>(z_int) / 32767.0f;

        QQuaternion quat(w, x, y, z);
        glWidget->setImuOrientation(quat);
        qDebug() << index << "Quat:  " << w << ", " << x << ", " << y << ", " << z;
    }
}
