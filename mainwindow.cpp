#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "functions.cpp"
#include "files.cpp"
#include "wiringPiI2C.h"

#include "passwindow.h"
#include "homewidow.h"

#include <QtDebug>
#include <QTimer>
#include <QDateTime>
#include <QDesktopWidget>
#include <QScreen>
#include <QMessageBox>
#include <QMetaEnum>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <QStyle>
#include <QDesktopWidget>
#include <linux/i2c-dev.h>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    //initSystem();
    QString testSystem = initSystem();
    qDebug() << testSystem;
    Homewidow homeDialog;
    homeDialog.setModal(true);
    homeDialog.setText(testSystem);
    homeDialog.exec();

    QRect screenGeometry = QApplication::desktop()->screenGeometry();
    int x = (screenGeometry.width()-this->width()) / 2;
    int y = (screenGeometry.height()-this->height()) / 2;
    this->move(x, y);
    this->show();
}

MainWindow::~MainWindow()
{
    delete ui;
}




void MainWindow::on_pushButton_service_clicked()
{
    passWindowCon = new PassWindow();
    QObject::connect(passWindowCon, SIGNAL(buttonPressed()), this, SLOT(changeModeService()));
    passWindowCon->show();

}

void MainWindow::changeModeService()
{
    qDebug() << "Mode service on";
    ui->tabWidget->setTabEnabled(3, true);
    ui->tabWidget->setTabEnabled(4, true);
    ui->tabWidget->setTabEnabled(5, true);

}



void MainWindow::on_pushButton_testGPIO_clicked()
{
    pinMode(4, INPUT);
    pinMode(17, INPUT);
    pinMode(27, INPUT);
    pinMode(22, INPUT);
    pinMode(0, INPUT);
    pinMode(26, INPUT);
    pinMode(14, INPUT);
    pinMode(15, INPUT);
    pinMode(18, INPUT);
    pinMode(23, INPUT);
    pinMode(24, INPUT);
    pinMode(25, INPUT);
    pinMode(8, INPUT);

    qDebug() <<
    "| 4:" << digitalRead(4) <<
    "| 17:" << digitalRead(17) <<
    "| 27:" << digitalRead(27) <<
    "| 22:" << digitalRead(22) <<
    "| 0:" << digitalRead(0) <<
    "| 26:" << digitalRead(26) <<
    "| 14:" << digitalRead(14) <<
    "| 15:" << digitalRead(15) <<
    "| 18:" << digitalRead(18) <<
    "| 23:" << digitalRead(23) <<
    "| 24:" << digitalRead(24) <<
    "| 25:" << digitalRead(25) <<
    "| 8:" << digitalRead(8);


}

void MainWindow::on_radioButton_esp_clicked()
{
    espontaneoPeriod = millis();
    ui->tabWidget_sel->setCurrentIndex(0);
    ui->tabWidget_sel->setTabEnabled(1,false);

}

void MainWindow::on_radioButton_assit_clicked()
{
    ui->tabWidget_sel->setTabEnabled(1,true);
}

void MainWindow::on_radioButton_manda_clicked()
{
    ui->tabWidget_sel->setTabEnabled(1,true);
}
