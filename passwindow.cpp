#include "passwindow.h"
#include "ui_passwindow.h"
#include "mainwindow.h"
#include <QtDebug>
#include <QPixmap>

PassWindow::PassWindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PassWindow)
{
    ui->setupUi(this);
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    QPixmap picture(":/img/logo.jpeg");
    ui->label_logo2->setPixmap(picture.scaled(150,130,Qt::KeepAspectRatio));
}

PassWindow::~PassWindow()
{
    delete ui;
}



void PassWindow::on_pushButton_n0_clicked()
{
    password = ui->lineEdit->text();
    password+="0";
    ui->lineEdit->setText(password);
}

void PassWindow::on_pushButton_n1_clicked()
{
    password = ui->lineEdit->text();
    password+="1";
    ui->lineEdit->setText(password);
}

void PassWindow::on_pushButton_n2_clicked()
{
    password = ui->lineEdit->text();
    password+="2";
    ui->lineEdit->setText(password);
}

void PassWindow::on_pushButton_n3_clicked()
{
    password = ui->lineEdit->text();
    password+="3";
    ui->lineEdit->setText(password);
}

void PassWindow::on_pushButton_n4_clicked()
{
    password = ui->lineEdit->text();
    password+="4";
    ui->lineEdit->setText(password);
}

void PassWindow::on_pushButton_n5_clicked()
{
    password = ui->lineEdit->text();
    password+="5";
    ui->lineEdit->setText(password);
}

void PassWindow::on_pushButton_n6_clicked()
{
    password = ui->lineEdit->text();
    password+="6";
    ui->lineEdit->setText(password);
}

void PassWindow::on_pushButton_n7_clicked()
{
    password = ui->lineEdit->text();
    password+="7";
    ui->lineEdit->setText(password);
}

void PassWindow::on_pushButton_n8_clicked()
{
    password = ui->lineEdit->text();
    password+="8";
    ui->lineEdit->setText(password);
}

void PassWindow::on_pushButton_n9_clicked()
{
    password = ui->lineEdit->text();
    password+="9";
    ui->lineEdit->setText(password);
}

void PassWindow::on_pushButton_back_clicked()
{
    password = ui->lineEdit->text();
    password.remove(password.length()-1,1);
    ui->lineEdit->setText(password);
}

void PassWindow::on_pushButton_aceptar_clicked()
{
    if(ui->lineEdit->text()=="1234"){
        qDebug() << "Password correcto";
        //connect(password,)
        emit buttonPressed();

        //MainWindow::changeModeService();
    }
    else {
        qDebug() << "Pasword incorrecto";
    }


    this->close();
}
