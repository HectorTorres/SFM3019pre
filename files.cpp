#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFile>
#include <QTextStream>
#include <QDateTime>


void MainWindow::getDateText(){
    dateTimeSys = QDateTime::currentDateTime();
}

bool MainWindow::initFile(){
    if(ui->radioButton_name->isChecked())
        nameFile = ui->lineEdit_textName->text();
    else {
        nameFile = dateTimeSys.toString(time_format);
        qDebug() << nameFile;
    }
    dirFile = ui->lineEdit_dir->text();

    QFile file(dirFile + nameFile + ".txt");
    if(file.open(QFile::Append | QFile::Text)){
    }
    else{
        return false;
    }
    QTextStream dataToFile(&file);
    return true;
}





void MainWindow::writeFile(QString textToFile){
    QFile file(dirFile + nameFile + ".txt");
    if(file.open(QFile::Append | QFile::Text)){
    }
    QTextStream dataToFile(&file);
    dataToFile << textToFile;
}
