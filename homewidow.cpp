#include "homewidow.h"
#include "ui_homewidow.h"
#include <QPixmap>

Homewidow::Homewidow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Homewidow)
{
    ui->setupUi(this);
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    QPixmap picture(":/img/logo.jpeg");
    ui->label_logo->setPixmap(picture.scaled(667,130,Qt::KeepAspectRatio));
}

Homewidow::~Homewidow()
{
    delete ui;
}

void Homewidow::on_pushButton_clicked()
{
    this->close();
}

void Homewidow::setPath(const QString &path)
{
    m_path = path;
}

void Homewidow::setText(QString textinfo)
{
    ui->label_status->setText(textinfo);
}
