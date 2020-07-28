#ifndef PASSWINDOW_H
#define PASSWINDOW_H

#include <QDialog>

namespace Ui {
class PassWindow;
}

class PassWindow : public QDialog
{
    Q_OBJECT

public:
    explicit PassWindow(QWidget *parent = nullptr);
    ~PassWindow();
    QString password = "";

signals:
    void buttonPressed();

private slots:

    void on_pushButton_n0_clicked();

    void on_pushButton_n1_clicked();

    void on_pushButton_n2_clicked();

    void on_pushButton_n3_clicked();

    void on_pushButton_n4_clicked();

    void on_pushButton_n5_clicked();

    void on_pushButton_n6_clicked();

    void on_pushButton_n7_clicked();

    void on_pushButton_n8_clicked();

    void on_pushButton_n9_clicked();

    void on_pushButton_back_clicked();

    void on_pushButton_aceptar_clicked();

private:
    Ui::PassWindow *ui;
};

#endif // PASSWINDOW_H
