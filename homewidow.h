#ifndef HOMEWIDOW_H
#define HOMEWIDOW_H

#include <QDialog>

namespace Ui {
class Homewidow;
}

class Homewidow : public QDialog
{
    Q_OBJECT

public:
    explicit Homewidow(QWidget *parent = nullptr);
    ~Homewidow();
    void setPath(const QString &path);
    void setText(QString textinfo);

private slots:
    void on_pushButton_clicked();

private:
    Ui::Homewidow *ui;
    QString m_path;

};

#endif // HOMEWIDOW_H
