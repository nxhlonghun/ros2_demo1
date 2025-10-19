#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>

QT_BEGIN_NAMESPACE
namespace Ui
{
    class Widget;
}
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    QString qmsg;
    Widget(QWidget *parent = nullptr);
    ~Widget();
    void setlabel(const QString &qmsg);
private slots:
    void on_label_linkActivated(const QString &link);

private:
    Ui::Widget *ui;
};
#endif // WIDGET_H
