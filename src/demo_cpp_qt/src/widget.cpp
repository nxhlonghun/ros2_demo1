#include "ui_widget.h"
#include "demo_cpp_qt/Widget.h"

Widget::Widget(QWidget *parent)
    : QWidget(parent), ui(new Ui::Widget)
{
    ui->setupUi(this);
    // ui->label->setText(qmsg);
    // ui->label->setText("qmsg");
}

Widget::~Widget()
{
    delete ui;
}

void Widget::on_label_linkActivated(const QString &link)
{
    ui->label->setText(link);
}

void Widget::setlabel(const QString &qmsg)
{
    on_label_linkActivated(qmsg);
}
