#include <QCoreApplication>
#include <QtWidgets/QApplication>
#include <iostream>
#include <string>
#include <QObject>

#include "mainwindow.h"
#include "URServer.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    URServer *urServer = new URServer(nullptr);
    MainWindow *controller = new MainWindow(nullptr, urServer);
    controller->show();

    

    return a.exec();
}
