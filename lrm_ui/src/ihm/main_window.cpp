/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/ihm/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ihm {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    ReadSettings();
    setWindowIcon(QIcon(":/images/lrm.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));


    QObject::connect(&qnode, SIGNAL(velocitySig(double)), this, SLOT(updateVelocity(double)));
    QObject::connect(&qnode, SIGNAL(brakeSig(int)), this, SLOT(updateBrake(int)));
    QObject::connect(&qnode, SIGNAL(throttleSig(int)), this, SLOT(updateThrottle(int)));
    QObject::connect(&qnode, SIGNAL(handBrakeSig(int)), this, SLOT(updateHandBrake(int)));
    QObject::connect(&qnode, SIGNAL(engineSpeedSig(int)), this, SLOT(updateEngineSpeed(int)));
    QObject::connect(&qnode, SIGNAL(gearShiftSig(int)), this, SLOT(updateGearShift(int)));
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}






/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

void MainWindow::updateVelocity(double a){
    ui.lcdNumber->display(a);
}

void MainWindow::updateBrake(int a){
    ui.progressBar->setValue((int)a);
}

void MainWindow::updateThrottle(int a){
    ui.progressBar_2->setValue((int)a);
}
void MainWindow::updateEngineSpeed(int a){
    ui.manometer->setValue((double)a/1000);
}

void MainWindow::updateGearShift(int a)
{
    switch(a){
    case -1:
        ui.gear->setText("R");
        break;
    case 0:
        ui.gear->setText("N");
        break;
    case 1:
        ui.gear->setText("1");
        break;
    case 2:
        ui.gear->setText("2");
        break;
    case 3:
        ui.gear->setText("3");
        break;
    case 4:
        ui.gear->setText("4");
        break;
    case 5:
        ui.gear->setText("5");
        break;
    }
}

void MainWindow::updateHandBrake(int a){
    if(a == 1){
        ui.led->setChecked(true);
    }else{
        ui.led->setChecked(false);
    }


}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "ihm");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "ihm");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}

}  // namespace ihm

