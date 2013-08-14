/**
 * @file /include/ihm/main_window.hpp
 *
 * @brief Qt based gui for ihm.
 *
 * @date November 2010
 **/
#ifndef ihm_MAIN_WINDOW_H
#define ihm_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ihm {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public slots:
    void updateVelocity(double);
    void updateBrake(int);
    void updateThrottle(int);
    void updateHandBrake(int);
    void updateEngineSpeed(int);
    void updateGearShift(int);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace ihm

#endif // ihm_MAIN_WINDOW_H
