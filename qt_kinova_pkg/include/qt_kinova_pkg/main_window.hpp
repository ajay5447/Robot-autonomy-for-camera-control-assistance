/**
 * @file /include/autoCam_pkg/main_window.hpp
 *
 * @brief Qt based gui for autoCam_pkg.
 *
 * @date November 2010
 **/
#ifndef autoCam_pkg_MAIN_WINDOW_H
#define autoCam_pkg_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <QImage>
#include <QGraphicsScene>
#include "common.h"


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace autoCam_pkg {

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

//	void ReadSettings(); // Load up qt program settings at startup
//	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
//	void on_actionAbout_triggered();
//	void on_button_connect_clicked(bool check );
//	void on_checkbox_use_environment_stateChanged(int state);
//  void on_Manual_SwitchButton_clicked();

    /******************************************
    ** Manual connections
    *******************************************/
//    void updateLoggingView(); // no idea why this can't connect automatically

private Q_SLOTS:

  void observeCartesian();

  void observeJoint();

  void on_Manual_SwitchButton_pressed();

  void on_UP_pushButton_clicked();

  void on_DOWN_pushButton_clicked();

  void on_OUT_pushButton_clicked();

  void on_IN_pushButton_clicked();

  void on_RIGHT_pushButton_clicked();

  void on_LEFT_pushButton_clicked();

  void on_J1_horizontalSlider_valueChanged(int value);

  void on_J2_horizontalSlider_valueChanged(int value);

  void on_J3_horizontalSlider_valueChanged(int value);

  void on_J4_horizontalSlider_valueChanged(int value);

  void on_J5_horizontalSlider_valueChanged(int value);

  void on_J6_horizontalSlider_valueChanged(int value);

  void on_J7_horizontalSlider_valueChanged(int value);

  void on_Roll_dial_valueChanged(int value);

  void on_Yaw_horizontalSlider_valueChanged(int value);

  void on_Pitch_verticalSlider_valueChanged(int value);

  void on_view1_pushButton_clicked();

  void on_view2_pushButton_clicked();

  void on_view3_pushButton_clicked();

  void on_view4_pushButton_clicked();

  void on_view5_pushButton_clicked();

  void on_addView_pushButton_clicked();

  //void on_addName_lineEdit_textChanged(QKeyEvent *e);

  void on_applyNewName_pushButton_clicked();

  void on_NewUP_pushButton_clicked();

  void on_NewDOWN_pushButton_clicked();

  void on_NewLEFT_pushButton_clicked();

  void on_NewIN_pushButton_clicked();

  void on_NewOUT_pushButton_clicked();

  void on_NewRIGHT_pushButton_clicked();

  void on_NewJ1_horizontalSlider_valueChanged(int value);

  void on_NewJ2_horizontalSlider_valueChanged(int value);

  void on_NewJ3_horizontalSlider_valueChanged(int value);

  void on_NewJ4_horizontalSlider_valueChanged(int value);

  void on_NewJ5_horizontalSlider_valueChanged(int value);

  void on_NewJ6_horizontalSlider_valueChanged(int value);

  void on_NewJ7_horizontalSlider_valueChanged(int value);

  void on_Newdial_valueChanged(int value);

  void on_New_horizontalSlider_valueChanged(int value);

  void on_New_verticalSlider_valueChanged(int value);

  void on_SpeedUpdate_pushButton_clicked();

  void on_ViewingUpdate_pushButton_clicked();

  void on_ElecationUpdate_pushButton_clicked();

  void on_RollUpdate_pushButton_clicked();

  void on_HandUpdate_pushButton_clicked();

  void on_ResetVar_pushButton_clicked();

  void on_UpdateAllVars_pushButton_clicked();

  void on_selectView_comboBox_activated(const QString &arg1);

  void on_ApplyAllChanges_pushButton_clicked();

//  void on_Speed_horizontalSlider_valueChanged(int value);

//  void on_ViewingDistance_horizontalSlider_valueChanged(int value);

//  void on_Elevation_horizontalSlider_valueChanged(int value);

//  void on_CameraRoll_horizontalSlider_valueChanged(int value);

//  void on_LookAtHand_horizontalSlider_valueChanged(int value);

  void on_comboBox_currentIndexChanged(const QString &arg1);

  void toggleCartesianControl(bool enable);
  void toggleJointControl(bool enable);

  void on_X_lineEdit_editingFinished();
  void on_Y_lineEdit_editingFinished();
  void on_Z_lineEdit_editingFinished();

  void on_J1_lineEdit_editingFinished();
  void on_J2_lineEdit_editingFinished();
  void on_J3_lineEdit_editingFinished();
  void on_J4_lineEdit_editingFinished();
  void on_J5_lineEdit_editingFinished();
  void on_J6_lineEdit_editingFinished();
  void on_J7_lineEdit_editingFinished();

  void on_Roll_lineEdit_editingFinished();
  void on_Pitch_lineEdit_editingFinished();
  void on_Yaw_lineEdit_editingFinished();

private:
  Ui::MainWindow ui;
	QNode qnode;
  int switchCounter;
  QGraphicsPixmapItem pixmap;
  QGraphicsPixmapItem pixmap_2;
  bool imageStream;
  int XNEWcounter;
  int YNEWcounter;
  int ZNEWcounter;
  float Xcounter;
  float Ycounter;
  float Zcounter;
  double RollCounter;
  double PitchCounter;
  double YawCounter;
  double RollCounter_rad;
  double PitchCounter_rad;
  double YawCounter_rad;
  float J1counter;
  float J2counter;
  float J3counter;
  float J4counter;
  float J5counter;
  float J6counter;
  float J7counter;
};

}  // namespace autoCam_pkg

#endif // autoCam_pkg_MAIN_WINDOW_H
