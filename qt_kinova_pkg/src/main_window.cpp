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
#include "ui_main_window.h"
#include "../include/qt_kinova_pkg/main_window.hpp"
#include "../include/qt_kinova_pkg/common.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace autoCam_pkg {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  qnode(argc,argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  /* Set the window icon because why not */
  setWindowIcon(QIcon(":/images/icon.png"));

  /* Connect to the ROS Master */
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /* Show a warning of the ROS master is not running */
    if ( !qnode.init() ) {
      showNoMasterMessage();
    }

  /* Attach scene and pixmap to the graphics views for the cameras. */
    ui.graphicsView->setScene(new QGraphicsScene(this));
    ui.graphicsView->scene()->addItem(&pixmap);

    ui.graphicsView_2->setScene(new QGraphicsScene(this));
    ui.graphicsView_2->scene()->addItem(&pixmap_2);

    ui.tabWidget->setCurrentIndex(0);

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
float round(float var)
{
    // 37.66666 * 100 =3766.66
    // 3766.66 + .5 =3767.16    for rounding off value
    // then type cast to int so value is 3767
    // then divided by 100 so the value converted into 37.67
    float value = (int)(var * 100 + .5);
    return (float)value / 100;
}
void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
  msgBox.setText("Couldn't find the ROS master.");
	msgBox.exec();
    close();
}

void MainWindow::closeEvent(QCloseEvent *event) {
  imageStream = false;
  //WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::on_Manual_SwitchButton_pressed() {
  ui.Manual_SwitchButton->setText(QString("Switch Cameras"));
  ui.SpeedWeight_lineEdit->setText("100");

  ui.ViewingWeight_lineEdit->setText("100");


  ui.ElevationWeight_lineEdit->setText("100");


  ui.RollWeight_lineEdit->setText("100");


  ui.HandWeight_lineEdit->setText("100");
  //figure out which widget should be made larger or smaller
  QGraphicsView* makeLarger = ui.graphicsView;
  QGraphicsView* makeSmaller = ui.graphicsView_2;
  ui.statusBar->showMessage("Showing Head Camera",3500);

  if (switchCounter%2 == 1)
  {
      makeLarger = ui.graphicsView_2;
      makeSmaller = ui.graphicsView;
      ui.statusBar->showMessage("Showing Arm Camera",3500);
  }

  makeLarger->move(5,60);
  makeLarger->resize(640,640);
  makeSmaller->resize(180,180);
  makeSmaller->move(465,60);
  makeSmaller->raise();

  //increment the counter
  switchCounter = switchCounter + 1;
  std::cout << "switching camera \n";


  //check that both image streams are populated with images
  if(my_global_cv_ptr != nullptr && my_global_cv_ptr_2 != nullptr){
    imageStream = true;
//    std::cout << "Display: image not null \n";
    while(imageStream) {
      if(!my_global_cv_ptr->image.empty()) {
        QImage qimg(my_global_cv_ptr->image.data,
                    my_global_cv_ptr->image.cols,
                    my_global_cv_ptr->image.rows,
                    my_global_cv_ptr->image.step,
                    QImage::Format_RGB888);
        pixmap.setPixmap( QPixmap::fromImage(qimg.rgbSwapped()) );
        ui.graphicsView->fitInView(&pixmap, Qt::KeepAspectRatio);
      }
      if(!my_global_cv_ptr_2->image.empty()) {
        QImage qimg_2(my_global_cv_ptr_2->image.data,
                    my_global_cv_ptr_2->image.cols,
                    my_global_cv_ptr_2->image.rows,
                    my_global_cv_ptr_2->image.step,
                    QImage::Format_RGB888);
        pixmap_2.setPixmap( QPixmap::fromImage(qimg_2.rgbSwapped()) );
        ui.graphicsView_2->fitInView(&pixmap_2, Qt::KeepAspectRatio);
      }
      //you dont want to be spamming this during cartesian control because you won't be able to type in the boxes
        observeCartesian();
        observeJoint();
      qApp->processEvents();
    }
  }
}

// 2)
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/

/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
/*///////////////////////////////////////////////////////    Command Window Tab    //////////////////////////////////////////////////////////////*/
/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

/*/////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
// Command Window
// Manual Cartesian Control
// PushButton Names: UP_pushButton, DOWN_pushButton, LEFT_pushButton, RIGHT_pushButton
//                   IN_pushButton, OUT_pushButton
// TextEdit Box Names: X_lineEdit, Y_lineEdit, Z_lineEdit
// Dial and Slider Names: Roll_dial, Pitch_horizontalSlider, Yaw_verticalSlider
// TextEdit Box Names: Roll_lineEdit, Pitch_lineEdit, Yaw_lineEdit
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_UP_pushButton_clicked() {
  Zcounter = Zcounter + 0.1;
  observeCartesian();
}

void MainWindow::on_DOWN_pushButton_clicked() {
  Zcounter = Zcounter - 0.1;
  observeCartesian();
}

void MainWindow::on_OUT_pushButton_clicked() {
  Xcounter = Xcounter + 0.1;
  observeCartesian();
}

void MainWindow::on_IN_pushButton_clicked() {
  Xcounter = Xcounter - 0.1;
  observeCartesian();
}

void MainWindow::on_RIGHT_pushButton_clicked() {
  Ycounter = Ycounter + 0.1;
  observeCartesian();
}

void MainWindow::on_LEFT_pushButton_clicked() {
  Ycounter = Ycounter - 0.1;
  observeCartesian();
}


void MainWindow::on_X_lineEdit_editingFinished() {
  Xcounter = ui.X_lineEdit->text().toFloat();
  observeCartesian();
}
void MainWindow::on_Y_lineEdit_editingFinished() {
    Ycounter = ui.Y_lineEdit->text().toFloat();
    observeCartesian();
}
void MainWindow::on_Z_lineEdit_editingFinished() {
    Zcounter = ui.Z_lineEdit->text().toFloat();
    observeCartesian();
}

void MainWindow::on_Roll_lineEdit_editingFinished() {
    RollCounter = ui.Roll_lineEdit->text().toDouble();
    observeCartesian();
}

void MainWindow::on_Pitch_lineEdit_editingFinished() {
    PitchCounter = ui.Pitch_lineEdit->text().toDouble();
    observeCartesian();
}

void MainWindow::on_Yaw_lineEdit_editingFinished() {
    YawCounter = ui.Yaw_lineEdit->text().toDouble();
    observeCartesian();
}

void MainWindow::on_Roll_dial_valueChanged(int value) {
  RollCounter = double(value);
  observeCartesian();
}

void MainWindow::on_Yaw_horizontalSlider_valueChanged(int value) {
  YawCounter = double(value);
  observeCartesian();
}

void MainWindow::on_Pitch_verticalSlider_valueChanged(int value) {
  PitchCounter = double(value);
  observeCartesian();
}

void MainWindow::observeCartesian(){

  if (controlState.data != "cartesian") {
    /* Update position values */
    Xcounter = hand_camera_pose.position.x;
    Ycounter = hand_camera_pose.position.y;
    Zcounter = hand_camera_pose.position.z;
    /* Update orientation values */
    tf2::Quaternion quat_tf;
    tf2::fromMsg(hand_camera_pose.orientation, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(RollCounter_rad, PitchCounter_rad, YawCounter_rad);
    RollCounter = RollCounter_rad * 180.0 / M_PI;
    PitchCounter = PitchCounter_rad * 180.0 / M_PI;
    YawCounter = YawCounter_rad * 180.0 / M_PI;
  }
  else {
    //set position
    commandCardtesianPose.position.x = Xcounter;
    commandCardtesianPose.position.y = Ycounter;
    commandCardtesianPose.position.z = Zcounter;

    //set orientation
    RollCounter_rad = RollCounter * M_PI / 180.0;
    PitchCounter_rad = PitchCounter * M_PI / 180.0;
    YawCounter_rad = YawCounter * M_PI / 180.0;
    tf2::Quaternion quat_tf_pub;
    quat_tf_pub.setRPY(RollCounter_rad, PitchCounter_rad, YawCounter_rad);
    commandCardtesianPose.orientation = tf2::toMsg(quat_tf_pub);

    //publish
    qnode.publishControl();
  }

  //displays the values in the local variables
  ui.X_lineEdit->setText(QString::number(double(round(Xcounter))));
  ui.Y_lineEdit->setText(QString::number(double(round(Ycounter))));
  ui.Z_lineEdit->setText(QString::number(double(round(Zcounter))));

  ui.Roll_lineEdit->setText(QString::number(double(round(RollCounter))));
  ui.Pitch_lineEdit->setText(QString::number(double(round(PitchCounter))));
  ui.Yaw_lineEdit->setText(QString::number(double(round(YawCounter))));

  ui.Roll_dial->setValue(int(RollCounter));
  ui.Pitch_verticalSlider->setValue(int(PitchCounter));
  ui.Yaw_horizontalSlider->setValue(int(YawCounter));

}

/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Command Window
// Manual Joint Pose Control
// Horizontal Slider Names: J1_horizontalSlider, J2_horizontalSlider, J3_horizontalSlider,
//                          J4_horizontalSlider, J5_horizontalSlider, J6_horizontalSlider,
//                          J7_horizontalSlider
// TextEdit Box Names: J1_lineEdit, J2_lineEdit, J3_lineEdit,J4_lineEdit, J5_lineEdit,
//                     J6_lineEdit, J7_lineEdit
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/

void MainWindow::on_J1_horizontalSlider_valueChanged(int value) {
  J1counter = float(value);
  observeJoint();
}

void MainWindow::on_J2_horizontalSlider_valueChanged(int value) {
  J2counter = float(value);
  observeJoint();
}

void MainWindow::on_J3_horizontalSlider_valueChanged(int value) {
  J3counter = float(value);
  observeJoint();
}

void MainWindow::on_J4_horizontalSlider_valueChanged(int value) {
  J4counter = float(value);
  observeJoint();
}

void MainWindow::on_J5_horizontalSlider_valueChanged(int value) {
  J5counter = float(value);
  observeJoint();
}

void MainWindow::on_J6_horizontalSlider_valueChanged(int value) {
  J6counter = float(value);
  observeJoint();
}

void MainWindow::on_J7_horizontalSlider_valueChanged(int value) {
  J7counter = float(value);
  observeJoint();
}

void MainWindow::on_J1_lineEdit_editingFinished() {
  J1counter = ui.J1_lineEdit->text().toFloat();
  observeJoint();
}
void MainWindow::on_J2_lineEdit_editingFinished() {
  J2counter = ui.J2_lineEdit->text().toFloat();
  observeJoint();
}
void MainWindow::on_J3_lineEdit_editingFinished() {
  J3counter = ui.J3_lineEdit->text().toFloat();
  observeJoint();
}
void MainWindow::on_J4_lineEdit_editingFinished() {
  J4counter = ui.J4_lineEdit->text().toFloat();
  observeJoint();
}
void MainWindow::on_J5_lineEdit_editingFinished() {
  J5counter = ui.J5_lineEdit->text().toFloat();
  observeJoint();
}
void MainWindow::on_J6_lineEdit_editingFinished() {
  J6counter = ui.J6_lineEdit->text().toFloat();
  observeJoint();
}
void MainWindow::on_J7_lineEdit_editingFinished() {
  J7counter = ui.J7_lineEdit->text().toFloat();
  observeJoint();
}

void MainWindow::observeJoint(){

  if (controlState.data != "joint") {

    //update values
    J1counter = float(jointState.position[7])*float(180.0/M_PI);
    J2counter = float(jointState.position[8])*float(180.0/M_PI);
    J3counter = float(jointState.position[9])*float(180.0/M_PI);
    J4counter = float(jointState.position[10])*float(180.0/M_PI);
    J5counter = float(jointState.position[11])*float(180.0/M_PI);
    J6counter = float(jointState.position[12])*float(180.0/M_PI);
    J7counter = float(jointState.position[13])*float(180.0/M_PI);
    //std::cout<<jointState.position[7];
  }
  else{
    //sensor_msgs::jointState
    commandJointPose.position = {double(J1counter*float(M_PI/180.0)),double(J2counter*float(M_PI/180.0)),
                                 double(J3counter*float(M_PI/180.0)),double(J4counter*float(M_PI/180.0)),
                                 double(J5counter*float(M_PI/180.0)),double(J6counter*float(M_PI/180.0)),
                                 double(J7counter*float(M_PI/180.0))};
    commandJointPose.name = {"rightjoint_1", "rightjoint_2", "rightjoint_3", "rightjoint_4", "rightjoint_5", "rightjoint_6",
                             "rightjoint_7"};
//    commandJointPose.position[8] = double(J2counter);
//    commandJointPose.position[9] = double(J3counter);
//    commandJointPose.position[7] = double(J1counter);
//    commandJointPose.position[10] = double(J4counter);
//    commandJointPose.position[11] = double(J5counter);
//    commandJointPose.position[12] = double(J6counter);
//    commandJointPose.position[13] = double(J7counter);
    qnode.publishControl();
  }

  //displays the values in the local variables
    ui.J1_lineEdit->setText(QString::number(double(round(J1counter))));
    ui.J2_lineEdit->setText(QString::number(double(round(J2counter))));
    ui.J3_lineEdit->setText(QString::number(double(round(J3counter))));
    ui.J4_lineEdit->setText(QString::number(double(round(J4counter))));
    ui.J5_lineEdit->setText(QString::number(double(round(J5counter))));
    ui.J6_lineEdit->setText(QString::number(double(round(J6counter))));
    ui.J7_lineEdit->setText(QString::number(double(round(J7counter))));

    ui.J1_horizontalSlider->setValue(int(J1counter));
    ui.J2_horizontalSlider->setValue(int(J2counter));
    ui.J3_horizontalSlider->setValue(int(J3counter));
    ui.J4_horizontalSlider->setValue(int(J4counter));
    ui.J5_horizontalSlider->setValue(int(J5counter));
    ui.J6_horizontalSlider->setValue(int(J6counter));
    ui.J7_horizontalSlider->setValue(int(J7counter));
//        setTickPosition(QString::number(double(round(J1counter))))
}

/*/////////////////////////////////////////////////////////////////////////////////////////

/*/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
// Command Window
// Standard Views Available
// Push Button Name: StandardView1_pushButton, StandardView2_pushButton, StandardView3_pushButton
//                   StandardView4_pushButton, StandardView5_pushButton
///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////*/


/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Command Window
// User Added View Selection
// Push Button Names: view1_pushButton, view2_pushButton, view3_pushButton, view4_pushButton,
//                    view5_pushButton
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/

void MainWindow::on_view1_pushButton_clicked()
{

}

void MainWindow::on_view2_pushButton_clicked()
{

}

void MainWindow::on_view3_pushButton_clicked()
{

}

void MainWindow::on_view4_pushButton_clicked()
{

}

void MainWindow::on_view5_pushButton_clicked()
{

}
/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Command Window
// Add a View
// Push Button Name: addView_pushButton
// TextEdit Box Names:addName_lineEdit
// Push Button Names: view1_pushButton, view2_pushButton, view3_pushButton, view4_pushButton,
//                    view5_pushButton
// Combo Box Name: view_ComboBox
// This function will replace the empty views untill all are full
// When all are full, you can replace specific views using the comboBox
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/

int count=0;
void MainWindow::on_addView_pushButton_clicked()
{
        count++;
        if (count==1){
          //This view has a name
          QString name1 = ui.addName_lineEdit->text();
          ui.view1_pushButton->setText("1: " + name1);
          ui.addName_lineEdit->clear();
          //This view has a cartesian pose for x,yz
          QString X1 = ui.X_lineEdit->text();
          QString Y1 = ui.Y_lineEdit->text();
          QString Z1 = ui.Z_lineEdit->text();
          ui.X_lineEdit->clear();
          ui.Y_lineEdit->clear();
          ui.Z_lineEdit->clear();
          //This view has 7DOF joint angles
          QString J1_1 = ui.J1_lineEdit->text();
          QString J2_1 = ui.J2_lineEdit->text();
          QString J3_1 = ui.J3_lineEdit->text();
          QString J4_1 = ui.J4_lineEdit->text();
          QString J5_1 = ui.J5_lineEdit->text();
          QString J6_1 = ui.J6_lineEdit->text();
          QString J7_1 = ui.J7_lineEdit->text();
          ui.J1_horizontalSlider->setValue(0);
          ui.J2_horizontalSlider->setValue(0);
          ui.J3_horizontalSlider->setValue(0);
          ui.J4_horizontalSlider->setValue(0);
          ui.J5_horizontalSlider->setValue(0);
          ui.J6_horizontalSlider->setValue(0);
          ui.J7_horizontalSlider->setValue(0);
          ui.J1_lineEdit->clear();
          ui.J2_lineEdit->clear();
          ui.J3_lineEdit->clear();
          ui.J4_lineEdit->clear();
          ui.J5_lineEdit->clear();
          ui.J6_lineEdit->clear();
          ui.J7_lineEdit->clear();
          //This view has a roll,pitch, and yaw
          QString ROLL1 = ui.Roll_lineEdit->text();
          QString PITCH1 = ui.Pitch_lineEdit->text();
          QString YAW1 = ui.Yaw_lineEdit->text();
          ui.Roll_dial->setValue(0);
          ui.Yaw_horizontalSlider->setValue(0);
          ui.Pitch_verticalSlider->setValue(0);
          ui.Roll_lineEdit->clear();
          ui.Pitch_lineEdit->clear();
          ui.Yaw_lineEdit->clear();
          //Assign these views to View 1
          ui.NEWNAME1->setText(name1);
          ui.NewX1->setText(X1);
          ui.NewY1->setText(Y1);
          ui.NewZ1->setText(Z1);
          ui.NewJ1_1->setText(J1_1);
          ui.NewJ2_1->setText(J2_1);
          ui.NewJ3_1->setText(J3_1);
          ui.NewJ4_1->setText(J4_1);
          ui.NewJ5_1->setText(J5_1);
          ui.NewJ6_1->setText(J6_1);
          ui.NewJ7_1->setText(J7_1);
          ui.NewROLL1->setText(ROLL1);
          ui.NewPITCH1->setText(PITCH1);
          ui.NewYAW1->setText(YAW1);
        }
        if (count==2){
          //This view has a name
          QString name2 = ui.addName_lineEdit->text();
          ui.view2_pushButton->setText("2: " + name2);
          ui.addName_lineEdit->clear();
          //This view has a cartesian pose for x,yz
          QString X2 = ui.X_lineEdit->text();
          QString Y2 = ui.Y_lineEdit->text();
          QString Z2 = ui.Z_lineEdit->text();
          ui.X_lineEdit->clear();
          ui.Y_lineEdit->clear();
          ui.Z_lineEdit->clear();
          //This view has 7DOF joint angles
          QString J1_2 = ui.J1_lineEdit->text();
          QString J2_2 = ui.J2_lineEdit->text();
          QString J3_2 = ui.J3_lineEdit->text();
          QString J4_2 = ui.J4_lineEdit->text();
          QString J5_2 = ui.J5_lineEdit->text();
          QString J6_2 = ui.J6_lineEdit->text();
          QString J7_2 = ui.J7_lineEdit->text();
          ui.J1_horizontalSlider->setValue(0);
          ui.J2_horizontalSlider->setValue(0);
          ui.J3_horizontalSlider->setValue(0);
          ui.J4_horizontalSlider->setValue(0);
          ui.J5_horizontalSlider->setValue(0);
          ui.J6_horizontalSlider->setValue(0);
          ui.J7_horizontalSlider->setValue(0);
          ui.J1_lineEdit->clear();
          ui.J2_lineEdit->clear();
          ui.J3_lineEdit->clear();
          ui.J4_lineEdit->clear();
          ui.J5_lineEdit->clear();
          ui.J6_lineEdit->clear();
          ui.J7_lineEdit->clear();
          //This view has a roll,pitch, and yaw
          QString ROLL2 = ui.Roll_lineEdit->text();
          QString PITCH2 = ui.Pitch_lineEdit->text();
          QString YAW2 = ui.Yaw_lineEdit->text();
          ui.Roll_dial->setValue(0);
          ui.Yaw_horizontalSlider->setValue(0);
          ui.Pitch_verticalSlider->setValue(0);
          ui.Roll_lineEdit->clear();
          ui.Pitch_lineEdit->clear();
          ui.Yaw_lineEdit->clear();
          //Assign these views to View 2
          ui.NEWNAME2->setText(name2);
          ui.NewX2->setText(X2);
          ui.NewY2->setText(Y2);
          ui.NewZ2->setText(Z2);
          ui.NewJ1_2->setText(J1_2);
          ui.NewJ2_2->setText(J2_2);
          ui.NewJ3_2->setText(J3_2);
          ui.NewJ4_2->setText(J4_2);
          ui.NewJ5_2->setText(J5_2);
          ui.NewJ6_2->setText(J6_2);
          ui.NewJ7_2->setText(J7_2);
          ui.NewROLL2->setText(ROLL2);
          ui.NewPITCH2->setText(PITCH2);
          ui.NewYAW2->setText(YAW2);
        }
        if (count==3){
          //This view has a name
          QString name3 = ui.addName_lineEdit->text();
          ui.view3_pushButton->setText("3: " + name3);
          ui.addName_lineEdit->clear();
          //This view has a cartesian pose for x,yz
          QString X3 = ui.X_lineEdit->text();
          QString Y3 = ui.Y_lineEdit->text();
          QString Z3 = ui.Z_lineEdit->text();
          ui.X_lineEdit->clear();
          ui.Y_lineEdit->clear();
          ui.Z_lineEdit->clear();
          //This view has 7DOF joint angles
          QString J1_3 = ui.J1_lineEdit->text();
          QString J2_3 = ui.J2_lineEdit->text();
          QString J3_3 = ui.J3_lineEdit->text();
          QString J4_3 = ui.J4_lineEdit->text();
          QString J5_3 = ui.J5_lineEdit->text();
          QString J6_3 = ui.J6_lineEdit->text();
          QString J7_3 = ui.J7_lineEdit->text();
          ui.J1_horizontalSlider->setValue(0);
          ui.J2_horizontalSlider->setValue(0);
          ui.J3_horizontalSlider->setValue(0);
          ui.J4_horizontalSlider->setValue(0);
          ui.J5_horizontalSlider->setValue(0);
          ui.J6_horizontalSlider->setValue(0);
          ui.J7_horizontalSlider->setValue(0);
          ui.J1_lineEdit->clear();
          ui.J2_lineEdit->clear();
          ui.J3_lineEdit->clear();
          ui.J4_lineEdit->clear();
          ui.J5_lineEdit->clear();
          ui.J6_lineEdit->clear();
          ui.J7_lineEdit->clear();
          //This view has a roll,pitch, and yaw
          QString ROLL3 = ui.Roll_lineEdit->text();
          QString PITCH3 = ui.Pitch_lineEdit->text();
          QString YAW3 = ui.Yaw_lineEdit->text();
          ui.Roll_dial->setValue(0);
          ui.Yaw_horizontalSlider->setValue(0);
          ui.Pitch_verticalSlider->setValue(0);
          ui.Roll_lineEdit->clear();
          ui.Pitch_lineEdit->clear();
          ui.Yaw_lineEdit->clear();
          //Assign these views to View 3
          ui.NEWNAME3->setText(name3);
          ui.NewX3->setText(X3);
          ui.NewY3->setText(Y3);
          ui.NewZ3->setText(Z3);
          ui.NewJ1_3->setText(J1_3);
          ui.NewJ2_3->setText(J2_3);
          ui.NewJ3_3->setText(J3_3);
          ui.NewJ4_3->setText(J4_3);
          ui.NewJ5_3->setText(J5_3);
          ui.NewJ6_3->setText(J6_3);
          ui.NewJ7_3->setText(J7_3);
          ui.NewROLL3->setText(ROLL3);
          ui.NewPITCH3->setText(PITCH3);
          ui.NewYAW3->setText(YAW3);
        }
        if (count==4){
          //This view has a name
          QString name4 = ui.addName_lineEdit->text();
          ui.view4_pushButton->setText("4: " + name4);
          ui.addName_lineEdit->clear();
          //This view has a cartesian pose for x,yz
          QString X4 = ui.X_lineEdit->text();
          QString Y4 = ui.Y_lineEdit->text();
          QString Z4 = ui.Z_lineEdit->text();
          ui.X_lineEdit->clear();
          ui.Y_lineEdit->clear();
          ui.Z_lineEdit->clear();
          //This view has 7DOF joint angles
          QString J1_4 = ui.J1_lineEdit->text();
          QString J2_4 = ui.J2_lineEdit->text();
          QString J3_4 = ui.J3_lineEdit->text();
          QString J4_4 = ui.J4_lineEdit->text();
          QString J5_4 = ui.J5_lineEdit->text();
          QString J6_4 = ui.J6_lineEdit->text();
          QString J7_4 = ui.J7_lineEdit->text();
          ui.J1_horizontalSlider->setValue(0);
          ui.J2_horizontalSlider->setValue(0);
          ui.J3_horizontalSlider->setValue(0);
          ui.J4_horizontalSlider->setValue(0);
          ui.J5_horizontalSlider->setValue(0);
          ui.J6_horizontalSlider->setValue(0);
          ui.J7_horizontalSlider->setValue(0);
          ui.J1_lineEdit->clear();
          ui.J2_lineEdit->clear();
          ui.J3_lineEdit->clear();
          ui.J4_lineEdit->clear();
          ui.J5_lineEdit->clear();
          ui.J6_lineEdit->clear();
          ui.J7_lineEdit->clear();
          //This view has a roll,pitch, and yaw
          QString ROLL4 = ui.Roll_lineEdit->text();
          QString PITCH4 = ui.Pitch_lineEdit->text();
          QString YAW4 = ui.Yaw_lineEdit->text();
          ui.Roll_dial->setValue(0);
          ui.Yaw_horizontalSlider->setValue(0);
          ui.Pitch_verticalSlider->setValue(0);
          ui.Roll_lineEdit->clear();
          ui.Pitch_lineEdit->clear();
          ui.Yaw_lineEdit->clear();
          //Assign these views to View 4
          ui.NEWNAME4->setText(name4);
          ui.NewX4->setText(X4);
          ui.NewY4->setText(Y4);
          ui.NewZ4->setText(Z4);
          ui.NewJ1_4->setText(J1_4);
          ui.NewJ2_4->setText(J2_4);
          ui.NewJ3_4->setText(J3_4);
          ui.NewJ4_4->setText(J4_4);
          ui.NewJ5_4->setText(J5_4);
          ui.NewJ6_4->setText(J6_4);
          ui.NewJ7_4->setText(J7_4);
          ui.NewROLL4->setText(ROLL4);
          ui.NewPITCH4->setText(PITCH4);
          ui.NewYAW4->setText(YAW4);
        }
        if (count==5){
          //This view has a name
          QString name5 = ui.addName_lineEdit->text();
          ui.view5_pushButton->setText("5: " + name5);
          ui.addName_lineEdit->clear();
          //This view has a cartesian pose for x,yz
          QString X5 = ui.X_lineEdit->text();
          QString Y5 = ui.Y_lineEdit->text();
          QString Z5 = ui.Z_lineEdit->text();
          ui.X_lineEdit->clear();
          ui.Y_lineEdit->clear();
          ui.Z_lineEdit->clear();
          //This view has 7DOF joint angles
          QString J1_5 = ui.J1_lineEdit->text();
          QString J2_5 = ui.J2_lineEdit->text();
          QString J3_5 = ui.J3_lineEdit->text();
          QString J4_5 = ui.J4_lineEdit->text();
          QString J5_5 = ui.J5_lineEdit->text();
          QString J6_5 = ui.J6_lineEdit->text();
          QString J7_5 = ui.J7_lineEdit->text();
          ui.J1_horizontalSlider->setValue(0);
          ui.J2_horizontalSlider->setValue(0);
          ui.J3_horizontalSlider->setValue(0);
          ui.J4_horizontalSlider->setValue(0);
          ui.J5_horizontalSlider->setValue(0);
          ui.J6_horizontalSlider->setValue(0);
          ui.J7_horizontalSlider->setValue(0);
          ui.J1_lineEdit->clear();
          ui.J2_lineEdit->clear();
          ui.J3_lineEdit->clear();
          ui.J4_lineEdit->clear();
          ui.J5_lineEdit->clear();
          ui.J6_lineEdit->clear();
          ui.J7_lineEdit->clear();
          //This view has a roll,pitch, and yaw
          QString ROLL5 = ui.Roll_lineEdit->text();
          QString PITCH5 = ui.Pitch_lineEdit->text();
          QString YAW5 = ui.Yaw_lineEdit->text();
          ui.Roll_dial->setValue(0);
          ui.Yaw_horizontalSlider->setValue(0);
          ui.Pitch_verticalSlider->setValue(0);
          ui.Roll_lineEdit->clear();
          ui.Pitch_lineEdit->clear();
          ui.Yaw_lineEdit->clear();
          //Assign these views to View 5
          ui.NEWNAME5->setText(name5);
          ui.NewX5->setText(X5);
          ui.NewY5->setText(Y5);
          ui.NewZ5->setText(Z5);
          ui.NewJ1_5->setText(J1_5);
          ui.NewJ2_5->setText(J2_5);
          ui.NewJ3_5->setText(J3_5);
          ui.NewJ4_5->setText(J4_5);
          ui.NewJ5_5->setText(J5_5);
          ui.NewJ6_5->setText(J6_5);
          ui.NewJ7_5->setText(J7_5);
          ui.NewROLL5->setText(ROLL5);
          ui.NewPITCH5->setText(PITCH5);
          ui.NewYAW5->setText(YAW5);
        }
        /*else {
            QString currentState = ui.view_comboBox->currentText();
            if (currentState == "View 1"){
                QString name1 = ui.addName_lineEdit->text();
                ui.view1_pushButton->setText("1: " + name1);
                ui.addName_lineEdit->clear();}
            if (currentState == "View 2"){
                QString name2 = ui.addName_lineEdit->text();
                ui.view2_pushButton->setText("2: " + name2);
                ui.addName_lineEdit->clear();}
            if (currentState == "View 3"){
                QString name3 = ui.addName_lineEdit->text();
                ui.view3_pushButton->setText("3: " + name3);
                ui.addName_lineEdit->clear();}
            if (currentState == "View 4"){
                QString name4 = ui.addName_lineEdit->text();
                ui.view4_pushButton->setText("4: " + name4);
                ui.addName_lineEdit->clear();}
            if (currentState == "View 5"){
                QString name5 = ui.addName_lineEdit->text();
                ui.view5_pushButton->setText("5: " + name5);
                ui.addName_lineEdit->clear();
            }
        }*/
}

/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
/*///////////////////////////////////////////////////////    View Editor Tab    /////////////////////////////////////////////////////////////////*/
/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

/*/////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
// Edit Views Window
// Edit Cartesian Control
// PushButton Names: NewUP_pushButton, NewDOWN_pushButton, NewLEFT_pushButton,
//                   NewRIGHT_pushButton, NewIN_pushButton, NewOUT_pushButton
// TextEdit Box Names: XNEW_lineEdit, YNEW_lineEdit, ZNEW_lineEdit
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_NewUP_pushButton_clicked()
{
    ZNEWcounter++;
    QString strZ = QString::number(ZNEWcounter);
    ui.ZNEW_lineEdit->setText(strZ);
}

void MainWindow::on_NewDOWN_pushButton_clicked()
{
    ZNEWcounter--;
    QString strZ = QString::number(ZNEWcounter);
    ui.ZNEW_lineEdit->setText(strZ);
}

void MainWindow::on_NewLEFT_pushButton_clicked()
{
    YNEWcounter--;
    QString strY = QString::number(YNEWcounter);
    ui.YNEW_lineEdit->setText(strY);
}

void MainWindow::on_NewRIGHT_pushButton_clicked()
{
    YNEWcounter++;
    QString strY = QString::number(YNEWcounter);
    ui.YNEW_lineEdit->setText(strY);
}

void MainWindow::on_NewIN_pushButton_clicked()
{
    XNEWcounter--;
    QString strX = QString::number(XNEWcounter);
    ui.XNEW_lineEdit->setText(strX);
}

void MainWindow::on_NewOUT_pushButton_clicked()
{
    XNEWcounter++;
    QString strX = QString::number(XNEWcounter);
    ui.XNEW_lineEdit->setText(strX);
}
/*//////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// Edit Views Window
// Edit Joint Pose Control
// Horizontal Slider Names: NewJ1_horizontalSlider, NewJ2_horizontalSlider, NewJ3_horizontalSlider,
//                          NewJ4_horizontalSlider, NewJ5_horizontalSlider, NewJ6_horizontalSlider,
//                          NewJ7_horizontalSlider
// TextEdit Box Names: NewJ1_lineEdit, NewJ2_lineEdit, NewJ3_lineEdit, NewJ4_lineEdit,
//                     NewJ5_lineEdit, NewJ6_lineEdit, NewJ7_lineEdit
////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_NewJ1_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewJ1_lineEdit->setText(b);
}

void MainWindow::on_NewJ2_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewJ2_lineEdit->setText(b);
}

void MainWindow::on_NewJ3_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewJ3_lineEdit->setText(b);
}

void MainWindow::on_NewJ4_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewJ4_lineEdit->setText(b);
}

void MainWindow::on_NewJ5_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewJ5_lineEdit->setText(b);
}

void MainWindow::on_NewJ6_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewJ6_lineEdit->setText(b);
}

void MainWindow::on_NewJ7_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewJ7_lineEdit->setText(b);
}
/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Edit Views Window
// Edit Camera Control
// Dial and Slider Names: NewRoll_dial, NewPitch_horizontalSlider, NewYaw_verticalSlider
// TextEdit Box Names: NewRoll_lineEdit, NewPitch_lineEdit, NewYaw_lineEdit
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/

void MainWindow::on_Newdial_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewRoll_lineEdit->setText(b);
}

void MainWindow::on_New_horizontalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewYaw_lineEdit->setText(b);
}

void MainWindow::on_New_verticalSlider_valueChanged(int value)
{
    QString b = QString::number(value);
    ui.NewPitch_lineEdit->setText(b);
}
/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Edit Views Window
// Change View Name
// Push Button Name: applyNewName_pushButton
// TextEdit Box Names:newName_lineEdit
// Push Button Names: view1_pushButton, view2_pushButton, view3_pushButton, view4_pushButton,
//                    view5_pushButton
// Combo Box Name: selectView_comboBox
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_applyNewName_pushButton_clicked()
{
    QString currentState = ui.selectView_comboBox->currentText();
    if (currentState == "View 1"){
        QString name = ui.newName_lineEdit->text();
        ui.view1_pushButton->setText("1: " + name);
        ui.newName_lineEdit->clear();
        ui.NEWNAME1->setText(name);
    }
    if (currentState == "View 2"){
        QString name = ui.newName_lineEdit->text();
        ui.view2_pushButton->setText("2: " + name);
        ui.newName_lineEdit->clear();
        ui.NEWNAME2->setText(name);
    }
    if (currentState == "View 3"){
        QString name = ui.newName_lineEdit->text();
        ui.view3_pushButton->setText("3: " + name);
        ui.newName_lineEdit->clear();
    }
    if (currentState == "View 4"){
        QString name = ui.newName_lineEdit->text();
        ui.view4_pushButton->setText("4: " + name);
        ui.newName_lineEdit->clear();
    }
    if (currentState == "View 5"){
        QString name = ui.newName_lineEdit->text();
        ui.view5_pushButton->setText("5: " + name);
        ui.newName_lineEdit->clear();
    }
}
/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Edit Views Window
// Apply All Changes
// Push Button Name:
// TextEdit Box Names:
// Push Button Names:
// Combo Box Name:
//This function clears and saves all new information when Apply All Changes is clicked
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_ApplyAllChanges_pushButton_clicked()
{
    QString currentState = ui.selectView_comboBox->currentText();
    if (currentState == "View 1"){
        QString name = ui.newName_lineEdit->text();
        ui.view1_pushButton->setText("1: " + name);
        ui.NEWNAME1->setText(name);
        //ui.newName_lineEdit->clear();

        QString NewX1 = ui.XNEW_lineEdit->text();
        QString NewY1 = ui.YNEW_lineEdit->text();
        QString NewZ1 = ui.ZNEW_lineEdit->text();
        ui.NewX1->setText(NewX1);
        ui.NewY1->setText(NewY1);
        ui.NewZ1->setText(NewZ1);
        /*ui.XNEW_lineEdit->clear();
        ui.YNEW_lineEdit->clear();
        ui.ZNEW_lineEdit->clear();*/

        QString NewJ1_1 = ui.NewJ1_lineEdit->text();
        QString NewJ2_1 = ui.NewJ2_lineEdit->text();
        QString NewJ3_1 = ui.NewJ3_lineEdit->text();
        QString NewJ4_1 = ui.NewJ4_lineEdit->text();
        QString NewJ5_1 = ui.NewJ5_lineEdit->text();
        QString NewJ6_1 = ui.NewJ6_lineEdit->text();
        QString NewJ7_1 = ui.NewJ7_lineEdit->text();
        ui.NewJ1_1->setText(NewJ1_1);
        ui.NewJ2_1->setText(NewJ2_1);
        ui.NewJ3_1->setText(NewJ3_1);
        ui.NewJ4_1->setText(NewJ4_1);
        ui.NewJ5_1->setText(NewJ5_1);
        ui.NewJ6_1->setText(NewJ6_1);
        ui.NewJ7_1->setText(NewJ7_1);
        /*ui.NewJ1_lineEdit->clear();
        ui.NewJ2_lineEdit->clear();
        ui.NewJ3_lineEdit->clear();
        ui.NewJ4_lineEdit->clear();
        ui.NewJ5_lineEdit->clear();
        ui.NewJ6_lineEdit->clear();
        ui.NewJ7_lineEdit->clear();*/

        QString NewROLL1 = ui.NewRoll_lineEdit->text();
        QString NewPITCH1 = ui.NewPitch_lineEdit->text();
        QString NewYAW1 = ui.NewYaw_lineEdit->text();
        ui.NewROLL1->setText(NewROLL1);
        ui.NewPITCH1->setText(NewPITCH1);
        ui.NewYAW1->setText(NewYAW1);
        /*ui.NewRoll_lineEdit->clear();
        ui.NewPitch_lineEdit->clear();
        ui.NewYaw_lineEdit->clear();*/

       /* ui.NewJ1_horizontalSlider->setValue(0);
        ui.NewJ2_horizontalSlider->setValue(0);
        ui.NewJ3_horizontalSlider->setValue(0);
        ui.NewJ4_horizontalSlider->setValue(0);
        ui.NewJ5_horizontalSlider->setValue(0);
        ui.NewJ6_horizontalSlider->setValue(0);
        ui.NewJ7_horizontalSlider->setValue(0);*/
    }
    if (currentState == "View 2"){
        QString name = ui.newName_lineEdit->text();
        ui.view2_pushButton->setText("2: " + name);
        ui.NEWNAME2->setText(name);
        //ui.newName_lineEdit->clear();

        QString NewX2 = ui.XNEW_lineEdit->text();
        QString NewY2 = ui.YNEW_lineEdit->text();
        QString NewZ2 = ui.ZNEW_lineEdit->text();
        ui.NewX2->setText(NewX2);
        ui.NewY2->setText(NewY2);
        ui.NewZ2->setText(NewZ2);
        /*ui.XNEW_lineEdit->clear();
        ui.YNEW_lineEdit->clear();
        ui.ZNEW_lineEdit->clear();*/

        QString NewJ1_2 = ui.NewJ1_lineEdit->text();
        QString NewJ2_2 = ui.NewJ2_lineEdit->text();
        QString NewJ3_2 = ui.NewJ3_lineEdit->text();
        QString NewJ4_2 = ui.NewJ4_lineEdit->text();
        QString NewJ5_2 = ui.NewJ5_lineEdit->text();
        QString NewJ6_2 = ui.NewJ6_lineEdit->text();
        QString NewJ7_2 = ui.NewJ7_lineEdit->text();
        ui.NewJ1_2->setText(NewJ1_2);
        ui.NewJ2_2->setText(NewJ2_2);
        ui.NewJ3_2->setText(NewJ3_2);
        ui.NewJ4_2->setText(NewJ4_2);
        ui.NewJ5_2->setText(NewJ5_2);
        ui.NewJ6_2->setText(NewJ6_2);
        ui.NewJ7_2->setText(NewJ7_2);
        /*ui.NewJ1_lineEdit->clear();
        ui.NewJ2_lineEdit->clear();
        ui.NewJ3_lineEdit->clear();
        ui.NewJ4_lineEdit->clear();
        ui.NewJ5_lineEdit->clear();
        ui.NewJ6_lineEdit->clear();
        ui.NewJ7_lineEdit->clear();*/

        QString NewROLL2 = ui.NewRoll_lineEdit->text();
        QString NewPITCH2 = ui.NewPitch_lineEdit->text();
        QString NewYAW2 = ui.NewYaw_lineEdit->text();
        ui.NewROLL2->setText(NewROLL2);
        ui.NewPITCH2->setText(NewPITCH2);
        ui.NewYAW2->setText(NewYAW2);
        /*ui.NewRoll_lineEdit->clear();
        ui.NewPitch_lineEdit->clear();
        ui.NewYaw_lineEdit->clear();*/
    }
    if (currentState == "View 3"){
        QString name = ui.newName_lineEdit->text();
        ui.view3_pushButton->setText("3: " + name);
        //ui.newName_lineEdit->clear();
        ui.NEWNAME3->setText(name);

        QString NewX3 = ui.XNEW_lineEdit->text();
        QString NewY3 = ui.YNEW_lineEdit->text();
        QString NewZ3 = ui.ZNEW_lineEdit->text();
        ui.NewX3->setText(NewX3);
        ui.NewY3->setText(NewY3);
        ui.NewZ3->setText(NewZ3);
        /*ui.XNEW_lineEdit->clear();
        ui.YNEW_lineEdit->clear();
        ui.ZNEW_lineEdit->clear();*/

        QString NewJ1_3 = ui.NewJ1_lineEdit->text();
        QString NewJ2_3 = ui.NewJ2_lineEdit->text();
        QString NewJ3_3 = ui.NewJ3_lineEdit->text();
        QString NewJ4_3 = ui.NewJ4_lineEdit->text();
        QString NewJ5_3 = ui.NewJ5_lineEdit->text();
        QString NewJ6_3 = ui.NewJ6_lineEdit->text();
        QString NewJ7_3 = ui.NewJ7_lineEdit->text();
        ui.NewJ1_3->setText(NewJ1_3);
        ui.NewJ2_3->setText(NewJ2_3);
        ui.NewJ3_3->setText(NewJ3_3);
        ui.NewJ4_3->setText(NewJ4_3);
        ui.NewJ5_3->setText(NewJ5_3);
        ui.NewJ6_3->setText(NewJ6_3);
        ui.NewJ7_3->setText(NewJ7_3);
        /*ui.NewJ1_lineEdit->clear();
        ui.NewJ2_lineEdit->clear();
        ui.NewJ3_lineEdit->clear();
        ui.NewJ4_lineEdit->clear();
        ui.NewJ5_lineEdit->clear();
        ui.NewJ6_lineEdit->clear();
        ui.NewJ7_lineEdit->clear();*/

        QString NewROLL3 = ui.NewRoll_lineEdit->text();
        QString NewPITCH3 = ui.NewPitch_lineEdit->text();
        QString NewYAW3 = ui.NewYaw_lineEdit->text();
        ui.NewROLL3->setText(NewROLL3);
        ui.NewPITCH3->setText(NewPITCH3);
        ui.NewYAW3->setText(NewYAW3);
        /*ui.NewRoll_lineEdit->clear();
        ui.NewPitch_lineEdit->clear();
        ui.NewYaw_lineEdit->clear();*/
    }
    if (currentState == "View 4"){
        QString name = ui.newName_lineEdit->text();
        ui.view4_pushButton->setText("4: " + name);
        //ui.newName_lineEdit->clear();
        ui.NEWNAME4->setText(name);

        QString NewX4 = ui.XNEW_lineEdit->text();
        QString NewY4 = ui.YNEW_lineEdit->text();
        QString NewZ4 = ui.ZNEW_lineEdit->text();
        ui.NewX4->setText(NewX4);
        ui.NewY4->setText(NewY4);
        ui.NewZ4->setText(NewZ4);
        /*ui.XNEW_lineEdit->clear();
        ui.YNEW_lineEdit->clear();
        ui.ZNEW_lineEdit->clear();*/

        QString NewJ1_4 = ui.NewJ1_lineEdit->text();
        QString NewJ2_4 = ui.NewJ2_lineEdit->text();
        QString NewJ3_4 = ui.NewJ3_lineEdit->text();
        QString NewJ4_4 = ui.NewJ4_lineEdit->text();
        QString NewJ5_4 = ui.NewJ5_lineEdit->text();
        QString NewJ6_4 = ui.NewJ6_lineEdit->text();
        QString NewJ7_4 = ui.NewJ7_lineEdit->text();
        ui.NewJ1_4->setText(NewJ1_4);
        ui.NewJ2_4->setText(NewJ2_4);
        ui.NewJ3_4->setText(NewJ3_4);
        ui.NewJ4_4->setText(NewJ4_4);
        ui.NewJ5_4->setText(NewJ5_4);
        ui.NewJ6_4->setText(NewJ6_4);
        ui.NewJ7_4->setText(NewJ7_4);
        /*ui.NewJ1_lineEdit->clear();
        ui.NewJ2_lineEdit->clear();
        ui.NewJ3_lineEdit->clear();
        ui.NewJ4_lineEdit->clear();
        ui.NewJ5_lineEdit->clear();
        ui.NewJ6_lineEdit->clear();
        ui.NewJ7_lineEdit->clear();*/

        QString NewROLL4 = ui.NewRoll_lineEdit->text();
        QString NewPITCH4 = ui.NewPitch_lineEdit->text();
        QString NewYAW4 = ui.NewYaw_lineEdit->text();
        ui.NewROLL4->setText(NewROLL4);
        ui.NewPITCH4->setText(NewPITCH4);
        ui.NewYAW4->setText(NewYAW4);
        /*ui.NewRoll_lineEdit->clear();
        ui.NewPitch_lineEdit->clear();
        ui.NewYaw_lineEdit->clear();*/
    }
    if (currentState == "View 5"){
        QString name = ui.newName_lineEdit->text();
        ui.view5_pushButton->setText("5: " + name);
        //ui.newName_lineEdit->clear();
        ui.NEWNAME5->setText(name);

        QString NewX5 = ui.XNEW_lineEdit->text();
        QString NewY5 = ui.YNEW_lineEdit->text();
        QString NewZ5 = ui.ZNEW_lineEdit->text();
        ui.NewX5->setText(NewX5);
        ui.NewY5->setText(NewY5);
        ui.NewZ5->setText(NewZ5);
        /*ui.XNEW_lineEdit->clear();
        ui.YNEW_lineEdit->clear();
        ui.ZNEW_lineEdit->clear();*/

        QString NewJ1_5 = ui.NewJ1_lineEdit->text();
        QString NewJ2_5 = ui.NewJ2_lineEdit->text();
        QString NewJ3_5 = ui.NewJ3_lineEdit->text();
        QString NewJ4_5 = ui.NewJ4_lineEdit->text();
        QString NewJ5_5 = ui.NewJ5_lineEdit->text();
        QString NewJ6_5 = ui.NewJ6_lineEdit->text();
        QString NewJ7_5 = ui.NewJ7_lineEdit->text();
        ui.NewJ1_5->setText(NewJ1_5);
        ui.NewJ2_5->setText(NewJ2_5);
        ui.NewJ3_5->setText(NewJ3_5);
        ui.NewJ4_5->setText(NewJ4_5);
        ui.NewJ5_5->setText(NewJ5_5);
        ui.NewJ6_5->setText(NewJ6_5);
        ui.NewJ7_5->setText(NewJ7_5);
        /*ui.NewJ1_lineEdit->clear();
        ui.NewJ2_lineEdit->clear();
        ui.NewJ3_lineEdit->clear();
        ui.NewJ4_lineEdit->clear();
        ui.NewJ5_lineEdit->clear();
        ui.NewJ6_lineEdit->clear();
        ui.NewJ7_lineEdit->clear();*/

        QString NewROLL5 = ui.NewRoll_lineEdit->text();
        QString NewPITCH5 = ui.NewPitch_lineEdit->text();
        QString NewYAW5 = ui.NewYaw_lineEdit->text();
        ui.NewROLL5->setText(NewROLL5);
        ui.NewPITCH5->setText(NewPITCH5);
        ui.NewYAW5->setText(NewYAW5);
        /*ui.NewRoll_lineEdit->clear();
        ui.NewPitch_lineEdit->clear();
        ui.NewYaw_lineEdit->clear();*/
    }
}
/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Edit Views Window
// Edit Specific View
// Push Button Name:
// TextEdit Box Names:
// Push Button Names:
// Combo Box Name:
// This function populates the tab with the information specific to that tab
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_selectView_comboBox_activated(const QString &arg1)
{
    if(arg1 == "View 1"){
        QString NEWNAME1 = ui.NEWNAME1->text();
        QString NewX1 = ui.NewX1->text();
        QString NewY1 = ui.NewY1->text();
        QString NewZ1 = ui.NewZ1->text();
        QString NewJ1_1 = ui.NewJ1_1->text();
        QString NewJ2_1 = ui.NewJ2_1->text();
        QString NewJ3_1 = ui.NewJ3_1->text();
        QString NewJ4_1 = ui.NewJ4_1->text();
        QString NewJ5_1 = ui.NewJ5_1->text();
        QString NewJ6_1 = ui.NewJ6_1->text();
        QString NewJ7_1 = ui.NewJ7_1->text();
        QString NewROLL1 = ui.NewROLL1->text();
        QString NewPITCH1 = ui.NewPITCH1->text();
        QString NewYAW1 = ui.NewYAW1->text();
        ui.newName_lineEdit->setText(NEWNAME1);
        ui.XNEW_lineEdit->setText(NewX1);
        ui.YNEW_lineEdit->setText(NewY1);
        ui.ZNEW_lineEdit->setText(NewZ1);
        ui.NewJ1_lineEdit->setText(NewJ1_1);
        float value1 = NewJ1_1.toFloat();
        ui.NewJ1_horizontalSlider->setValue(value1);
        ui.NewJ2_lineEdit->setText(NewJ2_1);
        float value2 = NewJ2_1.toFloat();
        ui.NewJ2_horizontalSlider->setValue(value2);
        ui.NewJ3_lineEdit->setText(NewJ3_1);
        float value3 = NewJ3_1.toFloat();
        ui.NewJ3_horizontalSlider->setValue(value3);
        ui.NewJ4_lineEdit->setText(NewJ4_1);
        float value4 = NewJ4_1.toFloat();
        ui.NewJ4_horizontalSlider->setValue(value4);
        ui.NewJ5_lineEdit->setText(NewJ5_1);
        float value5 = NewJ5_1.toFloat();
        ui.NewJ5_horizontalSlider->setValue(value5);
        ui.NewJ6_lineEdit->setText(NewJ6_1);
        float value6 = NewJ6_1.toFloat();
        ui.NewJ6_horizontalSlider->setValue(value6);
        ui.NewJ7_lineEdit->setText(NewJ7_1);
        float value7 = NewJ7_1.toFloat();
        ui.NewJ7_horizontalSlider->setValue(value7);
        ui.NewRoll_lineEdit->setText(NewROLL1);
        ui.NewPitch_lineEdit->setText(NewPITCH1);
        ui.NewYaw_lineEdit->setText(NewYAW1);
    }
    else if(arg1 == "View 2"){
        QString NEWNAME2 = ui.NEWNAME2->text();
        ui.newName_lineEdit->setText(NEWNAME2);
        QString NewX2 = ui.NewX2->text();
        QString NewY2 = ui.NewY2->text();
        QString NewZ2 = ui.NewZ2->text();
        QString NewJ1_2 = ui.NewJ1_2->text();
        QString NewJ2_2 = ui.NewJ2_2->text();
        QString NewJ3_2 = ui.NewJ3_2->text();
        QString NewJ4_2 = ui.NewJ4_2->text();
        QString NewJ5_2 = ui.NewJ5_2->text();
        QString NewJ6_2 = ui.NewJ6_2->text();
        QString NewJ7_2 = ui.NewJ7_2->text();
        QString NewROLL2 = ui.NewROLL2->text();
        QString NewPITCH2 = ui.NewPITCH2->text();
        QString NewYAW2 = ui.NewYAW2->text();
        ui.newName_lineEdit->setText(NEWNAME2);
        ui.XNEW_lineEdit->setText(NewX2);
        ui.YNEW_lineEdit->setText(NewY2);
        ui.ZNEW_lineEdit->setText(NewZ2);
        ui.NewJ1_lineEdit->setText(NewJ1_2);
        float value1 = NewJ1_2.toFloat();
        ui.NewJ1_horizontalSlider->setValue(value1);
        ui.NewJ2_lineEdit->setText(NewJ2_2);
        float value2 = NewJ2_2.toFloat();
        ui.NewJ2_horizontalSlider->setValue(value2);
        ui.NewJ3_lineEdit->setText(NewJ3_2);
        float value3 = NewJ3_2.toFloat();
        ui.NewJ3_horizontalSlider->setValue(value3);
        ui.NewJ4_lineEdit->setText(NewJ4_2);
        float value4 = NewJ4_2.toFloat();
        ui.NewJ4_horizontalSlider->setValue(value4);
        ui.NewJ5_lineEdit->setText(NewJ5_2);
        float value5 = NewJ5_2.toFloat();
        ui.NewJ5_horizontalSlider->setValue(value5);
        ui.NewJ6_lineEdit->setText(NewJ6_2);
        float value6 = NewJ6_2.toFloat();
        ui.NewJ6_horizontalSlider->setValue(value6);
        ui.NewJ7_lineEdit->setText(NewJ7_2);
        float value7 = NewJ7_2.toFloat();
        ui.NewJ7_horizontalSlider->setValue(value7);
        ui.NewRoll_lineEdit->setText(NewROLL2);
        ui.NewPitch_lineEdit->setText(NewPITCH2);
        ui.NewYaw_lineEdit->setText(NewYAW2);

    }
    else if(arg1 == "View 3"){
        QString NEWNAME3 = ui.NEWNAME3->text();
        ui.newName_lineEdit->setText(NEWNAME3);
        QString NewX3 = ui.NewX3->text();
        QString NewY3 = ui.NewY3->text();
        QString NewZ3 = ui.NewZ3->text();
        QString NewJ1_3 = ui.NewJ1_3->text();
        QString NewJ2_3 = ui.NewJ2_3->text();
        QString NewJ3_3 = ui.NewJ3_3->text();
        QString NewJ4_3 = ui.NewJ4_3->text();
        QString NewJ5_3 = ui.NewJ5_3->text();
        QString NewJ6_3 = ui.NewJ6_3->text();
        QString NewJ7_3 = ui.NewJ7_3->text();
        QString NewROLL3 = ui.NewROLL3->text();
        QString NewPITCH3 = ui.NewPITCH3->text();
        QString NewYAW3 = ui.NewYAW3->text();
        ui.newName_lineEdit->setText(NEWNAME3);
        ui.XNEW_lineEdit->setText(NewX3);
        ui.YNEW_lineEdit->setText(NewY3);
        ui.ZNEW_lineEdit->setText(NewZ3);
        ui.NewJ1_lineEdit->setText(NewJ1_3);
        float value1 = NewJ1_3.toFloat();
        ui.NewJ1_horizontalSlider->setValue(value1);
        ui.NewJ2_lineEdit->setText(NewJ2_3);
        float value2 = NewJ2_3.toFloat();
        ui.NewJ2_horizontalSlider->setValue(value2);
        ui.NewJ3_lineEdit->setText(NewJ3_3);
        float value3 = NewJ3_3.toFloat();
        ui.NewJ3_horizontalSlider->setValue(value3);
        ui.NewJ4_lineEdit->setText(NewJ4_3);
        float value4 = NewJ4_3.toFloat();
        ui.NewJ4_horizontalSlider->setValue(value4);
        ui.NewJ5_lineEdit->setText(NewJ5_3);
        float value5 = NewJ5_3.toFloat();
        ui.NewJ5_horizontalSlider->setValue(value5);
        ui.NewJ6_lineEdit->setText(NewJ6_3);
        float value6 = NewJ6_3.toFloat();
        ui.NewJ6_horizontalSlider->setValue(value6);
        ui.NewJ7_lineEdit->setText(NewJ7_3);
        float value7 = NewJ7_3.toFloat();
        ui.NewJ7_horizontalSlider->setValue(value7);
        ui.NewRoll_lineEdit->setText(NewROLL3);
        ui.NewPitch_lineEdit->setText(NewPITCH3);
        ui.NewYaw_lineEdit->setText(NewYAW3);
    }
    else if(arg1 == "View 4"){
        QString NEWNAME4 = ui.NEWNAME4->text();
        ui.newName_lineEdit->setText(NEWNAME4);
        QString NewX4 = ui.NewX4->text();
        QString NewY4 = ui.NewY4->text();
        QString NewZ4 = ui.NewZ4->text();
        QString NewJ1_4 = ui.NewJ1_4->text();
        QString NewJ2_4 = ui.NewJ2_4->text();
        QString NewJ3_4 = ui.NewJ3_4->text();
        QString NewJ4_4 = ui.NewJ4_4->text();
        QString NewJ5_4 = ui.NewJ5_4->text();
        QString NewJ6_4 = ui.NewJ6_4->text();
        QString NewJ7_4 = ui.NewJ7_4->text();
        QString NewROLL4 = ui.NewROLL4->text();
        QString NewPITCH4 = ui.NewPITCH4->text();
        QString NewYAW4 = ui.NewYAW4->text();
        ui.newName_lineEdit->setText(NEWNAME4);
        ui.XNEW_lineEdit->setText(NewX4);
        ui.YNEW_lineEdit->setText(NewY4);
        ui.ZNEW_lineEdit->setText(NewZ4);
        ui.NewJ1_lineEdit->setText(NewJ1_4);
        float value1 = NewJ1_4.toFloat();
        ui.NewJ1_horizontalSlider->setValue(value1);
        ui.NewJ2_lineEdit->setText(NewJ2_4);
        float value2 = NewJ2_4.toFloat();
        ui.NewJ2_horizontalSlider->setValue(value2);
        ui.NewJ3_lineEdit->setText(NewJ3_4);
        float value3 = NewJ3_4.toFloat();
        ui.NewJ3_horizontalSlider->setValue(value3);
        ui.NewJ4_lineEdit->setText(NewJ4_4);
        float value4 = NewJ4_4.toFloat();
        ui.NewJ4_horizontalSlider->setValue(value4);
        ui.NewJ5_lineEdit->setText(NewJ5_4);
        float value5 = NewJ5_4.toFloat();
        ui.NewJ5_horizontalSlider->setValue(value5);
        ui.NewJ6_lineEdit->setText(NewJ6_4);
        float value6 = NewJ6_4.toFloat();
        ui.NewJ6_horizontalSlider->setValue(value6);
        ui.NewJ7_lineEdit->setText(NewJ7_4);
        float value7 = NewJ7_4.toFloat();
        ui.NewJ7_horizontalSlider->setValue(value7);
        ui.NewRoll_lineEdit->setText(NewROLL4);
        ui.NewPitch_lineEdit->setText(NewPITCH4);
        ui.NewYaw_lineEdit->setText(NewYAW4);
    }
    else if (arg1 == "View 5"){
        QString NEWNAME5 = ui.NEWNAME5->text();
        ui.newName_lineEdit->setText(NEWNAME5);
        QString NewX5 = ui.NewX5->text();
        QString NewY5 = ui.NewY5->text();
        QString NewZ5 = ui.NewZ5->text();
        QString NewJ1_5 = ui.NewJ1_5->text();
        QString NewJ2_5 = ui.NewJ2_5->text();
        QString NewJ3_5 = ui.NewJ3_5->text();
        QString NewJ4_5 = ui.NewJ4_5->text();
        QString NewJ5_5 = ui.NewJ5_5->text();
        QString NewJ6_5 = ui.NewJ6_5->text();
        QString NewJ7_5 = ui.NewJ7_5->text();
        QString NewROLL5 = ui.NewROLL5->text();
        QString NewPITCH5 = ui.NewPITCH5->text();
        QString NewYAW5 = ui.NewYAW5->text();
        ui.newName_lineEdit->setText(NEWNAME5);
        ui.XNEW_lineEdit->setText(NewX5);
        ui.YNEW_lineEdit->setText(NewY5);
        ui.ZNEW_lineEdit->setText(NewZ5);
        ui.NewJ1_lineEdit->setText(NewJ1_5);
        float value1 = NewJ1_5.toFloat();
        ui.NewJ1_horizontalSlider->setValue(value1);
        ui.NewJ2_lineEdit->setText(NewJ2_5);
        float value2 = NewJ2_5.toFloat();
        ui.NewJ2_horizontalSlider->setValue(value2);
        ui.NewJ3_lineEdit->setText(NewJ3_5);
        float value3 = NewJ3_5.toFloat();
        ui.NewJ3_horizontalSlider->setValue(value3);
        ui.NewJ4_lineEdit->setText(NewJ4_5);
        float value4 = NewJ4_5.toFloat();
        ui.NewJ4_horizontalSlider->setValue(value4);
        ui.NewJ5_lineEdit->setText(NewJ5_5);
        float value5 = NewJ5_5.toFloat();
        ui.NewJ5_horizontalSlider->setValue(value5);
        ui.NewJ6_lineEdit->setText(NewJ6_5);
        float value6 = NewJ6_5.toFloat();
        ui.NewJ6_horizontalSlider->setValue(value6);
        ui.NewJ7_lineEdit->setText(NewJ7_5);
        float value7 = NewJ7_5.toFloat();
        ui.NewJ7_horizontalSlider->setValue(value7);
        ui.NewRoll_lineEdit->setText(NewROLL5);
        ui.NewPitch_lineEdit->setText(NewPITCH5);
        ui.NewYaw_lineEdit->setText(NewYAW5);    }
}
/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
/*///////////////////////////////////////////////////////    Rlexed IK Editor Tab    ////////////////////////////////////////////////////////////*/
/*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Relaxed IK Editor Window
// Update Specific Variable
// Dial and Slider Names:
// TextEdit Box Names:
// This function saves specific new variable information
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_SpeedUpdate_pushButton_clicked()
{
    //QString CurrentSpeedVal = ui.SpeedVal_lineEdit->text();
    //ui.SpeedVal_lineEdit->setText(CurrentSpeedVal);
    QString LookAtObjectWeight = ui.SpeedWeight_lineEdit->text();
    ui.SpeedWeight_lineEdit->setText(LookAtObjectWeight);
    float firstWeight = ui.SpeedWeight_lineEdit->text().toFloat();
    relaxedik_weights.position = {firstWeight/100.0, 1.0, 1.0, 1.0, 1.0};
    relaxedik_weights.name = {"Arm0_Look_At_Obj()", "Arm0_High()", "Roll_Limit()", "Away_From_Head()", "Dist_To_Target()"};
    qnode.publishControl();

}

void MainWindow::on_ViewingUpdate_pushButton_clicked()
{
    //QString CurrentViewingVal = ui.ViewingVal_lineEdit->text();
    //ui.ViewingVal_lineEdit->setText(CurrentViewingVal);
    QString CurrentViewingWeight = ui.ViewingWeight_lineEdit->text();
    ui.ViewingWeight_lineEdit->setText(CurrentViewingWeight);
    float secondWeight = ui.ViewingWeight_lineEdit->text().toFloat();
    relaxedik_weights.position = {1, secondWeight/100.0, 1.0, 1.0, 1.0};
    relaxedik_weights.name = {"Arm0_Look_At_Obj()", "Arm0_High()", "Roll_Limit()", "Away_From_Head()", "Dist_To_Target()"};
    qnode.publishControl();
}

void MainWindow::on_ElecationUpdate_pushButton_clicked()
{
    //QString CurrentElevationVal = ui.ElevationVal_lineEdit->text();
    //ui.ElevationVal_lineEdit->setText(CurrentElevationVal);
    QString CurrentElevationWeight = ui.ElevationWeight_lineEdit->text();
    ui.ElevationWeight_lineEdit->setText(CurrentElevationWeight);
    float thirdWeight = ui.ElevationWeight_lineEdit->text().toFloat();
    relaxedik_weights.position = {1.0, 1.0, thirdWeight/100.0, 1.0, 1.0};
    relaxedik_weights.name = {"Arm0_Look_At_Obj()", "Arm0_High()", "Roll_Limit()", "Away_From_Head()", "Dist_To_Target()"};
    qnode.publishControl();
}

void MainWindow::on_RollUpdate_pushButton_clicked()
{
    QString CurrentRollWeight = ui.RollWeight_lineEdit->text();
    ui.RollWeight_lineEdit->setText(CurrentRollWeight);
    float fourthWeight = ui.RollWeight_lineEdit->text().toFloat();
    relaxedik_weights.position = {1.0, 1.0, 1.0, fourthWeight/100.0, 1.0};
    relaxedik_weights.name = {"Arm0_Look_At_Obj()", "Arm0_High()", "Roll_Limit()", "Away_From_Head()", "Dist_To_Target()"};
    qnode.publishControl();
}

void MainWindow::on_HandUpdate_pushButton_clicked()
{
    QString CurrentHandWeight = ui.HandWeight_lineEdit->text();
    ui.HandWeight_lineEdit->setText(CurrentHandWeight);
    float fifthWeight = ui.HandWeight_lineEdit->text().toFloat();
    relaxedik_weights.position = {1.0, 1.0, 1.0, 1.0, fifthWeight/100.0};
    relaxedik_weights.name = {"Arm0_Look_At_Obj()", "Arm0_High()", "Roll_Limit()", "Away_From_Head()", "Dist_To_Target()"};
    qnode.publishControl();
}

/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Relaxed IK Editor Window
// Update Specific Variable
// Dial and Slider Names:
// TextEdit Box Names:
// This function resets the values and weights
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_ResetVar_pushButton_clicked()
{
    ui.SpeedWeight_lineEdit->setText("100");

    ui.ViewingWeight_lineEdit->setText("100");


    ui.ElevationWeight_lineEdit->setText("100");


    ui.RollWeight_lineEdit->setText("100");


    ui.HandWeight_lineEdit->setText("100");
}
/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Relaxed IK Editor Window
// Update Specific Variable
// Dial and Slider Names:
// TextEdit Box Names:
// This function updates and saves the values and weights
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/
void MainWindow::on_UpdateAllVars_pushButton_clicked()
{
    //QString CurrentSpeedVal = ui.SpeedVal_lineEdit->text();
    //ui.SpeedVal_lineEdit->setText(CurrentSpeedVal);
    QString LookAtObjectWeight = ui.SpeedWeight_lineEdit->text();
    ui.SpeedWeight_lineEdit->setText(LookAtObjectWeight);
    float value1 = LookAtObjectWeight.toFloat();


    //QString CurrentViewingVal = ui.ViewingVal_lineEdit->text();
    //ui.ViewingVal_lineEdit->setText(CurrentViewingVal);
    QString CurrentViewingWeight = ui.ViewingWeight_lineEdit->text();
    ui.ViewingWeight_lineEdit->setText(CurrentViewingWeight);
    float value2 = CurrentViewingWeight.toFloat();


    //QString CurrentElevationVal = ui.ElevationVal_lineEdit->text();
    //ui.ElevationVal_lineEdit->setText(CurrentElevationVal);
    QString CurrentElevationWeight = ui.ElevationWeight_lineEdit->text();
    ui.ElevationWeight_lineEdit->setText(CurrentElevationWeight);
    float value3 = CurrentElevationWeight.toFloat();


    QString CurrentRollWeight = ui.RollWeight_lineEdit->text();
    ui.RollWeight_lineEdit->setText(CurrentRollWeight);
    float value4 = CurrentRollWeight.toFloat();



    QString CurrentHandWeight = ui.HandWeight_lineEdit->text();
    ui.HandWeight_lineEdit->setText(CurrentHandWeight);
    float value5 = CurrentHandWeight.toFloat();

    relaxedik_weights.position = {value1/100.0, value2/100.0, value3/100.0, value4/100.0, value5/100.0};
    relaxedik_weights.name = {"Arm0_Look_At_Obj()", "Arm0_High()", "Roll_Limit()", "Away_From_Head()", "Dist_To_Target()"};
    qnode.publishControl();

}
/*/////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
// Relaxed IK Editor Window
// Weights Sliders
// Slider Names:
// TextEdit Box Names:
// This function updates the text edit boxes with the weights given by the sliders
///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////*/
//void MainWindow::on_Speed_horizontalSlider_valueChanged(int value)
//{
//    QString b = QString::number(value);
//    ui.LookAtObject_lineEdit->setText(b);
//}

//void MainWindow::on_ViewingDistance_horizontalSlider_valueChanged(int value)
//{
//    QString b = QString::number(value);
//    ui.ViewingWeight_lineEdit->setText(b);
//}

//void MainWindow::on_Elevation_horizontalSlider_valueChanged(int value)
//{
//    QString b = QString::number(value);
//    ui.ElevationWeight_lineEdit->setText(b);
//}

//void MainWindow::on_CameraRoll_horizontalSlider_valueChanged(int value)
//{
//    QString b = QString::number(value);
//    ui.RollWeight_lineEdit->setText(b);
//}

//void MainWindow::on_LookAtHand_horizontalSlider_valueChanged(int value)
//{
//    QString b = QString::number(value);
//    ui.HandWeight_lineEdit->setText(b);
//}

/* Change control method */

void MainWindow::on_comboBox_currentIndexChanged(const QString &arg1)
{
  //removes the initial <select> item upon selecting a control method
  if (ui.comboBox->count() > 3) {
    ui.comboBox->removeItem(0);
  }
  /* Build the string message */
  std::stringstream ss;
  if (arg1 == "joint"){
    ss << "joint";
    toggleCartesianControl(false);
    toggleJointControl(true);
  }
  else if (arg1 == "cartesian") {
    ss << "cartesian";
    toggleCartesianControl(true);
    toggleJointControl(false);
  }
  else {
    ss << "objective";
    toggleCartesianControl(false);
    toggleJointControl(false);
  }

  /* publish updated control method */
  controlState.data = ss.str();
  qnode.publishControl();
}

/* enables or disables cartesian controls, when disabled they display current state */
void MainWindow::toggleCartesianControl(bool enable) {
  if (enable) {
    //position buttons
    ui.LEFT_pushButton->setEnabled(true);
    ui.RIGHT_pushButton->setEnabled(true);
    ui.UP_pushButton->setEnabled(true);
    ui.DOWN_pushButton->setEnabled(true);
    ui.IN_pushButton->setEnabled(true);
    ui.OUT_pushButton->setEnabled(true);
    //position line edits
    ui.X_lineEdit->setReadOnly(false);
    ui.Y_lineEdit->setReadOnly(false);
    ui.Z_lineEdit->setReadOnly(false);
    //orientation buttons
    ui.Roll_dial->setEnabled(true);
    ui.Yaw_horizontalSlider->setEnabled(true);
    ui.Pitch_verticalSlider->setEnabled(true);
    //orientation line edits
    ui.Roll_lineEdit->setReadOnly(false);
    ui.Yaw_lineEdit->setReadOnly(false);
    ui.Pitch_lineEdit->setReadOnly(false);
  }
  else {
    //buttons
    ui.LEFT_pushButton->setEnabled(false);
    ui.RIGHT_pushButton->setEnabled(false);
    ui.UP_pushButton->setEnabled(false);
    ui.DOWN_pushButton->setEnabled(false);
    ui.IN_pushButton->setEnabled(false);
    ui.OUT_pushButton->setEnabled(false);
    //text edits
    ui.X_lineEdit->setReadOnly(true);
    ui.Y_lineEdit->setReadOnly(true);
    ui.Z_lineEdit->setReadOnly(true);
    //orientation buttons
    ui.Roll_dial->setEnabled(false);
    ui.Yaw_horizontalSlider->setEnabled(false);
    ui.Pitch_verticalSlider->setEnabled(false);
    //orientation line edits
    ui.Roll_lineEdit->setReadOnly(true);
    ui.Yaw_lineEdit->setReadOnly(true);
    ui.Pitch_lineEdit->setReadOnly(true);
  }
}

/* enables or disables joint controls, when disabled they display current state */
void MainWindow::toggleJointControl(bool enable) {
  if (enable) {
    //joint sliders
    ui.J1_horizontalSlider->setEnabled(true);
    ui.J2_horizontalSlider->setEnabled(true);
    ui.J3_horizontalSlider->setEnabled(true);
    ui.J4_horizontalSlider->setEnabled(true);
    ui.J5_horizontalSlider->setEnabled(true);
    ui.J6_horizontalSlider->setEnabled(true);
    ui.J7_horizontalSlider->setEnabled(true);

    //joint line edits
    ui.J1_lineEdit->setReadOnly(false);
    ui.J2_lineEdit->setReadOnly(false);
    ui.J3_lineEdit->setReadOnly(false);
    ui.J4_lineEdit->setReadOnly(false);
    ui.J5_lineEdit->setReadOnly(false);
    ui.J6_lineEdit->setReadOnly(false);
    ui.J7_lineEdit->setReadOnly(false);
  }
  else {
    //joint sliders
    ui.J1_horizontalSlider->setEnabled(false);
    ui.J2_horizontalSlider->setEnabled(false);
    ui.J3_horizontalSlider->setEnabled(false);
    ui.J4_horizontalSlider->setEnabled(false);
    ui.J5_horizontalSlider->setEnabled(false);
    ui.J6_horizontalSlider->setEnabled(false);
    ui.J7_horizontalSlider->setEnabled(false);

    //joint line edits
    ui.J1_lineEdit->setReadOnly(true);
    ui.J2_lineEdit->setReadOnly(true);
    ui.J3_lineEdit->setReadOnly(true);
    ui.J4_lineEdit->setReadOnly(true);
    ui.J5_lineEdit->setReadOnly(true);
    ui.J6_lineEdit->setReadOnly(true);
    ui.J7_lineEdit->setReadOnly(true);
  }
}


}  // namespace autoCam_pkg
