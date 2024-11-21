#include "dialog.h"
#include "ui_dialog.h"
 #include <QPixmap>
Dialog::Dialog(QWidget *parent)
  : QDialog(parent)
  , ui(new Ui::Dialog)
{
  ui->setupUi(this);

  QString imagePath1 = "/home/hai/1PID.PNG"; //
      qDebug() << "Current working directory:" << QDir::currentPath();
      QPixmap pix1(imagePath1);
      qDebug() << "Image loaded successfully!";
      ui->lbl_Image->setPixmap(pix1);
      ui->lbl_Image->setScaledContents(true);
      ui->lbl_Image->setFixedSize(650, 230);

  QString imagePath2 = "/home/hai/2PD_I.PNG"; //
      qDebug() << "Current working directory:" << QDir::currentPath();
      QPixmap pix2(imagePath2);
      qDebug() << "Image loaded successfully!";
      ui->lbl_image_2->setPixmap(pix2);
      ui->lbl_image_2->setScaledContents(true);
      ui->lbl_image_2->setFixedSize(670, 235);

  QString imagePath3 = "/home/hai/2PID.PNG"; //
      qDebug() << "Current working directory:" << QDir::currentPath();
      QPixmap pix3(imagePath3);
      qDebug() << "Image loaded successfully!";
      ui->lbl_image_3->setPixmap(pix3);
      ui->lbl_image_3->setScaledContents(true);
      ui->lbl_image_3->setFixedSize(670, 235);


  mChart = new QCustomPlot(this);
  mChart->setGeometry(6, 5.5, 1200, 520); // Adjust as needed

// Setup default graph for Acceleration:
  mChart->addGraph();
  mChart->graph(0)->setPen(QPen(Qt::green)); // Color for the graph
  mChart->graph(0)->setName("ControlPID");

// Setup default graph for Gyroscope:
  mChart->addGraph();
  mChart->graph(1)->setPen(QPen(Qt::red)); // Color for the graph
  mChart->graph(1)->setName("Acceleration");

// Setup default graph for ControlPID:
  mChart->addGraph();
  mChart->graph(2)->setPen(QPen(Qt::blue)); // Color for the graph
  mChart->graph(2)->setName("Gyroscope");

// Включите легенду
  mChart->legend->setVisible(true);
  mChart->legend->setFont(QFont("Helvetica", 9)); // Установка шрифта легенды
  mChart->legend->setBrush(QBrush(QColor(255, 255, 255, 200))); // Фоновый цвет легенды

// Установите диапазон для осей y по умолчанию
  mChart->yAxis->setRange(-32000, 32000); // Настройте диапазоны по мере необходимости
  mChart->xAxis->setLabel("Time"); // Заголовок для оси X

  mThread = new MyThread();
  mThread->setParent((QObject*) this);

  connect(mThread, &MyThread::response, this, &Dialog::myResponse);
  //for manual:
  connect(ui->horizontalSlider, &QSlider::valueChanged, this, &Dialog::on_horizontalSlider_valueChanged);
  connect(ui->horizontalSlider_2, &QSlider::valueChanged, this, &Dialog::on_horizontalSlider_2_valueChanged);
  //for 1PID:
  connect(ui->horizontalSlider_3, &QSlider::valueChanged, this, &Dialog::on_horizontalSlider_3_valueChanged);
  //for 2PID:
  connect(ui->horizontalSlider_4, &QSlider::valueChanged, this, &Dialog::on_horizontalSlider_4_valueChanged);
  //for 1COMMON_PID:
  connect(ui->horizontalSlider_7, &QSlider::valueChanged, this, &Dialog::on_horizontalSlider_7_valueChanged);

}
void Dialog::myResponse(const int16_t* dta){
  int grLen = 500;
  double key = 0.001*(*( (uint32_t*) (&dta[2]) ) );
  mChart->graph(0)->addData(key, dta[0]);
  mChart->graph(1)->addData(key, dta[1]);
  if(mChart->graph(0)->data()->size() > grLen){
      mChart->graph(0)->data()->removeBefore(mChart->graph(0)->data()->at(
                                               mChart->graph(0)->data()->size() - grLen
                                               )->key);
      mChart->graph(1)->data()->removeBefore(mChart->graph(1)->data()->at(
                                         mChart->graph(1)->data()->size() - grLen
                                         )->key);
      double strt = mChart->graph(0)->data()->at(
            mChart->graph(0)->data()->size() - grLen
            )->key;
      mChart->xAxis->setRange(strt, key);
  }
  else {
      mChart->xAxis->setRange(0, key);
  }
  mChart->replot();
  printf("pnts=%d a%d g%d td%d\n", mChart->graph(0)->data()->size(), dta[0], dta[1], dta[4]);
}

Dialog::~Dialog()
{
  delete ui;
}

void Dialog::on_tabWidget_currentChanged(int index)
{
    printf("selected %d\n", index);
    mCurrMethod = index;
}
void Dialog::on_horizontalSlider_valueChanged(int value)
{
    ui->progressBar->setValue(value);
    PWM_manual = qBound(1000, value, 2000);
}
void Dialog::on_horizontalSlider_2_valueChanged(int value)
{
    ui->progressBar_2->setValue(value);
    PID_PWM_manual = qBound(-300, value, 300);
}
void Dialog::on_horizontalSlider_3_valueChanged(int value)
{
    ui->progressBar_3->setValue(value);
    PWM_1PID = qBound(1000, value, 2000);
}
void Dialog::on_horizontalSlider_4_valueChanged(int value)
{
    ui->progressBar_4->setValue(value);
    PWM_2PID = qBound(1000, value, 2000);
}

void Dialog::on_horizontalSlider_7_valueChanged(int value)
{
    ui->progressBar_7->setValue(value);
    PWM_CM_PID = qBound(1000, value, 2000);
}
void Dialog::on_btl_start_clicked()
{
    mThread->start(QThread::HighestPriority);
}

void Dialog::on_btl_send_clicked()
{
//FOR 1 PID:
    Kp = ui->lineEdit_Kp->text().toFloat();
    Ti = ui->lineEdit_Ti->text().toFloat();
    Td = ui->lineEdit_Td->text().toFloat();
    Tf = ui->lineEdit_Tf->text().toFloat();
    Des_Ang = ui->lineEdit_angle->text().toFloat();
//FOR 2 PID:
    Kp_ang = ui->lineEdit_Kp_ang->text().toFloat();
    Ki_ang = ui->lineEdit_Ki_ang->text().toFloat();
    Kd_ang = ui->lineEdit_Kd_ang->text().toFloat();
    Des_Ang_2 = ui->lineEdit_angle_2->text().toFloat();

    Kp_vel = ui->lineEdit_Kp_vel->text().toFloat();
    Ki_vel = ui->lineEdit_Ki_vel->text().toFloat();
    Kd_vel = ui->lineEdit_Kd_vel->text().toFloat();
//FOR 1 COMMON PID:
    Kpc = ui->lineEdit_Kpc->text().toFloat();
    Kic = ui->lineEdit_Kic->text().toFloat();
    Kdc = ui->lineEdit_Kdc->text().toFloat();
    Des_Ang_cm = ui->lineEdit_angle_c->text().toFloat();
}
