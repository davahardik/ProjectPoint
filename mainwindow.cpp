#include "mainwindow.h"
#include "ui_mainwindow.h"
#include<QFileDialog>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <fstream>
#include<QClipboard>
#include <QApplication>
#include<QFileDialog>
#include <iostream>
#include <fstream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <QCoreApplication>
#include <string>
#include <QDebug>
#include "opencv2/core/affine.hpp"
#include<QTextEdit>

using namespace std;


cv::Mat camera_matrix = cv::Mat(3,3,CV_64FC1);
cv::Mat dist_coeffs = cv::Mat(3,3,CV_64FC1);
cv::Mat proj_matrix= cv::Mat(3,3,CV_64FC1);
cv::Mat proj_dist_coeffs;
cv::Mat proTcamR= cv::Mat(3,3,CV_64FC1);
cv::Mat proTcamT = cv::Mat(3,1,CV_64FC1);

cv::Mat Cam1Tcam2T = cv::Mat(3,1,CV_64FC1);
cv::Mat C0C1T = cv::Mat(3,1,CV_64FC1);
cv::Mat C0C2T = cv::Mat(3,1,CV_64FC1);
cv::Mat C0C3T = cv::Mat(3,1,CV_64FC1);



QString calibrationfile1;
QString Pointfile1;

double X,Y,Z;
QString plyfile;


void splitPoints(const std::string& inputStr, std::vector<std::string>& splitVec)
{
    std::size_t pos = 0, found;
     while((found = inputStr.find_first_of(' ', pos)) != std::string::npos) {

        splitVec.push_back(inputStr.substr(pos, found - pos));
        pos = found+1;
     }

     splitVec.push_back(inputStr.substr(pos));
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;

}
using namespace pcl;
using namespace std;
using namespace cv;
class pickPoints {
public:

    pickPoints () {
        viewer.reset (new pcl::visualization::PCLVisualizer ("Viewer", true));
        viewer->registerPointPickingCallback (&pickPoints::pickCallback, *this);
    }

    ~pickPoints () {}

    void setInputCloud (PointCloud<PointXYZ>::Ptr source_cloud)
    {
        cloudTemp = source_cloud;
    }

    vector<float> getpoints() {
        return p;
    }

    void simpleViewer ()
    {
        // Visualizer
        viewer->addPointCloud<pcl::PointXYZ>(cloudTemp, "source_cloud");
        viewer->resetCameraViewpoint ("source_cloud");
        viewer->spin();
    }
protected:
    void pickCallback (const pcl::visualization::PointPickingEvent& event, void*)
    {
        if (event.getPointIndex () == -1)
            return;

        PointXYZ picked_point1,picked_point2;
        event.getPoints(picked_point1.x,picked_point1.y,picked_point1.z,
            picked_point2.x,picked_point2.y,picked_point2.z);
        p.push_back(picked_point1.x); // store points
        p.push_back(picked_point1.y);
        p.push_back(picked_point1.z);
        p.push_back(picked_point2.x);
        p.push_back(picked_point2.y);
        p.push_back(picked_point2.z);

        //cout<<"first selected point: "<<p[0]<<" "<<p[1]<<" "<<p[2]<<endl;
        //cout<<"second selected point: "<<p[3]<<" "<<p[4]<<" "<<p[5]<<endl;
    }

    std::vector<pcl::PointXYZ> selectedPoints;

private:

    PointCloud<pcl::PointXYZ>::Ptr cloudTemp;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    vector<float> p;
};


void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
    float x, y, z;
    if (event.getPointIndex() == -1)
    {
        return;
    }
    event.getPoint(x, y, z);
    X=x;
    Y=y;
    Z=z;

    std::cout << "( " << x << ", " << y << ", " << z << ")" << std::endl;
    ofstream outfile;
    outfile.open("points.txt", ios::out | ios::app);
    outfile << x << " " << y << " " << z << std::endl;
    outfile.close();

    //selectedPoints.push_back(pcl::PointXYZ(x, y, z));
}




void MainWindow::on_pushButton_clicked()
{
    plyfile=QFileDialog::getOpenFileName(this,tr("Open PLY File"), "",
                                                          tr("PointCloudFile (*.ply);;All Files (*)"));
    if(!plyfile.isEmpty())
        ui->lineEdit->setText(plyfile);

std::string Callibration = plyfile.toLocal8Bit().constData();

  std::vector<cv::Point3f> filename;
std::vector<int> filenames;

   //filenames = pcl::console::parse_file_extension_argument();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());

     pcl::io::loadPLYFile(Callibration, *source_cloud);
    pcl::visualization::PCLVisualizer viewer(" Point Cloud Datsets Visualizer");



    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Set background to a dark grey


    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(source_cloud);

    viewer.addPointCloud(source_cloud, rgb, "original_cloud");
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.registerPointPickingCallback (pointPickingEventOccurred, (void*)&viewer);
    //vector<float> pointSelected;
    //pointSelected= pickViewer.getpoints();
//ui->textEdit->setText(QString::number(x);)\;

        //cout<<pointSelected[0]<<" "<<pointSelected[1]<<" "<<pointSelected[2]<<endl;
        //cout<<pointSelected[3]<<" "<<pointSelected[4]<<" "<<pointSelected[5]<<endl;
    viewer.spin();


       // return 0;
}



void MainWindow::on_pushButton2_clicked()
{
    calibrationfile1=QFileDialog::getOpenFileName(this,tr("Open Calibration File"), "",
                                                          tr("YML (*.yml);;All Files (*)"));
    if(!calibrationfile1.isEmpty())
        ui->lineEdit2->setText(calibrationfile1);

   // QFile::copy( calibrationfile1,"/home/hd/Desktop/QtWork/build-PCL-Desktop_Qt_5_8_0_GCC_64bit-Debug/calibration.yml");
}

void MainWindow::on_pushButton3_clicked()
{
    Pointfile1=QFileDialog::getOpenFileName(this, tr("Open Point File"), "", tr("text (*.txt);;All Files (*)"));
      if(!Pointfile1.isEmpty())
          ui->lineEdit3->setText(Pointfile1);
}

void MainWindow::on_pushButton4_clicked()
{
std::string Callibration2 = calibrationfile1.toLocal8Bit().constData();

FileStorage fs(Callibration2, FileStorage::READ);


    fs["cam_K"] >> camera_matrix;
    fs["cam_kc"] >> dist_coeffs;
    fs["proj_K"] >> proj_matrix;
    fs["R"] >> proTcamR;
    fs["T"] >> proTcamT;
    fs["proj_kc"] >> proj_dist_coeffs;



    double d=25;


    std::ifstream pointsFile1;
    pointsFile1.open("/home/hd/Desktop/QtWork/build-PCL-Desktop_Qt_5_8_0_GCC_64bit-Debug/points.txt");

    std::string line;
    std::vector<std::string> tmpLineCont;
    std::vector<std::vector<float> > allPointsForProjecting;


    if (pointsFile1.is_open())
    {
      while ( std::getline (pointsFile1,line) )
      {
       tmpLineCont.clear();
       splitPoints(line, tmpLineCont);
       if ( tmpLineCont.size() == 3) {
          float strToDouble0 = std::atof(tmpLineCont[0].c_str());
          float strToDouble1 = std::atof(tmpLineCont[1].c_str());
          float strToDouble2 = std::atof(tmpLineCont[2].c_str());
          std::vector<float> tmpDoubleVec;
          tmpDoubleVec.push_back(strToDouble0);
          tmpDoubleVec.push_back(strToDouble1);
          tmpDoubleVec.push_back(strToDouble2);
          allPointsForProjecting.push_back(tmpDoubleVec);
       }
      }
      pointsFile1.close();
    }
    std::cout<<"Number of points: "<<allPointsForProjecting.size()<<std::endl;
    std::vector<cv::Point3f> pointCloudPointsVec;

    for (int i = 0; i < allPointsForProjecting.size(); ++i) {
          cv::Point3f tmpPoint;
          tmpPoint.x = allPointsForProjecting[i][0];
          tmpPoint.y = allPointsForProjecting[i][1];
          tmpPoint.z = allPointsForProjecting[i][2];
          pointCloudPointsVec.push_back(tmpPoint);
     }

    std::vector<cv::Point2f> img2dPoints(pointCloudPointsVec.size());
    std::vector<cv::Point> imagePixelPointsVec;


    cv::projectPoints(pointCloudPointsVec,proTcamR,proTcamT, proj_matrix, proj_dist_coeffs,img2dPoints);

    for (int i = 0; i < img2dPoints.size(); ++i) {
              int xPixelVal = img2dPoints[i].x  ;
              int yPixelVal = img2dPoints[i].y  ;
              cv::Point tmpPoint;
              tmpPoint.x = xPixelVal ;
              tmpPoint.y = yPixelVal ;
             imagePixelPointsVec.push_back(tmpPoint);
         }
    cv::Mat leftImage = cv::Mat(600,800,CV_8UC3,cv::Scalar(0,0,0));


    for (int i = 0; i < imagePixelPointsVec.size(); ++i){
      if ( i < imagePixelPointsVec.size()-1){
          int xPixelValA = imagePixelPointsVec[i].x  ;
          int yPixelValA = imagePixelPointsVec[i].y  ;
  cv::line(leftImage,imagePixelPointsVec[i],imagePixelPointsVec[i+1],cv::Scalar(0,255,255),6);


      }

   //   std::cout<<"Image point "<<imagePixelPointsVec[i].x<<"   "<<imagePixelPointsVec[i].y<<std::endl;

    }


    imagePixelPointsVec.clear(); //Clear temp points and replace with new points
    // Display image.

    cv::namedWindow("Projector1",CV_WINDOW_NORMAL);
    cv::setWindowProperty("Projector1", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
      cv::moveWindow("Projector1",1921,0);
    cv::imshow("Projector1", leftImage);
    cv::waitKey(1);

        }

void MainWindow::on_pushButton5_clicked()
{
    QApplication::quit();
}

