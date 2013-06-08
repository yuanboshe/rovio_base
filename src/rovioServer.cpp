#include <ros/ros.h>
#include "rovioParser.h"
#include "rovio_base/manDrv.h"
#include "rovio_base/image.h"
#include "rovio_base/report.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

rovioParser parser;

bool control(rovio_base::manDrv::Request &req, rovio_base::manDrv::Response &res)
{
  rvManualDrive rv = parser.manualDrive(req.drive, req.speed);
  res.code = rv.responses;
  ROS_INFO("Rovio control message: drive=%d, speed=%d, code=%d", req.drive, req.speed, res.code);
  return true;
}

bool getImage(rovio_base::image::Request &req, rovio_base::image::Response &res)
{
  cv_bridge::CvImage cvImg(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, parser.getImg());
  cvImg.toImageMsg(res.img);
  ROS_INFO("Rovio image Responsed!");
  return true;
}

bool getReport(rovio_base::report::Request &req, rovio_base::report::Response &res)
{
  rvMCUReport rv = parser.getMCUReport();
  res.length = rv.length;
  res.lDirection = rv.lDirection;
  res.lNum = rv.lNum;
  res.rDirection = rv.rDirection;
  res.rNum = rv.rNum;
  res.rearDirection = rv.rearDirection;
  res.rearNum = rv.rearNum;
  res.headPosition = rv.headPosition;
  res.isLedOn = rv.isLedOn;
  res.isIrOn = rv.isIrOn;
  res.isDetectedBarrier = rv.isDetectedBarrier;
  ROS_INFO("Rovio MCU report Responsed!");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rovioControl");
  ros::NodeHandle node;

  string host, port, user, pw;
  if (!node.getParam(HOST, host))
  {
    ROS_ERROR("Parameter %s not found.", "host");
    exit(-1);
  }
  if (!node.getParam(PORT, port))
  {
    ROS_ERROR("Parameter %s not found.", "port");
    exit(-1);
  }
  if (!node.getParam(USER, user))
  {
    ROS_ERROR("Parameter %s not found.", "user");
    exit(-1);
  }
  if (!node.getParam(PW, pw))
  {
    ROS_ERROR("Parameter %s not found.", "pw");
    exit(-1);
  }
  parser.initParam(host, port, user, pw);

  ros::ServiceServer crlService = node.advertiseService("rovioControl", control);
  ros::ServiceServer imgService = node.advertiseService("rovioImage", getImage);
  ros::ServiceServer reportService = node.advertiseService("rovioReport", getImage);
  ROS_INFO("Rovio server ON...");
  ros::spin();

  return 0;
}
