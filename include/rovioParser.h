/*
 * rovioParser.h
 *
 *  Created on: May 9, 2013
 *      Author: viki
 */

#ifndef ROVIOPARSER_H_
#define ROVIOPARSER_H_

#include <curl/curl.h>
#include <opencv2/opencv.hpp>
#include <string>

#define HOST "/rovio_base/host"
#define PORT "/rovio_base/port"
#define USER "/rovio_base/user"
#define PW "/rovio_base/pw"

//typedef struct
//{
//  char *data; /*!< the data returned from the server */
//  size_t size; /*!< the size of the data */
//} rovio_response;

typedef struct
{
  std::string cmd;
  int responses;
} rvCR, rvManualDrive;

typedef struct
{
  std::string cmd;
  uint length;
  int lDirection; // 1:forward, -1:back, 0:no move
  uint lNum;
  int rDirection; // 1:forward, -1:back, 0:no move
  uint rNum;
  int rearDirection; // -1:left, 1:right, 0:no move
  uint rearNum;
  uint headPosition;
  uchar batteryStatus;
  bool isLedOn;
  bool isIrOn;
  bool isDetectedBarrier;
  uchar chargerStatus;
} rvMCUReport;

class rovioParser
{
  static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp);

private:
  CURL* curl;
  void initCurl();
  rvCR writeDataCR(void *ptr, size_t size, size_t nmemb, void *stream);

public:
  std::string url;

public:
//  size_t writeData(char *ptr, size_t size, size_t nmemb, rovio_response *buf);
  rovioParser();
  rovioParser(std::string host, std::string port, std::string user, std::string pw);
  virtual ~rovioParser();
  void initParam(std::string host, std::string port, std::string user, std::string pw);

  rvManualDrive manualDrive(int dValue, int sValue);
  cv::Mat getImg();
  rvMCUReport getMCUReport();
  void printMCUReport(rvMCUReport report);
  void test();
};

#endif /* ROVIOPARSER_H_ */
