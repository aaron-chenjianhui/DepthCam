#include "../common/common.hpp"


#include <pcl/common/transforms.h>

#include "CamData.hpp"
#include "CloudView.hpp"
#include "PCDHandler.hpp"
#include "FileHandler.hpp"


unsigned char buffer[1024 * 1024];
#define OPENRGB 1

// Choose depth image resolution
#define DEPTHRESO TY_IMAGE_MODE_640x480

// #define DEPTHRESO TY_IMAGE_MODE_1280x960

typedef pcl::PointXYZ                PointT;
typedef pcl::PointCloud<PointT>      PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;

void    StartDevice(HandleData              & cam,
                    const TY_DEVICE_BASE_INFO base_info);

void    FrameHandler(TY_FRAME_DATA *frame,
                     void          *userdata);

cv::Mat DepthToWorld(const cv::Mat& depth,
                     TY_DEV_HANDLE  hDevice);

void    GenPointCloud(cv::Mat        img,
                      PointCloudPtr& cloud_ptr);

bool    UpdateDevPose(std::vector<HandleData>& handle_data);

void    ConPoint3D(const std::vector<HandleData>& handle_data,
                   PointCloudPtr                & cloud_out);

int count = 0;

bool exit_main = false;
bool save_flag = false;

int main(int argc, char *argv[]) {
  // Device number
  int dev_num = 0;

  LOGD("=== Init lib");
  ASSERT_OK(TYInitLib());
  TY_VERSION_INFO *pVer = (TY_VERSION_INFO *)buffer;
  ASSERT_OK(TYLibVersion(pVer));
  LOGD("     - lib version: %d.%d.%d", pVer->major, pVer->minor, pVer->patch);

  LOGD("=== Get device info");
  TY_DEVICE_BASE_INFO *pBaseInfo = (TY_DEVICE_BASE_INFO *)buffer;
  ASSERT_OK(TYGetDeviceList(pBaseInfo, 100, &dev_num));

  if (dev_num < 1) {
    LOGD("=== Need more than 1 devices");
    return -1;
  }

  std::vector<HandleData> handle_data(dev_num);


  for (int i = 0; i < dev_num; ++i) {
    StartDevice(handle_data[i], pBaseInfo[i]);

    handle_data[i].p_render = new DepthRender();

    handle_data[i].p_pcviewer = new PointCloudViewer();
  }


  bool ret = UpdateDevPose(handle_data);

  if (!ret) {
    LOGD("Update device pose failed");
    return -1;
  }


  PointCloudPtr cloud_ptr(new PointCloud);
  CloudViewer   cloud_viewer;
  PCDHandler    pcd_handler;

  while (!exit_main) {
    for (int i = 0; i < handle_data.size(); i++) {
      TY_FRAME_DATA frame;
      int err = TYFetchFrame(handle_data[i].hDev, &frame, 1000);

      if (err != TY_STATUS_OK) {
        LOGD("cam %s %d ... Drop one frame", handle_data[i].sn,
             handle_data[i].idx);
        continue;
      }

      FrameHandler(&frame, &handle_data[i]);
    }

    // for display
    ConPoint3D(handle_data, cloud_ptr);
    cloud_viewer.show(cloud_ptr, "ConcatenatedCloud");

    cv::Mat depth_img = handle_data[0].depth;
    cv::Point center(320, 240);

    //    cv::Point center(640, 480);
    cv::circle(depth_img, center, 10, cv::Scalar(0, 0, 255), 3);

    char win[40];
    sprintf(win, "depth-%s", handle_data[0].sn);
    cv::imshow(win, depth_img);

    // cv::Mat img;1
    // cv::imshow("WaitKey", handle_data[0].point3d);


    if (true == save_flag) {
      std::string time_stamp      = getTimeStamp();
      std::string pcd_file_name   = time_stamp + ".pcd";
      std::string depth_file_name = time_stamp + "_depth.bmp";
      std::string color_file_name = time_stamp + "_color.bmp";

      pcd_handler.PCDWrite(*cloud_ptr, pcd_file_name);

      cv::imwrite(depth_file_name, handle_data[0].depth);
      cv::imwrite(color_file_name, handle_data[0].color);

      //    char file_name[64];
      //    sprintf(file_name, "%s.bmp", pData->sn);
      //    cv::imwrite(file_name, colorDepth);

      save_flag = false;
    }

    // use keyboard to control picture collect
    int key = cv::waitKey(1);

    switch (key & 0xff) {
    case 0xff:
      break;

    case 'q':
      exit_main = true;
      break;

    case 's':
      save_flag = true;
      break;

    default:
      LOGD("Unmapped key %d", key);
    }
  }


  // close device
  for (int i = 0; i < handle_data.size(); i++) {
    ASSERT_OK(TYStopCapture(handle_data[i].hDev));
    ASSERT_OK(TYCloseDevice(handle_data[i].hDev));

    // // MSLEEP(10); // sleep to ensure buffer is not used any more
    // delete handle_data[i].fb[0];
    // delete handle_data[i].fb[1];
    delete handle_data[i].p_pcviewer;
    delete handle_data[i].p_render;
  }
  ASSERT_OK(TYDeinitLib());

  LOGD("=== Main done!");
  return 0;
}

/**
 * @brief Start device by BaseInfo
 * @param [out] cam      CamInfo data, returned after start device
 * @param [in] BaseInfo  Device data, used to start device
 */
void StartDevice(HandleData              & cam,
                 const TY_DEVICE_BASE_INFO base_info) {
  strncpy(cam.sn, base_info.id, sizeof(cam.sn));

  // Open Device
  ASSERT_OK(TYOpenDevice(base_info.id, &(cam.hDev)));

  // Into develop mode
  #ifdef DEVELOPER_MODE
  LOGD("=== Enter Developer Mode");
  ASSERT_OK(TYEnterDeveloperMode(cam.hDev));
  #endif // ifdef DEVELOPER_MODE

  // handle component
  int32_t allComps;
  ASSERT_OK(TYGetComponentIDs(cam.hDev, &allComps));

  // whether to open RGB camera
  if (OPENRGB && allComps & TY_COMPONENT_RGB_CAM) {
    LOGD("=== Has RGB camera, open RGB cam");
    ASSERT_OK(TYEnableComponents(cam.hDev, TY_COMPONENT_RGB_CAM));
  }

  // open depth camera
  LOGD("=== Configure components, open depth cam");
  int32_t componentIDs = TY_COMPONENT_DEPTH_CAM;
  ASSERT_OK(TYEnableComponents(cam.hDev, componentIDs));

  LOGD("=== Configure components, open point3d cam");

  // int32_t componentIDs = TY_COMPONENT_POINT3D_CAM;
  ASSERT_OK(TYEnableComponents(cam.hDev, TY_COMPONENT_POINT3D_CAM));

  LOGD("=== Configure feature, set resolution to 640x480.");
  int err = TYSetEnum(cam.hDev,
                      TY_COMPONENT_DEPTH_CAM,
                      TY_ENUM_IMAGE_MODE,
                      DEPTHRESO);
  ASSERT(err == TY_STATUS_OK || err == TY_STATUS_NOT_PERMITTED);

  LOGD("=== Prepare image buffer");
  int32_t frameSize;
  ASSERT_OK(TYGetFrameBufferSize(cam.hDev, &frameSize));
  LOGD("     - Get size of framebuffer, %d", frameSize);
  ASSERT(frameSize >= 640 * 480 * 2);

  LOGD("     - Allocate & enqueue buffers");
  cam.fb[0] = new char[frameSize];
  cam.fb[1] = new char[frameSize];
  LOGD("     - Enqueue buffer (%p, %d)", cam.fb[0], frameSize);
  ASSERT_OK(TYEnqueueBuffer(cam.hDev, cam.fb[0], frameSize));
  LOGD("     - Enqueue buffer (%p, %d)", cam.fb[1], frameSize);
  ASSERT_OK(TYEnqueueBuffer(cam.hDev, cam.fb[1], frameSize));

  bool triggerMode = false;
  LOGD("=== Set trigger mode %d", triggerMode);
  ASSERT_OK(TYSetBool(cam.hDev, TY_COMPONENT_DEVICE, TY_BOOL_TRIGGER_MODE,
                      triggerMode));

  LOGD("=== Start capture");
  ASSERT_OK(TYStartCapture(cam.hDev));
}

void FrameHandler(TY_FRAME_DATA *frame,
                  void          *userdata) {
  HandleData *pData = (HandleData *)userdata;

  count++;
  LOGD("count is: %d", count);

  //
  cv::Mat depth, irl, irr, color, p3d;
  cv::Mat point;


  parseFrame(*frame, &depth, 0, 0, &color, &p3d);

  char win[64];

  if (!color.empty()){
    pData->color = color;
  }

  if (!depth.empty()) {
    cv::Mat colorDepth = pData->p_render->Compute(depth);
      pData->depth = colorDepth;

//    cv::Point center(320, 240);
//
//    //    cv::Point center(640, 480);
//    cv::circle(colorDepth, center, 10, cv::Scalar(0, 0, 255), 3);
//
//    sprintf(win, "depth-%s", pData->sn);
//    cv::imshow(win, colorDepth);

    //    char file_name[64];
    //    sprintf(file_name, "%s.bmp", pData->sn);
    //    cv::imwrite(file_name, colorDepth);

    //      point = DepthToWorld(depth, pData->hDev);
    //
    //      pData->p_pcviewer->show(p3d, "Point3D");
    //
    //      if (pData->p_pcviewer->isStopped("Point3D")) {
    //          exit_main = true;
    //        return;
    //      }
  }

  if (!p3d.empty()) {
    // Save 3d point image
    pData->point3d = p3d;

    //
    //    sprintf(win, "Point3D-%s", pData->sn);
    //    pData->p_pcviewer->show(p3d, win);
    //
    //    if (pData->p_pcviewer->isStopped(win)) {
    //      exit_main = true;
    //      return;
    //    }
  }

  // if (!irl.empty()) {
  //   sprintf(win, "LeftIR-%s", pData->sn);
  //   cv::imshow(win, irl);
  // }
  //
  // if (!irr.empty()) {
  //   sprintf(win, "RightIR-%s", pData->sn);
  //   cv::imshow(win, irr);
  // }
  //
  // if (!color.empty()) {
  //   sprintf(win, "color-%s", pData->sn);
  //   cv::imshow(win, color);
  // }

  pData->idx++;


  LOGD("=== Callback: Re-enqueue buffer(%p, %d)",
       frame->userBuffer,
       frame->bufferSize);

  ASSERT_OK(TYEnqueueBuffer(pData->hDev, frame->userBuffer,
                            frame->bufferSize));
}

cv::Mat DepthToWorld(const cv::Mat& depth,
                     TY_DEV_HANDLE  hDevice) {
  // conver depth to world
  //  static TY_VECT_3F depthbuf[1280 * 960];
  //  static TY_VECT_3F worldbuf[1280 * 960];
  static TY_VECT_3F depthbuf[640 * 480];
  static TY_VECT_3F worldbuf[640 * 480];

  int k            = 0;
  uint16_t *pdepth = (uint16_t *)depth.data;

  for (int r = 0; r < depth.rows; r++)
    for (int c = 0; c < depth.cols; c++) {
      depthbuf[k].x = c;
      depthbuf[k].y = r;
      depthbuf[k].z = pdepth[k];
      k++;
    }
  ASSERT_OK(TYDepthToWorld(hDevice, depthbuf, worldbuf, 0,
                           depth.rows * depth.cols));

  // show point3d
  cv::Mat point3D(depth.rows,
                  depth.cols,
                  CV_32FC3,
                  (void *)worldbuf);
  return point3D;
}

bool UpdateDevPose(std::vector<HandleData>& handle_data) {
  for (int i = 0; i < handle_data.size(); ++i) {
    char *ch = handle_data[i].sn;

    // if (0 == strcmp("207000002038", handle_data[i].sn)) {
    //   handle_data[i].dev_pose << 1, 0, 0, 0,
    //           0, 1, 0, 0,
    //           0, 0, 1, 0,
    //           0, 0, 0, 1;
    // } else {
    //   handle_data[i].dev_pose << 1, 0, 0, 720,
    //           0, 1, 0, 0,
    //           0, 0, 1, 0,
    //           0, 0, 0, 1;
    // }
    if (0 == strcmp("207000001670", handle_data[i].sn)) {
      handle_data[i].dev_pose = Eigen::Matrix4f::Identity();
    }
    else if (0 == strcmp("207000001678", handle_data[i].sn)) {
      Eigen::Matrix4f trans_mat;

      // trans_mat << 0.990809, 0.0197867, -0.133816, 1056.8,
      //   -0.0163958, 0.999517, 0.0263955, 7.05489,
      //   0.134274, -0.0239589, 0.990655, 62.7959,
      //   0, 0, 0, 1;
      trans_mat << 0.975401, 0.0319171, -0.218116, 1067.38,
        -0.0257499, 0.999186, 0.0310601, -0.814528,
        0.218929, -0.0246796, 0.975429, 66.6778,
        0, 0, 0, 1;

      handle_data[i].dev_pose = trans_mat.inverse();
    }
  }
  return true;
}

void GenPointCloud(cv::Mat img, PointCloudPtr& cloud_ptr) {
  cloud_ptr->clear();

  int n       = img.rows * img.cols;
  float *data = (float *)img.data;

  for (int i = 0; i < n; ++i) {
    cloud_ptr->push_back(PointT(data[i * 3 + 0], data[i * 3 + 1],
                                data[i * 3 + 2]));
  }

  float x = img.at<cv::Vec3f>(240, 320)[0];
  float y = img.at<cv::Vec3f>(240, 320)[1];
  float z = img.at<cv::Vec3f>(240, 320)[2];

  //  float x = img.at<cv::Vec3f>(480, 640)[0];
  //  float y = img.at<cv::Vec3f>(480, 640)[1];
  //  float z = img.at<cv::Vec3f>(480, 640)[2];


  std::cout << "the center point is: ( " << x << ", " << y << ", " << z << " )" <<
    std::endl;

  //  std::cout << "The depth img center point is: " << center_point <<
  // std::endl;
}

void ConPoint3D(const std::vector<HandleData>& handle_data,
                PointCloudPtr                & cloud_out) {
  cloud_out->clear();

  PointCloudPtr trans_cloud(new PointCloud);

  for (int i = 0; i < handle_data.size(); ++i) {
    trans_cloud->clear();

    PointCloudPtr cloud_ptr(new PointCloud);
    GenPointCloud(handle_data[i].point3d, cloud_ptr);
    pcl::transformPointCloud(*cloud_ptr, *trans_cloud, handle_data[i].dev_pose);

    for (int j = 0; j < trans_cloud->size(); ++j) {
      cloud_out->push_back(trans_cloud->at(j));
    }
  }
}

void SaveCloud() {}

// void ConP3d(const HandleData                   & handle_data,
//            pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
//  cloud->resize(0);
//
//  cv::Mat p3d = handle_data.point3d;
//
//  int n       = p3d.rows * p3d.cols;
//  float *data = p3d.data;
//
//  for (int j = 0; j < n; ++j) {
//    cloud->push_back(pcl::PointXYZ(data[i * 3 + 0], data[i * 3 + 1],
//                                   data[i * 3 + 2]));
//  }
// }
