#ifndef _DEPTH_DEVICE_HPP
#define _DEPTH_DEVICE_HPP

#include "../common/common.hpp"

#include "CamData.hpp"

#define DEVMAXNUM 5
#define DEVELOPER_MODE 1


class DepthDevice {
public:

  typedef pcl::PointXYZ                PointT;
  typedef pcl::PointCloud<PointT>      CloudT;
  typedef pcl::PointCloud<PointT>::Ptr CloudPtrT;

public:

  DepthDevice() : m_dev_num(0),
    m_depth_reso(TY_IMAGE_MODE_640x480),
    m_color_reso(TY_IMAGE_MODE_640x480),
    m_p3d_reso(TY_IMAGE_MODE_640x480) {
    m_depth_opened = true;
    m_color_opened = true;
    m_p3d_opened   = true;

    if (m_depth_opened) {
      m_depth_width  = 640;
      m_depth_height = 480;
      m_depth_reso   = TY_IMAGE_MODE_640x480;
    }

    if (m_color_opened) {
      m_color_width  = 640;
      m_color_height = 480;
      m_color_reso   = TY_IMAGE_MODE_640x480;
    }

    if (m_p3d_opened) {
      m_p3d_width  = 640;
      m_p3d_height = 480;
      m_p3d_reso   = TY_IMAGE_MODE_640x480;
    }
  }

  ~DepthDevice() {}

  bool InitDevice()                                               {
    LOGD("=== Init lib");
    ASSERT_OK(TYInitLib());

    TY_VERSION_INFO *pVer = (TY_VERSION_INFO *)buffer;
    ASSERT_OK(TYLibVersion(pVer));
    LOGD("     - lib version: %d.%d.%d", pVer->major, pVer->minor, pVer->patch);

    LOGD("=== Get device info");
    m_pBaseInfo = (TY_DEVICE_BASE_INFO *)buffer;
    ASSERT_OK(TYGetDeviceList(m_pBaseInfo, 100, &m_dev_num));

    if (m_dev_num < 1) {
      LOGD("=== Need more than 1 devices");
      return false;
    }

    m_handle_data.resize(m_dev_num);

    return true;
  }

  int OpenDevice()                                               {
    for (int i = 0; i < m_dev_num; ++i) {
      strncpy(m_handle_data[i].sn, m_pBaseInfo[i].id,
              sizeof(m_handle_data[i].sn));

      // Open Device
      ASSERT_OK(TYOpenDevice(m_pBaseInfo[i].id, &(m_handle_data[i].hDev)));

      // Into develop mode
  #ifdef DEVELOPER_MODE
      LOGD("=== Enter Developer Mode");
      ASSERT_OK(TYEnterDeveloperMode(m_handle_data[i].hDev));
  #endif // ifdef DEVELOPER_MODE

      uint32_t ideal_frame_size = 0;

      // whether to oepn RGB camera
      if (true == m_color_opened) {
        LOGD("=== Configure components, open depth cam");
        ASSERT_OK(TYEnableComponents(m_handle_data[i].hDev,
                                     TY_COMPONENT_RGB_CAM));

        LOGD("=== Configure feature, set resolution.");
        int err = TYSetEnum(m_handle_data[i].hDev,
                            TY_COMPONENT_RGB_CAM,
                            TY_ENUM_IMAGE_MODE,
                            m_color_reso);
        ASSERT(err == TY_STATUS_OK || err == TY_STATUS_NOT_PERMITTED);

        ideal_frame_size += m_color_width * m_color_height * 3;
      }

      // whether to oepn depth camera
      if (true == m_depth_opened) {
        LOGD("=== Configure components, open depth cam");
        ASSERT_OK(TYEnableComponents(m_handle_data[i].hDev,
                                     TY_COMPONENT_DEPTH_CAM));

        LOGD("=== Configure feature, set resolution.");
        int err = TYSetEnum(m_handle_data[i].hDev,
                            TY_COMPONENT_DEPTH_CAM,
                            TY_ENUM_IMAGE_MODE,
                            m_depth_reso);
        ASSERT(err == TY_STATUS_OK || err == TY_STATUS_NOT_PERMITTED);

        ideal_frame_size += m_depth_width * m_depth_height * 2;
      }

      // whether to open Point3d camera
      if (true == m_p3d_opened) {
        LOGD("=== Configure components, open point3d cam");
        ASSERT_OK(TYEnableComponents(m_handle_data[i].hDev,
                                     TY_COMPONENT_POINT3D_CAM));

        LOGD("=== Configure feature, set resolution.");
        int err = TYSetEnum(m_handle_data[i].hDev,
                            TY_COMPONENT_POINT3D_CAM,
                            TY_ENUM_IMAGE_MODE,
                            m_p3d_reso);
        ASSERT(err == TY_STATUS_OK || err == TY_STATUS_NOT_PERMITTED);

        ideal_frame_size += m_p3d_width * m_p3d_height * 12;
      }

      // Allocate frame buffer
      int32_t frame_size;
      ASSERT_OK(TYGetFrameBufferSize(m_handle_data[i].hDev, &frame_size));
      LOGD("     - Get size of framebuffer, %d", frame_size);
      ASSERT(frame_size >= ideal_frame_size);

      LOGD("     - Allocate & enqueue buffers");
      m_handle_data[i].fb[0] = new char[frame_size];
      m_handle_data[i].fb[1] = new char[frame_size];
      LOGD("     - Enqueue buffer (%p, %d)", m_handle_data[i].fb[0], frame_size);
      ASSERT_OK(TYEnqueueBuffer(m_handle_data[i].hDev, m_handle_data[i].fb[0],
                                frame_size));
      LOGD("     - Enqueue buffer (%p, %d)", m_handle_data[i].fb[1], frame_size);
      ASSERT_OK(TYEnqueueBuffer(m_handle_data[i].hDev, m_handle_data[i].fb[1],
                                frame_size));

      //
      bool triggerMode = false;
      LOGD("=== Set trigger mode %d", triggerMode);
      ASSERT_OK(TYSetBool(m_handle_data[i].hDev, TY_COMPONENT_DEVICE,
                          TY_BOOL_TRIGGER_MODE,
                          triggerMode));

      LOGD("=== Start capture");
      ASSERT_OK(TYStartCapture(m_handle_data[i].hDev));
    }
  }

  bool UpdateDevPose()                                            {}

  bool getPointCloud(const std::string sn, CloudT& cloud)         {}

  bool getConPointCloud(CloudT& cloud)                            {}

  bool FetchAll(cv::Mat& color, cv::Mat& depth, cv::Mat& point3d) {}

  bool FetchRGB(cv::Mat& color)                                   {}

  bool FetchDepth(cv::Mat& depth)                                 {}

  bool FetchP3d(cv::Mat& point3d)                                 {}

private:

  int m_dev_num;

  TY_DEVICE_BASE_INFO *m_pBaseInfo;
  std::vector<HandleData>m_handle_data;

  unsigned char buffer[sizeof(TY_DEVICE_BASE_INFO) * DEVMAXNUM];

  bool m_depth_opened;
  TY_IMAGE_MODE_LIST m_depth_reso;
  uint16_t m_depth_width;
  uint16_t m_depth_height;
  bool m_color_opened;
  TY_IMAGE_MODE_LIST m_color_reso;
  uint16_t m_color_width;
  uint16_t m_color_height;
  bool m_p3d_opened;
  TY_IMAGE_MODE_LIST m_p3d_reso;
  uint16_t m_p3d_width;
  uint16_t m_p3d_height;
};

#endif // ifndef _DEPTH_DEVICE_HPP
