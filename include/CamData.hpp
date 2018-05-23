#ifndef _CAMDATA_HPP
#define _CAMDATA_HPP


#include <Eigen/Dense>


struct CamInfo
{
  char          sn[32];
  TY_DEV_HANDLE hDev;
  char         *fb[2];
  TY_FRAME_DATA frame;
  int           idx;
  DepthRender   render;

  CamInfo() : hDev(0), idx(0) {
    fb[0] = 0; fb[1] = 0;
  }
};


struct HandleData {
  char          sn[32];
  TY_DEV_HANDLE hDev;
  char         *fb[2];
  TY_FRAME_DATA frame;

  // device pose
  Eigen::Matrix4f dev_pose;


  cv::Mat point3d;

  DepthRender *p_render;

  PointCloudViewer *p_pcviewer;

  int idx;
  HandleData() : hDev(0), idx(0), dev_pose(Eigen::Matrix4f::Identity()) {
    fb[0] = 0; fb[1] = 0;
  }
};


#endif // ifndef _CAMDATA_HPP
