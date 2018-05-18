#ifndef _CAMDATA_HPP
#define _CAMDATA_HPP

#define HAVE_PCL 1

#include <Eigen/Dense>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

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

  DepthRender      *p_render;
  PointCloudViewer *p_pcviewer;

  int idx;
  HandleData() : hDev(0), idx(0), dev_pose(Eigen::Matrix4f::Identity()) {
    fb[0] = 0; fb[1] = 0;
  }
};


static const std::string helpText[] = {
        "Left Button + Slide  Left/Right",
        "Left Button + Slide  Up/Down",
        "Left Button + CTRL + Slide Left/Right",
        "Left Button + SHIFT",
        "Mouse Wheel Up/Down",
        "Mouse Wheel PressDown"
};
static int helpTextSize = 11;
static int WinWidth = 640;
static int WinHeight = 480;
static void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);
    viewer.resetCamera();
    //viewer.addCoordinateSystem(1.0);
    viewer.setPosition(0, 0);
    viewer.setSize(WinWidth, WinHeight);

    for (size_t i=0; i<sizeof(helpText)/sizeof(helpText[0]); i++)
        viewer.addText(helpText[i], 3, WinHeight-(i+1)*helpTextSize, helpTextSize, 1.0, 1.0, 1.0);
}


typedef pcl::PointXYZ                PointT;
typedef pcl::PointCloud<PointT>      PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;

class PointCloudViewerImpl {
public:
    void show(const PointCloudPtr &cloud, const std::string &windowName)
    {
#ifdef HAVE_PCL

        std::map<std::string, pcl::visualization::CloudViewer*>::iterator it = m_viewerMap.find(windowName);

        bool reset_view = false;
        if(m_viewerMap.end() == it){
            pcl::visualization::CloudViewer* viewer = new pcl::visualization::CloudViewer(windowName);
            std::pair<std::map<std::string, pcl::visualization::CloudViewer*>::iterator, bool> ret;
            ret = m_viewerMap.insert(std::pair<std::string, pcl::visualization::CloudViewer*>(windowName, viewer));
            if(!ret.second){
                // LOGE("pcshow: insert viewer %s failed.\n", windowName.c_str());
                return;
            }
            it = ret.first;
            reset_view = true;
        }

        // PCL display
        it->second->showCloud(cloud);
        if(reset_view){
            it->second->runOnVisualizationThreadOnce(viewerOneOff);
        }
#endif // HAVE_PCL
    }


    bool isStopped(const std::string &windowName)
    {
        bool ret = true;
#ifdef HAVE_PCL
        std::map<std::string, pcl::visualization::CloudViewer*>::iterator it = m_viewerMap.find(windowName);
        ret = it->second->wasStopped(0);
#endif // HAVE_PCL
        return ret;
    }


#ifdef HAVE_PCL
    void genPointCloudXYZFromVec3f(const float* data, int n, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        cloud->resize(0);
        for(int i = 0; i < n; i++){
            cloud->push_back(pcl::PointXYZ(data[i*3+0], data[i*3+1], data[i*3+2]));
        }
    }

private:
    std::map<std::string, pcl::visualization::CloudViewer*> m_viewerMap;
#endif // HAVE_PCL

};


#endif // ifndef _CAMDATA_HPP
