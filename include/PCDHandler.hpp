#include <pcl/io/pcd_io.h>

#include "CloudViewer.hpp"

class PCDHandler {
public:

  typedef pcl::PointXYZ                PointT;
  typedef pcl::PointCloud<PointT>      CloudT;
  typedef pcl::PointCloud<PointT>::Ptr CloudPtrT;

private:

  CloudT m_cloud;
  CloudPtrT m_cloud_ptr;

  CloudViewer m_cloud_viewer;

public:

  PCDHandler() : m_cloud_ptr(new CloudPtrT) {}

  ~PCDHandler() {
    delete m_cloud_ptr;
  }

  bool PCDRead(const std::string& file_name, CloudPtrT& cloud_ptr) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud_ptr) == -1) {
      return false;
    }
    else {
      m_cloud_ptr = cloud_ptr;
      m_cloud     = *m_cloud_ptr;

      return true;
    }
  }

  bool PCDWrite(const CloudT& cloud, const std::string& file_name) {
    if (pcl::io::savePCDFileASCII(file_name, cloud) < 0) {
      return false;
    }
    else {
      return true;
    }
  }

  bool PCDDisplay(const std::string& file_name) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *m_cloud_ptr) == -1) {
      return false;
    }
    else {
      m_cloud_viewer.show(m_cloud_ptr, file_name);

      return true;
    }
  }
};
