#include "DepthDevice.hpp"
#include "CloudView.hpp"
#include "PCDHandler.hpp"


int main(int argc, char *argv[])
{
	DepthDevice depth_device({ RGB_VGA, DEPTH_VGA, POINT3D_VGA });

	depth_device.InitDevice();
	depth_device.OpenDevice();
	depth_device.UpdateDevPose("../../config/extrinsic.xml");


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
	CloudViewer cloud_viewer;
	PCDHandler pcd_handler;

	pcd_handler.PCDRead("../../data/2018_4_24_15_15_26.pcd", cloud_ptr);

	while(1){
		cloud_viewer.show(cloud_ptr, "cloud_show");
	}
//
//	while (1) {
//		depth_device.getConPointCloud(cloud_ptr);
//		cloud_viewer.show(cloud_ptr, "test");
//		pcd_handler.PCDWrite(cloud_ptr, "test.pcd");
//	}



	return 0;
}
