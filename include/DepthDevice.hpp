#ifndef _DEPTH_DEVICE_HPP
#define _DEPTH_DEVICE_HPP

#include <initializer_list>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include "TY_API.h"
#include "../common/common.hpp"

#include "CamData.hpp"
#include "CalibParser.hpp"

#define DEVMAXNUM 5
#define DEVELOPER_MODE 1


class DepthDevice {
public:

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT>      CloudT;
typedef pcl::PointCloud<PointT>::Ptr CloudPtrT;

public:

DepthDevice()
{
	// m_depth_opened = true;
	// m_depth_width = 640;
	// m_depth_height = 480;
	// m_depth_reso = TY_IMAGE_MODE_640x480;
	//
	// m_color_opened = true;
	// m_color_width = 640;
	// m_color_height = 480;
	// m_color_reso = TY_IMAGE_MODE_640x480;
	//
	// m_p3d_opened = true;
	// m_p3d_width = 640;
	// m_p3d_height = 480;
	// m_p3d_reso = TY_IMAGE_MODE_640x480;

	ParamInit();
}

DepthDevice(std::initializer_list<CAM_TYPE> list)
{
	ParamInit();

	for (auto it = list.begin(); it != list.end(); ++it) {
		switch (*it) {
		case RGB_VGA:
			m_color_opened = true;
			m_color_width = 640;
			m_color_height = 480;
			m_color_reso = TY_IMAGE_MODE_640x480;
			break;
		case RGB_HD:
			m_color_opened = true;
			m_color_width = 1280;
			m_color_height = 960;
			m_color_reso = TY_IMAGE_MODE_1280x960;
			break;
		case DEPTH_VGA:
			m_depth_opened = true;
			m_depth_width = 640;
			m_depth_height = 480;
			m_depth_reso = TY_IMAGE_MODE_640x480;
			break;
		case DEPTH_HD:
			m_depth_opened = true;
			m_depth_width = 1280;
			m_depth_height = 960;
			m_depth_reso = TY_IMAGE_MODE_1280x960;
			break;
		case POINT3D_VGA:
			m_p3d_opened = true;
			m_p3d_width = 640;
			m_p3d_height = 480;
			m_depth_reso = TY_IMAGE_MODE_640x480;
			break;
		case POINT3D_HD:
			m_p3d_opened = true;
			m_p3d_width = 1280;
			m_p3d_height = 960;
			m_p3d_reso = TY_IMAGE_MODE_1280x960;
			break;
		}
	}
}

void ParamInit()
{
	m_dev_num = 0;
	m_depth_opened = false;
	m_color_opened = false;
	m_p3d_opened = false;
}

~DepthDevice()
{
}

bool InitDevice()
{
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

int OpenDevice()
{
	for (int i = 0; i < m_dev_num; ++i) {
		strncpy(m_handle_data[i].sn, m_pBaseInfo[i].id,
			sizeof(m_handle_data[i].sn));

		// Open Device
		ASSERT_OK(TYOpenDevice(m_pBaseInfo[i].id, &(m_handle_data[i].hDev)));

		// Into develop mode
#ifdef DEVELOPER_MODE
		LOGD("=== Enter Developer Mode");
		ASSERT_OK(TYEnterDeveloperMode(m_handle_data[i].hDev));
#endif   // ifdef DEVELOPER_MODE

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

			// This doesnot work
			// LOGD("=== Configure feature, set resolution.");
			// int err = TYSetEnum(m_handle_data[i].hDev,
			//                     TY_COMPONENT_POINT3D_CAM,
			//                     TY_ENUM_IMAGE_MODE,
			//                     m_p3d_reso);
			// ASSERT(err == TY_STATUS_OK || err == TY_STATUS_NOT_PERMITTED);

			ideal_frame_size += m_p3d_width * m_p3d_height * 12;
		}

		// Allocate frame buffer
		int32_t frame_size;
		ASSERT_OK(TYGetFrameBufferSize(m_handle_data[i].hDev, &frame_size));
		LOGD("     - Get size of framebuffer, %d", frame_size);

		// TODO(CJH)
		// this judgement isnot correct
		// ASSERT(frame_size >= ideal_frame_size);
		ASSERT(frame_size >= 0);

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

	return m_dev_num;
}

bool UpdateDevPose(const std::string file_name)
{
	const char* ch = file_name.c_str();
//	CalibParser calib_parser(file_name);
	CalibParser calib_parser(ch);

	for (int i = 0; i < m_dev_num; ++i) {
		std::string sn_str = m_handle_data[i].sn;

		// Find transform matrix in xml file
		Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
		calib_parser.getDevPose(sn_str, mat);

		m_handle_data[i].dev_pose = mat;
	}

	return true;
}

bool getPointCloud(const std::string sn, CloudPtrT& cloud)
{
	cloud->clear();
	int dev_index = 0;

	for (int i = 0; i < m_dev_num; ++i, ++dev_index)
		if (sn == m_handle_data[i].sn)
			break;

	if (m_dev_num == dev_index)
		return false;

	TY_FRAME_DATA frame;
	cv::Mat p3d;

	int err = TYFetchFrame(m_handle_data[dev_index].hDev, &frame, 1000);
	if (TY_STATUS_OK != err)
		LOGD("cam %s %d ... Drop one frame", m_handle_data[dev_index].sn,
		     m_handle_data[dev_index].idx);

	parseFrame(frame, 0, 0, 0, 0, &p3d);
	if (!p3d.empty()) {
		m_handle_data[dev_index].point3d = p3d;
		GenPointCloud(p3d, cloud);

		return true;
	}

	return false;
}

bool getConPointCloud(CloudPtrT& cloud)
{
	cloud->clear();

	CloudPtrT trans_cloud(new CloudT());
	CloudPtrT raw_cloud(new CloudT());

	std::vector<cv::Mat> point3d_vec;
	FetchP3d();

	for (int i = 0; i < m_dev_num; ++i) {
		trans_cloud->clear();
		raw_cloud->clear();

		GenPointCloud(m_handle_data[i].point3d, raw_cloud);
		pcl::transformPointCloud(*raw_cloud, *trans_cloud, m_handle_data[i].dev_pose);

		*cloud += *trans_cloud;
	}

	return true;
}

bool FetchAll(std::vector<cv::Mat>&	color_vec,
	      std::vector<cv::Mat>&	depth_vec,
	      std::vector<cv::Mat>&	point3d_vec)
{
	color_vec.clear();
	depth_vec.clear();
	point3d_vec.clear();

	TY_FRAME_DATA frame;
	cv::Mat depth, color, p3d;

	for (int i = 0; i < m_dev_num; ++i) {
		int err = TYFetchFrame(m_handle_data[i].hDev, &frame, 1000);

		if (TY_STATUS_OK != err)
			LOGD("cam %s %d ... Drop one frame", m_handle_data[i].sn,
			     m_handle_data[i].idx);

		parseFrame(frame, &depth, 0, 0, &color, &p3d);

		if (!color.empty()) {
			m_handle_data[i].color = color;
			color_vec.push_back(color);
		}
		if (!depth.empty()) {
			m_handle_data[i].depth = depth;
			color_vec.push_back(depth);
		}
		if (!p3d.empty()) {
			m_handle_data[i].point3d = p3d;
			point3d_vec.push_back(p3d);
		}
	}
}

bool FetchRGB(cv::Mat& color)
{
}

bool FetchDepth(cv::Mat& depth)
{
}

bool FetchP3d(std::vector<cv::Mat>& point3d_vec)
{
	point3d_vec.clear();

	TY_FRAME_DATA frame;
	cv::Mat p3d;

	for (int i = 0; i < m_dev_num; ++i) {
		int err = TYFetchFrame(m_handle_data[i].hDev, &frame, 1000);

		if (TY_STATUS_OK != err)
			LOGD("cam %s %d ... Drop one frame", m_handle_data[i].sn,
			     m_handle_data[i].idx);

		parseFrame(frame, 0, 0, 0, 0, &p3d);

		if (!p3d.empty()) {
			m_handle_data[i].point3d = p3d;
			point3d_vec.push_back(p3d);
		}
	}

	return true;
}

bool FetchP3d()
{
	TY_FRAME_DATA frame;
	cv::Mat p3d;

	for (int i = 0; i < m_dev_num; ++i) {
		int err = TYFetchFrame(m_handle_data[i].hDev, &frame, 1000);

		if (TY_STATUS_OK != err)
			LOGD("cam %s %d ... Drop one frame", m_handle_data[i].sn,
			     m_handle_data[i].idx);

		parseFrame(frame, 0, 0, 0, 0, &p3d);

		if (!p3d.empty())
			m_handle_data[i].point3d = p3d;
	}

	return true;
}

void GenPointCloud(const cv::Mat img, CloudPtrT& cloud)
{
	cloud->clear();

	int n = img.rows * img.cols;
	float *data = (float *)img.data;

	for (int i = 0; i < n; ++i)
		cloud->push_back(PointT(data[i * 3 + 0], data[i * 3 + 1],
					data[i * 3 + 2]));
}

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
