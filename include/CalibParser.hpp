#ifndef _CALIB_PARSER_HPP
#define _CALIB_PARSER_HPP

#include <stdlib.h>
#include <map>
#include <sstream>

#include <Eigen/Dense>

#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rapidxml_utils.hpp"


class CalibParser {
public:
typedef std::map<std::string, Eigen::Matrix4f> PoseMapT;
typedef std::pair<std::string, Eigen::Matrix4f> PosePairT;

public:
CalibParser()
{
}

CalibParser(const char *file_name)
{
	rapidxml::file<> xml_file(file_name);

	m_doc.parse<0>(xml_file.data());
	m_node_root = m_doc.first_node();

	rapidxml::xml_node<> *node_num = m_node_root->first_node("CAM_NUM");
	m_dev_num = atoi(node_num->value());

	ParseXML();
}

~CalibParser()
{
}

void ParseXML()
{
	m_pose_map.clear();
	m_sn_vec.clear();

	auto node_cam = m_node_root->first_node("CAM");

	for (int i = 0; i < m_dev_num; ++i) {
		auto node_cam_E = node_cam->first_node("E");
		auto node_cam_SN = node_cam->first_node("SN");

		std::string mat_str = node_cam_E->value();
		std::string sn_str = node_cam_SN->value();

		Eigen::Matrix4f pose_mat;
		ParseTransMat(mat_str, pose_mat);

		m_sn_vec.push_back(sn_str);
		m_pose_map.insert(PosePairT(sn_str, pose_mat));

		node_cam = node_cam->next_sibling("CAM");
	}
}

void ParseTransMat(const std::string mat_str, Eigen::Matrix4f &mat)
{
	std::istringstream i_str(mat_str);

	for (int i = 0; i < 4 * 4; ++i)
		i_str >> mat(i);
}

int getDevNum()
{
	return m_dev_num;
}


bool getDevPose(const std::string &sn_str, Eigen::Matrix4f &pose_mat)
{
	pose_mat = m_pose_map[sn_str];

	// if (pose_mat)
	return true;
}

void getDevSn(std::vector<std::string> &sn_vec)
{
	sn_vec = m_sn_vec;
}


private:
int m_dev_num;
std::vector<std::string> m_sn_vec;
PoseMapT m_pose_map;

rapidxml::xml_document<> m_doc;
rapidxml::xml_node<> *m_node_root;
};



#endif
