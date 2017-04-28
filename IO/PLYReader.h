/*
 * PLYReader.h
 *
 *  Created on: 26.04.2017
 *      Author: andreask
 */

#ifndef IO_PLYREADER_H_
#define IO_PLYREADER_H_

#include <iostream>
#include <vector>

#include <eigen2/Eigen/Dense>

#include "rply/rply.h"

static int runIndex;
static int lastIndex;
static double* data;
static double* static_bbMin;
static double* static_bbMax;

static int vertex_read(p_ply_argument argument){
	long index;
	void* pdata;
	ply_get_argument_user_data(argument, &pdata, &index);
	switch(index){
	case 0:
		data[(lastIndex+1)*runIndex]=(double)ply_get_argument_value(argument);
		break;
	case 1:
		data[(lastIndex+1)*runIndex+1]=(double)ply_get_argument_value(argument);
		break;
	case 2:
		data[(lastIndex+1)*runIndex+2]=(double)ply_get_argument_value(argument);
		break;
	case 3:
		data[(lastIndex+1)*runIndex+3]=(double)ply_get_argument_value(argument);
		break;
	case 4:
		data[(lastIndex+1)*runIndex+4]=(double)ply_get_argument_value(argument);
		break;
	case 5:
		data[(lastIndex+1)*runIndex+5]=(double)ply_get_argument_value(argument);
		break;
	case 6:
		data[(lastIndex+1)*runIndex+6]=(double)ply_get_argument_value(argument);
		//std::cout << (double)ply_get_argument_value(argument) << std::endl;
		break;
	}
	if(index<=2){
		if(static_bbMin[index] > data[(lastIndex+1)*runIndex+index])
			static_bbMin[index] = data[(lastIndex+1)*runIndex+index];
		if(static_bbMax[index] < data[(lastIndex+1)*runIndex+index])
			static_bbMax[index] = data[(lastIndex+1)*runIndex+index];
	}
	if(index == lastIndex)
		runIndex++;
	return 1;
}

template<typename Patch>
class PLYReader{
public:
	PLYReader(std::string filename){
		bbMin << std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max();
		bbMax << -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max();
		std::cout << "Reading file " << filename << std::endl;
		ply = ply_open(filename.c_str(), NULL, 0, NULL);
		if(!ply)
			std::cout << "File not found" << std::endl;
		ply_read_header(ply);
	}

	void read(std::vector<Patch>& patches){
		int numOfVertices = ply_set_read_cb(ply, "vertex", "x", vertex_read, NULL, 0);
		ply_set_read_cb(ply, "vertex", "y", vertex_read, NULL, 1);
		ply_set_read_cb(ply, "vertex", "z", vertex_read, NULL, 2);
		int numOfDirections = ply_set_read_cb(ply, "vertex", "nx", vertex_read, NULL, 3);
		ply_set_read_cb(ply, "vertex", "ny", vertex_read, NULL, 4);
		ply_set_read_cb(ply, "vertex", "nz", vertex_read, NULL, 5);
		int numOfSizes = ply_set_read_cb(ply, "vertex", "value", vertex_read, NULL, 6);
		lastIndex=6;
		std::cout << "Going to read " << numOfVertices << " patches" << std::endl;
		patches.resize(numOfVertices);
		data = (double*)&patches[0];
		static_bbMin = &bbMin[0];
		static_bbMax = &bbMax[0];
		ply_read(ply);
		for(auto i:patches){
			std::cout << std::endl;
			std::cout << i.p3D.transpose() << std::endl;
			std::cout << i.normal.transpose() << std::endl;
			std::cout << i.size << std::endl;
			break;
		}
		ply_close(ply);
	}

	double getMaxSpread(){
		return std::max(bbMax[0]-bbMin[0], std::max(bbMax[1]-bbMin[1], bbMax[2]-bbMin[2]));
	}

	Eigen::Vector3d getCentre(){
		return ((bbMax-bbMin) / 2.0);
	}

private:
	p_ply ply;
	Eigen::Vector3d bbMin, bbMax;
};

#endif /* IO_PLYREADER_H_ */
