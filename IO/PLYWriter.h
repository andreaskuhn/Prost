/*
 * PLYWriter.h
 *
 *  Created on: 26.04.2017
 *      Author: andreask
 */

#ifndef IO_PLYWRITER_H_
#define IO_PLYWRITER_H_

#include "rply/rply.h"

class PLYWriter{
public:
	PLYWriter(){}
	void init(std::string filename, int _numOfPoints){
		numOfPoints = _numOfPoints;
		currPointNumber = 0;
		ply = ply_create(filename.c_str(), PLY_ASCII, NULL, 0, NULL);
		ply_add_element(ply, "vertex", numOfPoints);
		ply_add_property(ply, "x", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
		ply_add_property(ply, "y", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
		ply_add_property(ply, "z", PLY_FLOAT, PLY_FLOAT, PLY_FLOAT);
		ply_add_property(ply, "red", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
		ply_add_property(ply, "green", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
		ply_add_property(ply, "blue", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
		ply_write_header(ply);
	}
	void iter(Eigen::Vector3d p3D, Eigen::Vector3i color){
		if(currPointNumber++<numOfPoints){
			ply_write(ply,(float)p3D[0]);
			ply_write(ply,(float)p3D[1]);
			ply_write(ply,(float)p3D[2]);
			ply_write(ply,(unsigned char)color[0]);
			ply_write(ply,(unsigned char)color[1]);
			ply_write(ply,(unsigned char)color[2]);
		}
	}
	void close(){
		ply_close(ply);
	}
private:
	p_ply ply;
	int numOfPoints;
	int currPointNumber;
};

#endif /* IO_PLYWRITER_H_ */
