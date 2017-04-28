/*
 * main.cpp
 *
 *  Created on: 22.04.2017
 *      Author: andreask
 */

#include <eigen2/Eigen/Dense>

#include "Functors/BayesGauss.h"
#include "IO/PLYReader.h"
#include "Octree/Octree.h"
#include "Octree/PosNode.h"
#include "Octree/IndexNode.h"
#include "globals.h"

PLYWriter globals::plyWriter;
int globals::octreeNodes;
JET globals::jet;

struct Patch{
	Eigen::Vector3d p3D;
	Eigen::Vector3d normal;
	double size;
};

int main(){

	std::string filename = "dataset_01.ply";

	//Read ply file
	PLYReader<Patch> plyReader(filename);
	std::vector<Patch> patches;
	plyReader.read(patches);

	//Generate octree
	Octree<PosNode<Patch, BayesGauss>, Patch> octree(plyReader.getCentre(), plyReader.getMaxSpread());
	globals::octreeNodes = 0;
	octree.addPatches(patches);
	patches.clear();

	//Propagate octree
	octree.propagate();

	//Generate optimized octree
	//TODO

	//Filter optimized octree
	//TODO

	//Write file
	globals::plyWriter.init("dataset_02.ply", globals::octreeNodes);
	octree.writeProb();
	globals::plyWriter.close();

}


