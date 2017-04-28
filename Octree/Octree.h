/*
 * Octree.h
 *
 *  Created on: 22.04.2017
 *      Author: andreask
 */

#ifndef OCTREE_H_
#define OCTREE_H_

#include <eigen2/Eigen/Dense>

#include "PosNode.h"
#include "IndexNode.h"
#include "Node.h"

template <typename Node, typename Patch>
struct Octree{
public:
	Octree(Eigen::Vector3d centre, double size){
		std::cout << centre.transpose() << "    " << size << std::endl;
		root = new Node(centre, size*10);
	}
	void addPatch(const Patch p){
		root->addPoint(p);
	}
	void addPatches(const std::vector<Patch>& p){
		for(auto& i:p){
			//std::cout << "Going to add " << i.p3D.transpose() << std::endl;
			if(i.size>0){ //AKK
				root->addPoint(i);
			}
		}
	}
	void propagate(){
		root->propagate();
	}
	void writeProb(){
		root->writeProb();
	}
private:
	Node* root;
};

#endif /* OCTREE_H_ */
