/*
 * Node.h
 *
 *  Created on: 22.04.2017
 *      Author: andreask
 */

#ifndef OCTREE_NODE_H_
#define OCTREE_NODE_H_

#include <eigen2/Eigen/Dense>

template<typename Patch>
class Node{
public:
	Node(Node<Patch>* _parent){	}
	virtual ~Node() {}
	virtual void addPoint(const Patch p3D) = 0;
	bool filterNodesOnLine();
protected:
	inline bool getPlaneIntersection(const Eigen::Vector3d &n, const Eigen::Vector3d &p0,
			                         const Eigen::Vector3d &l0, const Eigen::Vector3d &l,
			                         double& distance) {
		// assuming vectors are all normalized
		double denom = n.dot(l);
		//std::cout << "denom: " << denom << std::endl;
		if (denom != 0) {
			Eigen::Vector3d p0l0 = p0 - l0;
			distance = p0l0.dot(n) / denom;
			//std::cout << "t: " << t << std::endl;
			return (distance >= 0);
			//return true;
		}
		return false;
	}
};


#endif /* OCTREE_NODE_H_ */
