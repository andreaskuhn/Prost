/*
 * PosNode.h
 *
 *  Created on: 22.04.2017
 *      Author: andreask
 */

#ifndef OCTREE_POSNODE_H_
#define OCTREE_POSNODE_H_

#define debug

#include <vector>

#include <chrono>
#include <thread>

#include "Node.h"
#include "../globals.h"

template<typename Patch, typename Functor>
class PosNode : Node<Patch> {

public:

	PosNode(PosNode<Patch, Functor>* _parent, Eigen::Vector3d _centre) : Node<Patch>((Node<Patch>*)_parent){
		centre = _centre;
		size=_parent->size*0.5;
		parent = _parent;
		ulf = urf = ulb = urb = llf = lrf = llb = lrb = NULL;
	}

	PosNode(Eigen::Vector3d _centre, double _size) : Node<Patch>(NULL){
		this->parent = NULL;
		centre = _centre;
		size=_size;
		parent = NULL;
		ulf = urf = ulb = urb = llf = lrf = llb = lrb = NULL;
	}

	~PosNode(){}

	void addPoint(const Patch p) {
		if(size < p.size){
#ifdef debug
			if(p.p3D[0]<centre[0]-size*0.5 || p.p3D[1]<centre[1]-size*0.5 || p.p3D[2]<centre[2]-size*0.5
			 ||p.p3D[0]>centre[0]+size*0.5 || p.p3D[1]>centre[1]+size*0.5 || p.p3D[2]>centre[2]+size*0.5)
				std::cout << "Error" << std::endl;
#endif
			patches.push_back(p);
		}
		else{
			PosNode<Patch, Functor>* child = NULL;
			int childID = getChild(child, p.p3D);
			if(child==NULL)
				child = createChild(childID);
			child->addPoint(p);
		}
	}

	PosNode<Patch, Functor>* createChild(int childID){
		PosNode<Patch, Functor>* child;
		Eigen::Vector3d childCentre = centre;
		const double quartersize = size*0.25;
		switch(childID){
		case 1:
				childCentre[0]+=quartersize; childCentre[1]+=quartersize; childCentre[2]+=quartersize;
				this->urb = new PosNode<Patch, Functor>(this, childCentre);
				child = (PosNode<Patch, Functor>*)this->urb;
				break;
		case 2:
				childCentre[0]+=quartersize; childCentre[1]+=quartersize; childCentre[2]-=quartersize;
				this->urf = new PosNode<Patch, Functor>(this, childCentre);
				child = (PosNode<Patch, Functor>*)this->urf;
				break;
		case 3:
				childCentre[0]+=quartersize; childCentre[1]-=quartersize; childCentre[2]+=quartersize;
				this->lrb = new PosNode<Patch, Functor>(this, childCentre);
				child = (PosNode<Patch, Functor>*)this->lrb;
				break;
		case 4:
				childCentre[0]+=quartersize; childCentre[1]-=quartersize; childCentre[2]-=quartersize;
				this->lrf = new PosNode<Patch, Functor>(this, childCentre);
				child = (PosNode<Patch, Functor>*)this->lrf;
				break;
		case 5:
				childCentre[0]-=quartersize; childCentre[1]+=quartersize; childCentre[2]+=quartersize;
				this->ulb = new PosNode<Patch, Functor>(this, childCentre);
				child = (PosNode<Patch, Functor>*)this->ulb;
				break;
		case 6:
				childCentre[0]-=quartersize; childCentre[1]+=quartersize; childCentre[2]-=quartersize;
				this->ulf = new PosNode<Patch, Functor>(this, childCentre);
				child = (PosNode<Patch, Functor>*)this->ulf;
				break;
		case 7:
				childCentre[0]-=quartersize; childCentre[1]-=quartersize; childCentre[2]+=quartersize;
				this->llb = new PosNode<Patch, Functor>(this, childCentre);
				child = (PosNode<Patch, Functor>*)this->llb;
				break;
		case 8:
				childCentre[0]-=quartersize; childCentre[1]-=quartersize; childCentre[2]-=quartersize;
				this->llf = new PosNode<Patch, Functor>(this, childCentre);
				child = (PosNode<Patch, Functor>*)this->llf;
				break;
		}
		globals::octreeNodes++;
		return child;
	}

	void propagate(){
		if(this->ulf)
			this->ulf->propagate();
		if(this->urf)
			this->urf->propagate();
		if(this->ulb)
			this->ulb->propagate();
		if(this->urb)
			this->urb->propagate();
		if(this->llf)
			this->llf->propagate();
		if(this->lrf)
			this->lrf->propagate();
		if(this->llb)
			this->llb->propagate();
		if(this->lrb)
			this->lrb->propagate();
		for(auto p:patches){
			double overalldistance = 0;
			PosNode* currNode = this;
			Eigen::Vector3d intersPoint = p.p3D;
			//std::cout << std::endl;
			double siqsqrt = sqrt(2.0*p.size*p.size);
			while(overalldistance < p.size*2){
				//std::cout << "overall: " << overalldistance << std::endl;
				//std::cout << currNode->centre.transpose() << std::endl;
				double distance = 0;
				PosNode* tempNode = currNode->getNextNode(intersPoint, -p.normal, distance);
				currNode->functor.addDist(-overalldistance, siqsqrt);
				currNode = tempNode;
				overalldistance+=distance;
				//std::cout << "Javier" << std::endl;
				//std::this_thread::sleep_for(std::chrono::nanoseconds(1000000000));
			}
			overalldistance = 0;
			currNode = this;
			intersPoint = p.p3D;
			//std::cout << std::endl;
			while(overalldistance < p.size*2){
				//std::cout << "overall: " << overalldistance << std::endl;
				//std::cout << currNode->centre.transpose() << std::endl;
				double distance = 0;
				currNode = currNode->getNextNode(intersPoint, p.normal, distance);
				overalldistance+=distance;
				currNode->functor.addDist(overalldistance, siqsqrt);
				//std::cout << "Javier" << std::endl;
				//std::this_thread::sleep_for(std::chrono::nanoseconds(1000000000));
			}
		}
		patches.clear();
	}


	void writeProb(){
		if(this->ulf)
			this->ulf->writeProb();
		if(this->urf)
			this->urf->writeProb();
		if(this->ulb)
			this->ulb->writeProb();
		if(this->urb)
			this->urb->writeProb();
		if(this->llf)
			this->llf->writeProb();
		if(this->lrf)
			this->lrf->writeProb();
		if(this->llb)
			this->llb->writeProb();
		if(this->lrb)
			this->lrb->writeProb();
		double prob = functor.getVal();
		//std::cout << prob << std::endl;
		Eigen::Vector3i color = Eigen::Vector3i::Zero();
		if(functor.valid){
			color[0]=globals::jet.getred(prob)*255;
			color[1]=globals::jet.getgreen(prob)*255;
			color[2]=globals::jet.getblue(prob)*255;
		}
		else{
			color[0]=255;
			color[1]=255;
			color[2]=255;
		}
		globals::plyWriter.iter(centre, color);
	}


private:

	bool filterRay();

	PosNode<Patch, Functor>* getNeighbor(Eigen::Vector3d p3D, int levelOff){
		//std::cout << "level: " << levelOff << "   " << p3D.transpose() << std::endl;
		//std::cout << centre.transpose() << std::endl;
		if(p3D[0]<centre[0]-size*0.5 || p3D[1]<centre[1]-size*0.5 || p3D[2]<centre[2]-size*0.5
		 ||p3D[0]>centre[0]+size*0.5 || p3D[1]>centre[1]+size*0.5 || p3D[2]>centre[2]+size*0.5){
			//std::cout << "hoch" << std::endl;
			if(parent==NULL){
				std::cout << "f alter" << std::endl;
			}
			return this->parent->getNeighbor(p3D, levelOff+1);
		}
		else{
			//std::cout << "no hoch" << std::endl;
		}
		PosNode* child = NULL;
		//std::cout << "going toge " << std::endl;
		int childID = getChild(child, p3D);
		if(child==NULL)
			child = createChild(childID);
		//std::cout << "id: " << childID << std::endl;
		if(levelOff==1){
			//std::cout << "NOWW" << std::endl;
			return child;
		}
		return child->getNeighbor(p3D, levelOff-1);
	}

	PosNode<Patch, Functor>* getNextNode(Eigen::Vector3d& point,
			 Eigen::Vector3d direction, double& distance){

		Eigen::Vector3d planenormal, planecoord;
		//std::cout << std::endl << std::endl;
		//std::cout << centre.transpose() << "   " << size << std::endl;
		//std::cout << p3D.transpose() << "   " << direction.transpose() << std::endl;
		for(int j=0; j<6; j++){
			planecoord = centre;
			switch(j){
			case 0:
				planenormal[0]=-1.0; planenormal[1]=0.0; planenormal[2]=0.0;
				planecoord[0]+=size*0.5;
				break;
			case 1:
				planenormal[0]=1.0; planenormal[1]=0.0; planenormal[2]=0.0;
				planecoord[0]-=size*0.5;
				break;
			case 2:
				planenormal[0]=0.0; planenormal[1]=-1.0; planenormal[2]=0.0;
				planecoord[1]+=size*0.5;
				break;
			case 3:
				planenormal[0]=0.0; planenormal[1]=1.0; planenormal[2]=0.0;
				planecoord[1]-=size*0.5;
				break;
			case 4:
				planenormal[0]=0.0; planenormal[1]=0.0; planenormal[2]=-1.0;
				planecoord[2]+=size*0.5;
				break;
			case 5:
				planenormal[0]=0.0; planenormal[1]=0.0; planenormal[2]=1.0;
				planecoord[2]-=size*0.5;
				break;
			}
			//std::cout << "inters: " << planenormal.transpose() << "  " << planecoord.transpose() << std::endl;
			if(this->getPlaneIntersection(planenormal, planecoord, point, direction, distance)){
				Eigen::Vector3d intersecpoint = point + distance*direction;
				//std::cout << "ttt " << tttt.transpose() << std::endl;
				Eigen::Vector3d dist = planecoord-intersecpoint;
				//std::cout << "jup " << planecoord.transpose() << std::endl;
				//std::cout << "aa " << intersecpoint.transpose() << std::endl;
				//std::cout << "dist:" << dist.transpose() << "   "  << size*0.5 << std::endl;
				//WHY NOT 0.5*
				if((fabs(dist[0])>size*0.5) || (fabs(dist[1])>size*0.5) || (fabs(dist[2])>size*0.5)){
					//std::cout << "NO NO: " << size*0.5 << std::endl;
				}
				else{
					//distance+=size/2;
					point = intersecpoint + 1e-12*direction;
					//std::cout << "YEAH" << std::endl;
					break;
				}
				//if((intersecpoint[0] < edgeP1[0] && intersecpoint[0] < edgeP1[0]) &&
				//   (intersecpoint[0] < edgeP1[0] && intersecpoint[0] < edgeP1[0])) //check
				//	break;
			}
			//std::cout << distance << std::endl;
		}
#ifdef debug
		if(distance<=0){
			std::cout << "ERROR" << std::endl;
		}
		if(!(point[0]<centre[0]-size*0.5 || point[1]<centre[1]-size*0.5 || point[2]<centre[2]-size*0.5
		   ||point[0]>centre[0]+size*0.5 || point[1]>centre[1]+size*0.5 || point[2]>centre[2]+size*0.5))
			std::cout << "Error: Point in voxel" << std::endl;
#endif
		//std::cout << "fin" << std::endl;
		//std::cout << this->parent->getNeighbor(intersecpoint, 1)->centre << std::endl;
		return this->parent->getNeighbor(point, 1);
	}

	inline int getChild(PosNode<Patch, Functor>*& child, Eigen::Vector3d p3D){
		if(p3D[0]>centre[0]){
			if(p3D[1]>centre[1]){
				if(p3D[2]>centre[2]){
					child=(PosNode<Patch, Functor>*)this->urb;
					return 1;
				}
				else{
					child=(PosNode<Patch, Functor>*)this->urf;
					return 2;
				}
			}
			else{
				if(p3D[2]>centre[2]){
					child=(PosNode<Patch, Functor>*)this->lrb;
					return 3;
				}
				else{
					child=(PosNode<Patch, Functor>*)this->lrf;
					return 4;
				}
			}
		}
		else{
			if(p3D[1]>centre[1]){
				if(p3D[2]>centre[2]){
					child=(PosNode<Patch, Functor>*)this->ulb;
					return 5;
				}
				else{
					child=(PosNode<Patch, Functor>*)this->ulf;
					return 6;
				}
			}
			else{
				if(p3D[2]>centre[2]){
					child=(PosNode<Patch, Functor>*)this->llb;
					return 7;
				}
				else{
					child=(PosNode<Patch, Functor>*)this->llf;
					return 8;
				}
			}
		}
		std::cout << "WARNING: Does not fit" << std::endl;
		return 0;
	}


	Eigen::Vector3d centre;
	double size;
	Functor functor;
	std::vector<Patch> patches;


	PosNode<Patch, Functor>* parent;
	PosNode<Patch, Functor>* ulf; //upper left front child
	PosNode<Patch, Functor>* urf; //upper right front child
	PosNode<Patch, Functor>* ulb; //upper left back child
	PosNode<Patch, Functor>* urb; //upper right back child
	PosNode<Patch, Functor>* llf; //lower left front child
	PosNode<Patch, Functor>* lrf; //lower right front child
	PosNode<Patch, Functor>* llb; //lower left back child
	PosNode<Patch, Functor>* lrb; //lower right back child
};

#endif /* OCTREE_POSNODE_H_ */
