/*
 * BayesGauss.hpp
 *
 *  Created on: 26.04.2017
 *      Author: andreask
 */

#ifndef FUNCTORS_BAYESGAUSS_H_
#define FUNCTORS_BAYESGAUSS_H_

struct BayesGauss{
public:
	BayesGauss(){
		logprob=0;
		valid=false;
	}
	void addDist(double dist, double sigsqrt){
		double prob = 1.0 - (0.5 * (1.0 + erf(dist/sigsqrt)));
		logprob += log(prob / (1.0 -prob));
		valid = true;
	}
	double getVal(){
		double tempProb = exp(logprob);
		return tempProb/(1.0+tempProb);
	}
	bool valid;
private:
	double logprob;
};

#endif /* FUNCTORS_BAYESGAUSS_H_ */
