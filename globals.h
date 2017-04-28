/*
 * globals.h
 *
 *  Created on: 28.04.2017
 *      Author: andreask
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "IO/PLYWriter.h"

struct JET{
	double interpolate( double val, double y0, double x0, double y1, double x1 ) {
		return (val-x0)*(y1-y0)/(x1-x0) + y0;
	}
	double base( double val ) {
		if ( val <= -0.75 ) return 0;
		else if ( val <= -0.25 ) return interpolate( val, 0.0, -0.75, 1.0, -0.25 );
		else if ( val <= 0.25 ) return 1.0;
		else if ( val <= 0.75 ) return interpolate( val, 1.0, 0.25, 0.0, 0.75 );
		else return 0.0;
	}
	double getred( double gray ) {
		return base( gray*2.0 - 1.5 );
	}
	double getgreen( double gray ) {
		return base( gray*2.0 - 1.0);
	}
	double getblue( double gray ) {
		return base( gray*2.0 - 0.5 );
	}
};

namespace globals
{
	extern PLYWriter plyWriter;
	extern JET jet;
	extern int octreeNodes;
};

#endif /* GLOBALS_H_ */
