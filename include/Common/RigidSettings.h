#ifndef RIGID_SETTINGS_H
#define RIGID_SETTINGS_H

// Uncomment next line to compile out asserts from all source files.
//#define NDEBUG
#include <cassert>

// Set ENABLE_DEBUG to 0 to compile out DEBUG statements.
#define ENABLE_DEBUG 0
#if ENABLE_DEBUG
#include <iostream>
#define DEBUG( x ) x
#else
#define DEBUG( x )
#endif

// NULL is not guaranteed to be defined.
#ifndef NULL
#define NULL 0
#endif


// Place all global library settings and constants in this file.
namespace Rigid2D{
	
	typedef float Real;
	typedef double PreciseReal;

	const PreciseReal tau = 6.283185307;

	// Starting step size for the OdeSolver.
	const Real StartingStepSize = 0.01;


}

#endif
