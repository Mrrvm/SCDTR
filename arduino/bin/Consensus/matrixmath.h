/*
 *  MatrixMath.h Library for Matrix Math
 *
 *  Modified from Charlie Matlack.
 *  Modified from code by RobH45345 on Arduino Forums, algorithm from
 *  NUMERICAL RECIPES: The Art of Scientific Computing.
 *  Modified to work with Arduino 1.0/1.5 by randomvibe & robtillaart
 *  Made into a real library on GitHub by Vasilis Georgitzikis (tzikis)
 *  so that it's easy to use and install (March 2015)
 */

#ifndef matrixmath_h
#define matrixmath_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class MatrixMath
{
public:
	//MatrixMath();
	void Print(float* A, int m, int n, String label);
	float* Copy(float* A, int n, int m);
	float* Multiply(float* A, float* B, int m, int p, int n);
	float* Add(float* A, float* B, int m, int n);
	float* Subtract(float* A, float* B, int m, int n);
	float* Transpose(float* A, int m, int n);
	int Invert(float* A, int n);
	float* MultiplyConst(float* A, int m, int n, float constant);
	float* AddConst(float* A, int m, int n, float constant);
	float* InvertElements(float* A, int m, int n);
};

extern MatrixMath Matrix;
#endif