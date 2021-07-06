

#include "BaseLib/Geometry/CurveFitting.h"

namespace mg
{

bool fitCurveLLS2Order(std::vector< cml::vector3d> &point_set_2d, double &out_b2, double &out_b1, double &out_b0)
{
	int n = (int)point_set_2d.size();

	// X*b = y
	cml::matrixd X(n, 3);
	cml::vectord b(3);
	cml::vectord y(n);

	for ( int i=0; i<n; i++ )
	{
		double px = point_set_2d[i][0];
		double py = point_set_2d[i][1];

		X(i, 0) = px*px;
		X(i, 1) = px;
		X(i, 2) = 1;

		y[i] = py;
	}


	// (Xt*X)*b = Xt*y
	// Xt = transpose of X.
	// A = Xt*X
	// Y = Xt*y
	cml::matrixd Xt;
	cml::matrixd A(3, 3);
	Xt = cml::transpose(X);
	A = Xt * X;

	cml::vectord Y(n);
	Y  = Xt * y;

	
	b = cml::lu_solve(lu(A), Y);
	//b.solve(A, Y);

	out_b2 = b[0];
	out_b1 = b[1];
	out_b0 = b[2];

	return true;
}

};
