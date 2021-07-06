

#include "BaseLib/Geometry/Interpolation.h"

namespace mg
{

Interpolation::Interpolation()
{
}







void
Interpolation::SetKeyPoints(const std::vector<double> &times, const std::vector<cml::vectord> &keys)
{
	keys_.clear();
	times_.clear();

	keys_.assign(keys.begin(), keys.end());
	times_.assign(times.begin(), times.end());

	dimension_ = (int)keys_.front().size();

	Interpolate();
}


void
Interpolation::SetKeyPoints(const std::vector<cml::vectord> &keys)
{
	std::vector<double> times(keys.size());

	for ( int i=0; i<(int)keys.size(); i++ )
	{
		times[i] = (double)i;
	}

	SetKeyPoints(times, keys);
}

void
Interpolation::SetKeyPoints1d(const std::vector<double> &times, const std::vector<double> &keys1d)
{
	std::vector<cml::vectord> keys;

	for ( int i=0; i<(int)keys1d.size(); i++ )
	{
		cml::vectord d(1);
		d[0] = keys1d[i];
		keys.push_back(d);
	}

	SetKeyPoints(times, keys);
}

void
Interpolation::SetKeyPoints1d(const std::vector<double> &keys1d)
{
	std::vector<cml::vectord> keys;

	for ( int i=0; i<(int)keys1d.size(); i++ )
	{
		cml::vectord d(1);
		d[0] = keys1d[i];
		keys.push_back(d);
	}

	SetKeyPoints(keys);
}


void
Interpolation::SetKeyPoints3d(const std::vector<double> &times, const std::vector<cml::vector3d> &keys3d)
{
	std::vector<cml::vectord> keys;

	for ( int i=0; i<(int)keys3d.size(); i++ )
	{
		cml::vectord d(3);
		d[0] = keys3d[i][0];
		d[1] = keys3d[i][1];
		d[2] = keys3d[i][2];
		keys.push_back(d);
	}

	SetKeyPoints(times, keys);
}

void
Interpolation::SetKeyPoints3d(const std::vector<cml::vector3d> &keys3d)
{
	std::vector<cml::vectord> keys;

	for ( int i=0; i<(int)keys3d.size(); i++ )
	{
		cml::vectord d(3);
		d[0] = keys3d[i][0];
		d[1] = keys3d[i][1];
		d[2] = keys3d[i][2];
		keys.push_back(d);
	}

	SetKeyPoints(keys);
}

cml::vector3d 
Interpolation::Get3d(double t) const
{
	cml::vectord r = Get(t);
	cml::vector3d r3(0., 0., 0.);

	for ( int i=0; i<std::min(3, (int)r.size()); i++ )
	{
		r3[i] = r[i];
	}

	return r3;
}


double
Interpolation::Get1d(double t) const
{
	cml::vectord r = Get(t);

	return r[0];
}






void
PolynomialInterpolation::Interpolate()
{
	if ( keys_.size() <=1 )
	{
		std::cerr << "PolynomialCurve::ComputePolynomial() : Not enought keys." << std::endl;
		exit(0);
	}
	poly_order_ = keys_.size()-1;

	int k = keys_.size(); // the number of the input key values
	int d = dimension_; // the dimension of the input data vector.
	int o = poly_order_; // the order of the polynomial
	int c_num = o+1; // the number of coefficients for o's order function.
	
	// k is always same to the c_num.
	

	// A * C = K.
	cml::matrixd A, C, K;
	// A: A is a k-by-c_num matrix (square), filled with t^0, t^1, t^2.... t is the variable of the polynomials. The polynomial, we want to find, is function of t. t comes from times_
	// C: It is a c_num-by-d by matrix. It will be filled with coefficients, we want to compute. Each column has the set of coefficients for one polynomial function.
	// K: It is a k-by-d matrix of the key values (keys_). 

	// Build A
	A.resize(k, c_num);
	{
		for ( int r=0; r<k; r++ )
		{
			for ( int c=0; c<c_num; c++ )
			{
				A(r,c) = 1.;
			
				for ( int i=0; i<c; i++ )
				{
					A(r, c) *= times_[c];
				}
			}
		}
	}

	// Build K
	K.resize(k, d);
	{
		for ( int r=0; r<k; r++ )
		{
			for ( int c=0; c<d; c++ )
			{
				K(r,c) = keys_[r][c];
			}
		}
	}

	// Compute
	C = cml::inverse(A) * K;
	coefficients_ = cml::transpose(C);
}

cml::vectord 
PolynomialInterpolation::Get(double t) const
{
	cml::vectord T(poly_order_+1);
	for ( int i=0; i<poly_order_+1; i++)
	{
		T[i] = 1.;
		for ( int j=0; j<i; j++ )
		{
			T[i] *= t;
		}
	}

	cml::vectord result = (coefficients_) * T;

	return result;
}






double
CubicBSplineInterpolation::B0(double n_t) const
{
	return (1./6.)*(1.-n_t)*(1.-n_t)*(1.-n_t);
}

double
CubicBSplineInterpolation::B1(double n_t) const
{
	return (1./6.)*(3*n_t*n_t*n_t - 6.*n_t*n_t + 4);
}

double
CubicBSplineInterpolation::B2(double n_t) const
{
	return (1./6.)*(-3*n_t*n_t*n_t + 3.*n_t*n_t + 3.*n_t + 1.);
}

double
CubicBSplineInterpolation::B3(double n_t) const
{
	return (1./6.)*(n_t)*(n_t)*(n_t);
}




void
CubicBSplineInterpolation::Interpolate()
{
	int num_controls = (int)keys_.size()+2; // number of control points.

	cml::matrixd A(num_controls, num_controls);
	for ( int r=0; r<num_controls; r++ )
	{
		if ( r == 0 )
		{
			A(r, 0) = 1.;
			A(r, 1) = -2.;
			A(r, 2) = 1.;
		}
		else if ( r== num_controls-1 )
		{
			A(r, r-2) = 1.;
			A(r, r-1) = -2.;
			A(r, r) = 1.;
		}
		else
		{
			A(r, r-1) = 1./6;
			A(r, r) = 4./6;
			A(r, r+1) = 1./6;
		}
	}

	cml::matrixd P(num_controls, dimension_); // A c by d matrix , where c is the number of control points and d is dimension_. Filled by key values
	for ( int r=0; r<num_controls; r++ )
	{
		for ( int c=0; c<dimension_; c++ )
		{
			if ( r == 0 || r == num_controls-1 )
			{
				P(r, c) = 0.;
			}
			else
			{
				P(r, c) = keys_[r-1][c];
			}

		}
	}



	cml::matrixd C;		// Matrix of the result control points. It will be a c by d matrix, where c is the number of control points and d is dimension_.

	C = cml::inverse(A) * P;

	// copy results to control_points_
	control_points_.resize(num_controls);
	for ( int r=0; r<num_controls; r++ )
	{
		control_points_[r].resize(dimension_);
		for ( int c=0; c<dimension_; c++ )
		{
			control_points_[r][c] = C(r, c);
		}
	}
}


cml::vectord 
CubicBSplineInterpolation::CubicFunction(int i, double n_t) const
{
	return control_points_[i]*B0(n_t) +
		control_points_[i+1]*B1(n_t) +
		control_points_[i+2]*B2(n_t) +
		control_points_[i+3]*B3(n_t);

}

cml::vectord 
CubicBSplineInterpolation::Get(double t) const
{
	if ( t <= times_.front() ) return keys_.front();
	if ( t >= times_.back() ) return keys_.back();
		
	int i=0;
	double n_t =0.;
	for ( i=0; i<(int)times_.size()-1; i++ )
	{
		if ( times_[i] <= t && t < times_[i+1] )
		{
			n_t = (t-times_[i])/(times_[i+1]-times_[i]);
			break;
		}
	}

	return CubicFunction(i, n_t);
}








};
