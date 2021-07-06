


#include "BaseLib/Algorithm/Signal1D.h"

namespace mg
{

void
Signal1D::MakeSmooth()
{
	static const int g_kernel_size = 5;
	static double g_kernel[5]={7.0/107.0, 26.0/107.0, 41.0/107.0, 26.0/107.0, 7.0/107.0};

	Signal1D &data = (*this);
	std::vector<double> ori = data;

	for ( int i=0; i<(int)ori.size(); i++ )
	{
		double mean = 0;
		for ( int k=0; k<g_kernel_size; k++ )
		{
			int ik = i - (g_kernel_size/2) + k;
			if ( ik<0 ) ik = 0;
			if ( ik>=(int)ori.size() ) ik = ori.size()-1;

			mean += g_kernel[k] * ori[ik];
		}

		data[i] = mean;
	}
}


void
Signal1D::FindValleyPoints(std::vector<int> &out) const
{
	out.clear();

	for ( int i=1; i<(int)size()-1; i++ )
	{
		if ( (*this)[i-1] > (*this)[i] && (*this)[i+1] > (*this)[i] )
			out.push_back(i);
	}
}


double
Signal1D::GetSum(int start, int end) const
{
	double sum = 0.0;
	for ( int i=start; i<=end; i++ )
	{
		sum += (*this)[i];
	}

	return sum;
}


int
Signal1D::GetMax() const
{
	return GetMax(0, this->size()-1);
}

int
Signal1D::GetMax(int start, int end) const
{
	int max_f = 0;
	double max = 0.0;
	for ( int i=start; i<=end; i++ )
	{
		if ( i==start || max < (*this)[i] )
		{
			max = (*this)[i];
			max_f = i;
		}
	}

	return max_f;
}

double
Signal1D::GetMaxValue() const
{
	return GetMaxValue(0, this->size()-1);
}

double
Signal1D::GetMaxValue(int start, int end) const
{
	double max = 0.0;
	for ( int i=start; i<=end; i++ )
	{
		if ( i==start || max < (*this)[i] )
		{
			max = (*this)[i];
		}
	}

	return max;
}


};