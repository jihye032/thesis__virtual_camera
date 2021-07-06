

#include "BaseLib/Geometry/Curve.h"

namespace mg
{

Curve::Curve()
{
	dimension_ = 0;
	domain_start_ = 0.;
	domain_end_ = 1.;
}

Curve::Curve(int dim, double domain_start, double domain_end)
{
	SetDimension(dim);
	SetDomain(domain_start, domain_end);
}

void 
Curve::Sample(std::vector<cml::vectord> &out_sample, double step) const
{
	Sample(out_sample, domain_start_, domain_end_-domain_start_, step);
}

void 
Curve::Sample(std::vector<cml::vectord> &out_sample, double start_t, double range, double step) const
{
	double t = start_t;
	while ( t <= start_t+range )
	{
		out_sample.push_back(Get(t));
		t += step;
	}
}




cml::vectord
SampledCurve::Get(double t) const
{
	if ( data_.empty() )
	{
		std::cerr << "SampledCurve::Get(double t): No Data!" << std::endl;
		exit(0);
	}

	if ( data_.size() == 1) return data_.front();
	if ( t <= domain_start_ ) return data_.front();
	if ( t >= domain_end_ ) return data_.back();
	

	// Linear Search. Quick search will be better. 
	cml::vectord d0, d1;
	for ( int i=1; i<(int)data_.size(); i++ )
	{
		if ( times_[i-1] <= t && times_[i] >= t )
		{
			d0 = data_[i-1];
			d1 = data_[i];
			break;
		}
	}

	double real = t - ((int)t);
	
	cml::vectord d;

	d = (1.-real)*d0 + real*d1;
	return d;
}




void
SampledCurve::SetDomain(double new_start, double new_end)
{
	double new_range = new_end-new_start;
	
	double scale = new_range/(domain_end_-domain_start_);

	// Update Time Values
	for ( int i=0; i<(int)times_.size(); i++ )
	{
		times_[i] = (times_[i]-domain_start_)*scale + new_start;
	}

	Curve::SetDomain(new_start, new_end);
}

void 
SampledCurve::SetSampledCurve(const std::vector<double> &times, const std::vector<cml::vectord> &data)
{
	times_.clear();
	data_.clear();

	times_.assign(times.begin(), times.end());
	data_.assign(data.begin(), data.end());
}


void 
SampledCurve::SetUniformlySampledCurve(const std::vector<cml::vectord> &data)
{
	times_.clear();
	data_.clear();

	data_.assign(data.begin(), data.end());

	int n = (int)data_.size();
	times_.resize(n);

	
	if ( n == 1 )
	{
		times_[0] = domain_start_;
	}
	else if ( n > 1 )
	{
		double time_step = (domain_end_-domain_start_)/(n-1);

		for ( int i=0; i<n; i++ )
		{
			times_[i] = (domain_start_)+i*time_step;
		}

		// To ensure the last time is exactly same to the domain_end_
		times_.back() = domain_end_;
	}
}


void 
SampledCurve::UniformResample(double step)
{
	std::vector<double> new_times;
	std::vector<cml::vectord> new_data;
	
	double t = domain_start_;
	while ( t <= domain_end_ )
	{
		new_times.push_back(t);
		new_data.push_back(Get(t));
		t += step;
	}

	SetSampledCurve(new_times, new_data);
}








CubicBezierCurve::CubicBezierCurve(const cml::vectord &c0, const cml::vectord &c1, const cml::vectord &c2, const cml::vectord &c3)
{
	dimension_ = 1;
	domain_start_ = 0.;
	domain_end_ = 1.;
	SetControlPoints(c0, c1, c2, c3);
}

void
CubicBezierCurve::SetControlPoints(const cml::vectord &c0, const cml::vectord &c1, const cml::vectord &c2, const cml::vectord &c3)
{
	c0_ = c0;
	c1_ = c1;
	c2_ = c2;
	c3_ = c3;

	SetDimension(c0_.size());
}


cml::vectord
CubicBezierCurve::Get(double t) const
{
	cml::vectord p;

	t = t-domain_start_;
	t /= (domain_end_-domain_start_);

	double t2 = 1-t;

	p = t2*t2*t2*c0_ + 3.*t2*t2*t*c1_ + 3.*t2*t*t*c2_ + t*t*t*c3_;

	return p;
}







};
