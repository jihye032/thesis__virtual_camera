
#pragma once
#include "BaseLib/CmlExt/CmlExt.h"
#include <vector>
#include <memory>

namespace mg
{

class Curve
{
public:
	Curve();
	Curve(int dim, double domain_start, double domain_end);
	
	virtual void SetDimension(int d) { dimension_ = d; }
	virtual void SetDomain(double start, double end) { domain_start_ = start; domain_end_ = end; }


	virtual cml::vectord Get(double t) const = 0;
	virtual void Sample(std::vector<cml::vectord> &out_sampled_data, double step) const;
	virtual void Sample(std::vector<cml::vectord> &out_sampled_data, double start_t, double range, double step) const;

protected:
	int dimension_;
	double domain_start_;
	double domain_end_;
};


class SampledCurve : public Curve
{
public:

	virtual void SetDomain(double start, double end);

	/**
	@param times Must be sorted in increasing order.
	@param data the size must be sample to the times.
	*/
	void SetSampledCurve(const std::vector<double> &times, const std::vector<cml::vectord> &data);

	/**
	This function regards as @param data was sampled uniformly, so set uniformly distributed times from domain_start_ to domain_end_ for each element.
	*/
	void SetUniformlySampledCurve(const std::vector<cml::vectord> &data);

	virtual cml::vectord Get(double t) const;


	void UniformResample(double step);


protected:
	std::vector<double> times_;
	std::vector<cml::vectord> data_;
};


class CubicBezierCurve : public Curve
{
public:
	CubicBezierCurve(const cml::vectord &c0, const cml::vectord &c1, const cml::vectord &c2, const cml::vectord &c3);
	
	void SetControlPoints(const cml::vectord &c0, const cml::vectord &c1, const cml::vectord &c2, const cml::vectord &c3);
	
	virtual cml::vectord Get(double t) const;

protected:
	cml::vectord c0_, c1_, c2_, c3_;
};



};
