
#pragma once
#include "BaseLib/CmlExt/CmlExt.h"
#include <vector>
#include <memory>

namespace mg
{

class Interpolation
{
public:
	Interpolation();


	virtual cml::vectord Get(double t) const = 0;
	virtual cml::vector3d Get3d(double t) const;
	virtual double Get1d(double t) const;

	/**
	@param times Must be sorted in increasing order. The domain range is set by the first and the last times.
	@param data the size must be sample to the times.
	*/
	void SetKeyPoints(const std::vector<double> &times, const std::vector<cml::vectord> &keys);
	void SetKeyPoints(const std::vector<cml::vectord> &keys);

	void SetKeyPoints1d(const std::vector<double> &times, const std::vector<double> &keys);
	void SetKeyPoints1d(const std::vector<double> &keys);

	void SetKeyPoints3d(const std::vector<double> &times, const std::vector<cml::vector3d> &keys);
	void SetKeyPoints3d(const std::vector<cml::vector3d> &keys);

	int dimension() const { return dimension_ ; }
	std::pair<double, double> range() const { return std::make_pair(times_.front(), times_.back()); }

protected:
	virtual void Interpolate() = 0;

protected:
	int dimension_;

	std::vector<double> times_;
	std::vector<cml::vectord> keys_;
};



class PolynomialInterpolation : public Interpolation
{
public:

	
	virtual cml::vectord Get(double t) const;

protected:
	virtual void Interpolate() override;

protected:
	int poly_order_;	// will be set to (keys_.size() - 1)
	cml::matrixd coefficients_;	// Each row has the set of coefficients for one polynomial function.
	
};


class CubicBSplineInterpolation : public Interpolation
{
public:
	virtual cml::vectord Get(double t) const;

protected:
	/**
	It is the first basis function.
	@param n_t a normalized variable in [0, 1].
	*/
	double B0(double n_t) const;

	/**
	It is the second basis function.
	@param n_t a normalized variable in [0, 1].
	*/
	double B1(double n_t) const;

	/**
	It is the third basis function.
	@param n_t a normalized variable in [0, 1].
	*/
	double B2(double n_t) const;

	/**
	It is the fourth basis function.
	@param n_t a normalized variable in [0, 1].
	*/
	double B3(double n_t) const;


	/**
	It produces keys_.size()-1 cubic curves. 
	*/
	virtual void Interpolate() override;


	/**
	It evaulated i's cubic function with n_t, a normalized variable.. 
	@param n_t a normalized variable in [0, 1].
	*/
	cml::vectord CubicFunction(int i, double n_t) const;


protected:
	//cml::matrixd control_points_;	// c by d matrix. c is the number of condrol points. d is dimension_.
	std::vector<cml::vectord> control_points_;	// Its size will be same to the number of keys + 2.
};




};
