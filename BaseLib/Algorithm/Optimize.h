#pragma once

#include "BaseLib/CmlExt/CmlExt.h"

void mnbrak(double&, double&, double&,
	double&, double&, double&, double (*func)(double));

double brent(double, double, double,
	double (*f)(double), double, double&);

void linmin(cml::vectord&, cml::vectord&, int, double&, double (*func)(cml::vectord const&));

void gradient_descent(cml::vectord&, int, double, int&, double&,
	double (*func)(cml::vectord const&),
	double (*dfunc)(cml::vectord const&, cml::vectord&));

void frprmn(cml::vectord&, int, double, int&, double&,
	double (*func)(cml::vectord const&),
	double (*dfunc)(cml::vectord const&, cml::vectord&));

void error(char*);

