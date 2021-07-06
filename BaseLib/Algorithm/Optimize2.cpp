
#include "BaseLib/Algorithm/Optimize.h"

#define GOLD 1.61803398874989484820
#define GLIMIT 100.0
#define TINY 1.0e-20
//#define ITMAX 100
#define ITMAX 50

#define CGOLD 0.3819660
#define ZEPS 1.0e-10

#define TRUE    1
#define FALSE   0

#define MAX(x,y) ( ((x)>(y)) ? (x) : (y) )
#define MIN(x,y) ( ((x)<(y)) ? (x) : (y) )
#define ABS(x)   ( ((x)>0.0) ? (x) :-(x) )
#define ACOS(x)  ( ((x)>1.0) ? (0) : ( ((x)<-1.0) ? (M_PI) : (acos(x)) ) )
#define ASIN(x)  ( ((x)>1.0) ? (M_PI/2.0) : ( ((x)<-1.0) ? (-M_PI/2.0) : (asin(x)) ) )
#define SQR(x)   ( (x)*(x) )
#define SHIFT(a,b,c,d) (a)=(b);(b)=(c);(c)=(d);
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

#define TOL 2.0e-4
//#define EPS 1.0e-4
#define EPS 1.0e-3


int    ncom;
double (*nrfunc) (cml::vectord const&);
static cml::vectord pcom, xicom;

double f1dim(double x)
{
	static cml::vectord xt; xt.resize(ncom);

	for (int j=0; j<ncom; j++)
		xt[j] = pcom[j] + x * xicom[j];
	double f = (*nrfunc) (xt);

	return f;
}

void linmin( cml::vectord &p, cml::vectord &xi, int n, double &fret, double (*func) (cml::vectord const&))
{
	double xx, xmin, fx, fb, fa, bx, ax;

	ncom = n;
	pcom.resize(n);
	xicom.resize(n);
	nrfunc = func;

	int j=0;
	for (j=0; j<n; j++)
	{
		pcom[j] = p[j];
		xicom[j] = xi[j];
	}

	ax = 0.0;
	xx = 1.0;
	mnbrak(ax, xx, bx, fa, fx, fb, f1dim);
	fret = brent(ax, xx, bx, f1dim, TOL, xmin);

	for (j=0; j<n; j++)
	{
		xi[j] *= xmin;
		p[j] += xi[j];
	}
}


void gradient_descent(cml::vectord &p, int n, double ftol, int &iter, double &fret,
	double (*func)  (cml::vectord const&),
	double (*dfunc) (cml::vectord const&,cml::vectord&))
{
	double fp;
	static cml::vectord xi; xi.resize(n);

	for (iter=0; iter<ITMAX; iter++)
	{
		fp = (*dfunc) (p, xi);

		linmin(p, xi, n, fret, func);
		if (2.0 * fabs(fret - fp) <= ftol * (fabs(fret) + fabs(fp) + EPS))
			return;
	}

	error("Too many iterations in gradient_descent");
}

void frprmn(cml::vectord &p, int n, double ftol, int &iter, double &fret,
	double (*func)  (cml::vectord const&),
	double (*dfunc) (cml::vectord const&,cml::vectord&))
{
	double gg, gam, fp, dgg;

	static cml::vectord g;   g.resize(n);
	static cml::vectord h;   h.resize(n);
	static cml::vectord xi; xi.resize(n);

	fp = (*dfunc) (p, xi);

	for (int j=0; j<n; j++)
	{
		g[j] = -xi[j];
		xi[j] = h[j] = g[j];
	}

	for (iter=0; iter<ITMAX; iter++)
	{
		linmin(p, xi, n, fret, func);
		if (2.0 * fabs(fret - fp) <= ftol * (fabs(fret) + fabs(fp) + EPS)) return;

		fp = (*dfunc) (p, xi);
		dgg = gg = 0.0;

		int j=0;
		for (j=0; j<n; j++)
		{
			gg += g[j] * g[j];
			dgg += (xi[j] + g[j]) * xi[j];
		}

		if (gg == 0.0) return;

		gam = dgg / gg;

		for (j=0; j<n; j++)
		{
			g[j] = -xi[j];
			xi[j] = h[j] = g[j] + gam * h[j];
		}
	}

	error("Too many iterations in frprmn");
}

#undef ITMAX

