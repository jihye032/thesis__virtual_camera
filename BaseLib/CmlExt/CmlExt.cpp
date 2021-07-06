

#include <math.h>
#include "BaseLib/CmlExt/CmlExt.h"
#include <assert.h>

#define MAX(x,y) ( ((x)>(y)) ? (x) : (y) )
#define MIN(x,y) ( ((x)<(y)) ? (x) : (y) )
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

namespace cml
{
	cml::vector3d ToRAD(const cml::vector3d v) { return cml::vector3d(cml::rad(v[0]), cml::rad(v[1]), cml::rad(v[2])); }
	cml::vector3d ToDEG(const cml::vector3d v) { return cml::vector3d(cml::deg(v[0]), cml::deg(v[1]), cml::deg(v[2])); }

	cml::matrix44d MatrixRotationQuaternion(const cml::quaterniond &q)
	{
		cml::matrix44d m;
		cml::matrix_rotation_quaternion(m, q);

		return m;
	}

	cml::matrix44d MatrixRotationEuler(const cml::vector3d& euler)
	{
		cml::matrix44d m;
		cml::matrix_rotation_euler(m, euler[0], euler[1], euler[2], cml::euler_order_xyz);

		return m;
	}

	cml::matrix44d MatrixRotationEuler(double x, double y, double z)
	{
		cml::matrix44d m;
		cml::matrix_rotation_euler(m, x, y, z, cml::euler_order_xyz);

		return m;
	}


	cml::matrix44d MatrixRotationAxisAngle(const cml::vector3d &axis, double angle)
	{
		cml::matrix44d m;
		cml::matrix_rotation_axis_angle(m, axis, angle);

		return m;
	}

	cml::matrix44d MatrixTranslation(double x, double y, double z)
	{
		cml::matrix44d m;
		cml::matrix_translation(m, x, y, z);

		return m;
	}

	cml::matrix44d MatrixTranslation(const cml::vector3d &t)
	{
		cml::matrix44d m;
		cml::matrix_translation(m, t);

		return m;
	}


	cml::matrix44d MatrixUniformScaling(double s)
	{
		cml::matrix44d m;
		cml::matrix_uniform_scale(m, s);

		return m;
	}

	cml::matrix44d MatrixScaling(const cml::vector3d &s)
	{
		cml::matrix44d m;
		cml::matrix_scale(m, s);

		return m;
	}

	cml::matrix44d MatrixScaling(double x, double y, double z)
	{
		cml::matrix44d m;
		cml::matrix_scale(m, x, y, z);

		return m;
	}








	cml::matrix44f MatrixRotationEuler(const cml::vector3f& euler)
	{
		cml::matrix44f m;
		cml::matrix_rotation_euler(m, euler[0], euler[1], euler[2], cml::euler_order_xyz);

		return m;
	}

	cml::matrix44f MatrixRotationEuler(float x, float y, float z)
	{
		cml::matrix44f m;
		cml::matrix_rotation_euler(m, x, y, z, cml::euler_order_xyz);

		return m;
	}


	cml::matrix44f MatrixRotationAxisAngle(const cml::vector3f &axis, float angle)
	{
		cml::matrix44f m;
		cml::matrix_rotation_axis_angle(m, axis, angle);

		return m;
	}

	cml::matrix44f MatrixTranslation(float x, float y, float z)
	{
		cml::matrix44f m;
		cml::matrix_translation(m, x, y, z);

		return m;
	}

	cml::matrix44f MatrixTranslation(const cml::vector3f &t)
	{
		cml::matrix44f m;
		cml::matrix_translation(m, t);

		return m;
	}


	cml::matrix44f MatrixUniformScaling(float s)
	{
		cml::matrix44f m;
		cml::matrix_uniform_scale(m, s);

		return m;
	}

	cml::matrix44f MatrixScaling(const cml::vector3f &s)
	{
		cml::matrix44f m;
		cml::matrix_scale(m, s);

		return m;
	}

	cml::matrix44f MatrixScaling(float x, float y, float z)
	{
		cml::matrix44f m;
		cml::matrix_scale(m, x, y, z);

		return m;
	}

	cml::quaterniond QuaternionMatrix(const cml::matrix44d &m)
	{
		cml::matrix33d mm;
		for (int i = 0; i<3; i++)
			for (int j = 0; j<3; j++)
			{
				mm(i, j) = m(i, j);
			}
		return QuaternionMatrix(mm);
	}

	cml::quaterniond QuaternionMatrix(const cml::matrix33d &m)
	{
		cml::quaterniond q;
		cml::quaternion_rotation_matrix(q, m);
		return q;
	}


	cml::quaterniond QuaternionEuler(const cml::vector3d &euler)
	{
		cml::quaterniond q;
		cml::quaternion_rotation_euler(q, euler[0], euler[1], euler[2], cml::euler_order_xyz);
		return q;
	}




	cml::vector3d TranslationMatrix(const cml::matrix44d &m)
	{
		cml::vector3d v;
		v[0] = m(3, 0);
		v[1] = m(3, 1);
		v[2] = m(3, 2);

		return v;
	}

	cml::vector3d Rotate(const cml::quaterniond &q, const cml::vectord &in)
	{
		double angle;
		cml::vector3d axis;
		cml::quaternion_to_axis_angle(q, axis, angle);
		return cml::rotate_vector(in, axis, angle);
	}


	cml::quaterniond EXP(const cml::vector3d v)
	{
		cml::quaterniond qq(0, v);
		return cml::exp(qq);
	}

	cml::vector3d LOG(const cml::quaterniond q)
	{
		cml::quaterniond qq = cml::log(q);
		return qq.imaginary();
	}


	static double pythag(double a, double b)
	{
		double pa = fabs(a);
		double pb = fabs(b);

		if (pa > pb) return pa * sqrt(1.0f + sqr(pb / pa));
		else return (pb == 0.0f ? 0.0f : pb * sqrt(1.0f + sqr(pa / pb)));
	}

	void TRSdecompose(const cml::matrix44d &in, cml::vector3d &out_T, cml::quaterniond &out_R, cml::vector3d &out_S)
	{
		// extract translation
		out_T[0] = in(0, 3);
		out_T[1] = in(1, 3);
		out_T[2] = in(2, 3);

		// extract the rows of the matrix
		cml::vector3d vRows[3] = {
			cml::vector3d(in(0, 0),in(1, 0),in(2, 0)),
			cml::vector3d(in(0, 1),in(1, 1),in(2, 1)),
			cml::vector3d(in(0, 2),in(1, 2),in(2, 2))
		};

		// extract the scaling factors
		out_S[0] = vRows[0].length();
		out_S[1] = vRows[1].length();
		out_S[2] = vRows[2].length();

		// and the sign of the scaling
		
		if (cml::determinant(in) < 0) {
			out_S = -1 * out_S;
		}

		// and remove all scaling from the matrix
		if(out_S[0])
		{
			vRows[0] /= out_S[0];
		}
		if(out_S[1])
		{
			vRows[1] /= out_S[1];
		}
		if(out_S[2])
		{
			vRows[2] /= out_S[2];
		}

		// build a 3x3 rotation matrix
		cml::matrix33d m(vRows[0][0],vRows[1][0],vRows[2][0],
			vRows[0][1],vRows[1][1],vRows[2][1],
			vRows[0][2],vRows[1][2],vRows[2][2]);

		// and generate the rotation quaternion from it
		out_R = cml::QuaternionMatrix(m);
	}

	void RSdecompose(const cml::matrix33d &in, cml::quaterniond &out_R, cml::vector3d &out_S)
	{
		// extract the rows of the matrix
		cml::vector3d vRows[3] = {
			cml::vector3d(in(0, 0),in(1, 0),in(2, 0)),
			cml::vector3d(in(0, 1),in(1, 1),in(2, 1)),
			cml::vector3d(in(0, 2),in(1, 2),in(2, 2))
		};

		// extract the scaling factors
		out_S[0] = vRows[0].length();
		out_S[1] = vRows[1].length();
		out_S[2] = vRows[2].length();

		// and the sign of the scaling

		if (cml::determinant(in) < 0) {
			out_S = -1 * out_S;
		}

		// and remove all scaling from the matrix
		if(out_S[0])
		{
			vRows[0] /= out_S[0];
		}
		if(out_S[1])
		{
			vRows[1] /= out_S[1];
		}
		if(out_S[2])
		{
			vRows[2] /= out_S[2];
		}

		// build a 3x3 rotation matrix
		cml::matrix33d m(vRows[0][0],vRows[1][0],vRows[2][0],
			vRows[0][1],vRows[1][1],vRows[2][1],
			vRows[0][2],vRows[1][2],vRows[2][2]);

		// and generate the rotation quaternion from it
		out_R = cml::QuaternionMatrix(m);
	}

	void Eigendecompose(const cml::matrixd &in, cml::vectord& out_eigenvalues, std::vector< cml::vectord > &out_eigenvectors, bool descending_sort)
	{
		if (in.cols() != in.rows())
		{
			std::cerr << "in.cols() != in.rows() : Eigendecompose()" << std::endl;
			return;
		}

		int n = in.cols();
		if (n == 0)
		{
			std::cerr << "in.cols() == 0 : Eigendecompose()" << std::endl;
			return;
		}

		cml::matrixd U, V;

		SVdecompose(in, U, out_eigenvalues, V);

		out_eigenvectors.resize(n);
		for (int i = 0; i<n; i++)
		{
			out_eigenvectors[i].resize(n);

			for (int j = 0; j<n; j++)
			{
				out_eigenvectors[i][j] = V(i, j);
			}
		}

		if (descending_sort)
		{
			// sort by eigenvalues
			for (int i = 1; i<n; i++)
				for (int j = i; j>0; j--)
				{
					if (out_eigenvalues[j] > out_eigenvalues[j - 1])
					{
						double tmp_d = out_eigenvalues[j];
						out_eigenvalues[j] = out_eigenvalues[j - 1];
						out_eigenvalues[j - 1] = tmp_d;

						cml::vectord tmp_v = out_eigenvectors[j];
						out_eigenvectors[j] = out_eigenvectors[j - 1];
						out_eigenvectors[j - 1] = tmp_v;
					}
				}
		}
	}

	void SVdecompose(const cml::matrixd &in, cml::matrixd &out_U, cml::vectord& out_W, cml::matrixd &out_V)
	{
		out_U = in;

		cml::matrixd &a = out_U;
		int m = a.rows();
		int n = a.cols();

		cml::vectord &w = out_W;
		cml::matrixd &v = out_V;

		w.resize(n);
		v.resize(n, n);

		int flag, i, its, j, jj, k, l, nm;
		double anorm, c, f, g, h, s, scale, x, y, z;

		static cml::vectord rv1; rv1.resize(n);
		g = scale = anorm = 0.0;

		for (i = 0; i<n; i++)
		{
			l = i + 1;
			rv1[i] = scale * g;
			g = s = scale = 0.0;

			if (i<m)
			{
				for (k = i; k<m; k++)
					scale += fabs(a(k, i));

				if (scale)
				{
					for (k = i; k<m; k++)
					{
						a(k, i) /= scale;
						s += a(k, i) * a(k, i);
					}

					f = a(i, i);
					g = -SIGN(sqrt(s), f);
					h = f * g - s;
					a(i, i) = f - g;

					for (j = l; j<n; j++)
					{
						for (s = 0.0, k = i; k<m; k++)
							s += a(k, i) * a(k, j);
						f = s / h;

						for (k = i; k<m; k++)
							a(k, j) += f * a(k, i);
					}

					for (k = i; k<m; k++)
						a(k, i) *= scale;
				}
			}

			w[i] = scale * g;
			g = s = scale = 0.0;

			if (i<m && i != n - 1)
			{
				for (k = l; k<n; k++)
					scale += fabs(a(i, k));

				if (scale)
				{
					for (k = l; k<n; k++)
					{
						a(i, k) /= scale;
						s += a(i, k) * a(i, k);
					}

					f = a(i, l);
					g = -SIGN(sqrt(s), f);
					h = f * g - s;
					a(i, l) = f - g;

					for (k = l; k<n; k++)
						rv1[k] = a(i, k) / h;

					for (j = l; j<m; j++)
					{
						for (s = 0.0, k = l; k<n; k++)
							s += a(j, k) * a(i, k);

						for (k = l; k<n; k++)
							a(j, k) += s * rv1[k];
					}

					for (k = l; k<n; k++)
						a(i, k) *= scale;
				}
			}

			anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
		}

		for (i = n - 1; i >= 0; i--)
		{
			if (i<n - 1)
			{
				if (g)
				{
					for (j = l; j<n; j++)
						v(j, i) = (a(i, j) / a(i, l)) / g;

					for (j = l; j<n; j++)
					{
						for (s = 0.0, k = l; k<n; k++)
							s += a(i, k) * v(k, j);

						for (k = l; k<n; k++)
							v(k, j) += s * v(k, i);
					}
				}

				for (j = l; j<n; j++)
					v(i, j) = v(j, i) = 0.0;
			}

			v(i, i) = 1.0;
			g = rv1[i];
			l = i;
		}

		for (i = MIN(m, n) - 1; i >= 0; i--)
		{
			l = i + 1;
			g = w[i];
			for (j = l; j<n; j++)
				a(i, j) = 0.0;

			if (g)
			{
				g = 1.0 / g;
				for (j = l; j<n; j++)
				{
					for (s = 0.0, k = l; k<m; k++)
						s += a(k, i) * a(k, j);

					f = (s / a(i, i)) * g;

					for (k = i; k<m; k++)
						a(k, j) += f * a(k, i);
				}

				for (j = i; j<m; j++)
					a(j, i) *= g;
			}
			else
				for (j = i; j<m; j++)
					a(j, i) = 0.0;

			++a(i, i);
		}

		for (k = n - 1; k >= 0; k--)
		{
			for (its = 1; its<30; its++)
			{
				flag = 1;
				for (l = k; l >= 0; l--)
				{
					nm = l - 1;
					if ((double)(fabs(rv1[l]) + anorm) == anorm)
					{
						flag = 0;
						break;
					}

					if ((double)(fabs(w[nm]) + anorm) == anorm) break;
				}

				if (flag)
				{
					c = 0.0;
					s = 1.0;

					for (i = l; i <= k; i++)
					{
						f = s * rv1[i];
						rv1[i] = c * rv1[i];

						if ((double)(fabs(f) + anorm) == anorm) break;

						g = w[i];
						h = pythag(f, g);
						w[i] = h;
						h = 1.0f / h;
						c = g * h;
						s = -f * h;

						for (j = 0; j<m; j++)
						{
							y = a(j, nm);
							z = a(j, i);
							a(j, nm) = y * c + z * s;
							a(j, i) = z * c - y * s;
						}
					}
				}

				z = w[k];
				if (l == k)
				{
					if (z < 0.0)
					{
						w[k] = -z;
						for (j = 0; j<n; j++)
							v(j, k) = -v(j, k);
					}
					break;
				}

				//if (its == 29)
				//	std::cerr << "no convergence in 30 svdcmp iterations" << std::endl;

				x = w[l];
				nm = k - 1;
				y = w[nm];
				g = rv1[nm];
				h = rv1[k];
				f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0f * h * y);
				g = pythag(f, 1.0f);
				f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
				c = s = 1.0f;

				for (j = l; j <= nm; j++)
				{
					i = j + 1;
					g = rv1[i];
					y = w[i];
					h = s * g;
					g = c * g;
					z = pythag(f, h);
					rv1[j] = z;
					c = f / z;
					s = h / z;
					f = x * c + g * s;
					g = g * c - x * s;
					h = y * s;
					y *= c;

					for (jj = 0; jj<n; jj++)
					{
						x = v(jj, j);
						z = v(jj, i);
						v(jj, j) = x * c + z * s;
						v(jj, i) = z * c - x * s;
					}

					z = pythag(f, h);
					w[j] = z;

					if (z)
					{
						z = 1.0f / z;
						c = f * z;
						s = h * z;
					}

					f = c * g + s * y;
					x = c * y - s * g;

					for (jj = 0; jj<m; jj++)
					{
						y = a(jj, j);
						z = a(jj, i);
						a(jj, j) = y * c + z * s;
						a(jj, i) = z * c - y * s;
					}
				}

				rv1[l] = 0.0;
				rv1[k] = f;
				w[k] = x;
			}
		}
	}












	cml::vector3d operator*(const cml::vector3d &a, const cml::vector3d &b) 
	{
		cml::vector3d c;
		c[0] = a[0] * b[0];
		c[1] = a[1] * b[1];
		c[2] = a[2] * b[2];
		return c;
	}

	cml::vector3d operator/(const cml::vector3d &a, const cml::vector3d &b)
	{
		cml::vector3d c;
		c[0] = a[0] / b[0];
		c[1] = a[1] / b[1];
		c[2] = a[2] / b[2];
		return c;
	}

	cml::vector3d operator/(const double &a, const cml::vector3d &b)
	{
		cml::vector3d c;
		c[0] = a / b[0];
		c[1] = a / b[1];
		c[2] = a / b[2];
		return c;
	}

	cml::vector3d operator/(const float &a, const cml::vector3d &b)
	{
		cml::vector3d c;
		c[0] = a / b[0];
		c[1] = a / b[1];
		c[2] = a / b[2];
		return c;
	}



	cml::vectord& solve( cml::matrixd const& a, cml::vectord const& b, double tolerance)
	{
		int m = a.rows();
		int n = a.cols();

		assert( m >= n );
		assert( b.size()==m );

		cml::vectord c;
		c.resize( n );

		static cml::matrixd u; u.resize( m, n );
		static cml::vectord w; w.resize( n );
		static cml::matrixd v; v.resize( n, n );

		u = a;
		cml::SVdecompose(u, u, w, v);
		// u.SVdecompose( w, v );

		int i, j;
		double s;
		static cml::vectord tmp; tmp.resize( n );

		double wmax = 0.0f;
		for( j=0; j<n; j++ )
			if ( w[j] > wmax ) wmax = w[j];

		for( j=0; j<n; j++ )
			if ( w[j] < wmax * tolerance ) w[j] = 0.0f;

		for( j=0; j<n; j++ )
		{
			s = 0.0f;
			if ( w[j] )
			{
				for( i=0; i<m; i++ )
					s += u(i, j) * b[i];
				s /= w[j];
			}
			tmp[j] = s;
		}

		for ( j=0; j<n; j++ )
		{
			s = 0.0;
			for ( i=0; i<n; i++ )
				s += v(j, i) * tmp[i];
			c[j] = s;
		}

		return c;
	}
}