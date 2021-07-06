
#include "BaseLib/Algorithm/PCA.h"

namespace mg
{

void CalculPrincipalAxis2D(const std::vector<cml::vector2d> &data, cml::vector2d &out_axis)
{
	std::vector<cml::vector2d> out_axes;
	CalculPrincipalAxes2D(data, out_axes);

	out_axis = out_axes.front();
}

void CalculPrincipalAxes2D(const std::vector<cml::vector2d> &data, std::vector<cml::vector2d> &out_axes)
{
	cml::vectord tmp_eigenvalues;
	CalculPrincipalAxes2D(data, tmp_eigenvalues, out_axes);
}

void CalculPrincipalAxes2D(const std::vector<cml::vector2d> &data, cml::vectord &out_eigenvalues, std::vector<cml::vector2d> &out_axes)
{
	if ( data.empty() ) return;

	int m = (int)data.size();
	int n = 2;
	cml::matrixd mat(m, n);

	for ( int i=0; i<m; i++ )
		for ( int j=0; j<n; j++ )
		{
			mat(i, j) = data[i][j];
		}


	std::vector<cml::vectord> out_d_axes;
	CalculPrincipalAxes(mat, out_eigenvalues, out_d_axes);

	out_axes.resize(n);
	for ( int i=0; i<n; i++ )
	{
		for ( int j=0; j<n; j++ )
			out_axes[i][j] = out_d_axes[i][j];
	}
}

void CalculPrincipalAxes3D(const std::vector<cml::vector3d> &data, std::vector<cml::vector3d> &out_axes)
{
	cml::vectord tmp_eigenvalues;
	CalculPrincipalAxes3D(data, tmp_eigenvalues, out_axes);
}

void CalculPrincipalAxes3D(const std::vector<cml::vector3d> &data, cml::vectord &out_eigenvalues, std::vector<cml::vector3d> &out_axes)
{
	if ( data.empty() ) return;

	int m = (int)data.size();
	int n = 3;
	cml::matrixd mat(m, n);

	for ( int i=0; i<m; i++ )
		for ( int j=0; j<n; j++ )
			mat(i, j) = data[i][j];

	std::vector<cml::vectord> out_d_axes;
	CalculPrincipalAxes(mat, out_eigenvalues, out_d_axes);
	
	out_axes.resize(n);
	for ( int i=0; i<n; i++ )
	{
		for ( int j=0; j<n; j++ )
			out_axes[i][j] = out_d_axes[i][j];
	}
}

void CalculPrincipalAxes(const std::vector<cml::vectord> &data, cml::vectord &out_eigenvalues, std::vector<cml::vectord> &out_axes)
{
	if ( data.empty() ) return;

	int m = (int)data.size();
	int n = (int)data[0].size();
	cml::matrixd mat(m, n);

	for ( int i=0; i<m; i++ )
		for ( int j=0; j<n; j++ )
			mat(i, j) = data[i][j];


	CalculPrincipalAxes(mat, out_eigenvalues, out_axes);
}

/**
in_matrix: m by n. m > n
*/
void CalculPrincipalAxes(const cml::matrixd &in_matrix, cml::vectord &out_eigenvalues, std::vector<cml::vectord> &out_axes)
{
	int row_n = in_matrix.rows();
	int col_n = in_matrix.cols();

	// Covarience matrix
	cml::matrixd cov(col_n, col_n);
	{
		cml::vectord mean(col_n);
		{
			for ( int i=0; i<row_n; i++ )
				for ( int j=0; j<col_n; j++ )
					mean[j] += in_matrix(i, j);

			mean /= row_n;
		}

		cml::matrixd centered_matrix = in_matrix;
		{
			for ( int i=0; i<row_n; i++ )
				for ( int j=0; j<col_n; j++ )
			{
				centered_matrix(i, j) -= mean[j];
			}
		}
		
		cov = cml::transpose(centered_matrix) * centered_matrix;
	}

	// out_eigenvalues
	// out_eigenvectors
	{
		cml::Eigendecompose(cov, out_eigenvalues, out_axes);
	}


}

};
