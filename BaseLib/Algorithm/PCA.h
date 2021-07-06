#pragma once

#include <vector>
#include "BaseLib/CmlExt/CmlExt.h"

namespace mg
{

	/**
	in_matrix: m by n. m > n
	*/
	void CalculPrincipalAxes(const cml::matrixd &in_matrix, cml::vectord &out_eigenvalues, std::vector<cml::vectord> &out_axes);

	void CalculPrincipalAxes(const std::vector<cml::vectord> &data, cml::vectord &out_eigenvalues, std::vector<cml::vectord> &out_axes);

	void CalculPrincipalAxis2D(const std::vector<cml::vector2d> &data, cml::vector2d &out_axis);
	void CalculPrincipalAxes2D(const std::vector<cml::vector2d> &data, std::vector<cml::vector2d> &out_axes);
	void CalculPrincipalAxes2D(const std::vector<cml::vector2d> &data, cml::vectord &out_eigenvalues, std::vector<cml::vector2d> &out_axes);

	void CalculPrincipalAxes3D(const std::vector<cml::vector3d> &data, std::vector<cml::vector3d> &out_axes);
	void CalculPrincipalAxes3D(const std::vector<cml::vector3d> &data, cml::vectord &out_eigenvalues, std::vector<cml::vector3d> &out_axes);
	

};
