
#pragma once


#include <cml/cml.h>

namespace cml
{
	const double EPS = 1.0e-4;
	cml::matrix44d MatrixRotationQuaternion(const cml::quaterniond &q);
	cml::matrix44d MatrixRotationEuler(const cml::vector3d &euler);
	cml::matrix44d MatrixRotationEuler(double x, double y, double z);
	cml::matrix44d MatrixRotationAxisAngle(const cml::vector3d &axis, double angle);

	cml::matrix44d MatrixTranslation(const cml::vector3d &t);
	cml::matrix44d MatrixTranslation(double x, double y, double z);

	cml::matrix44d MatrixUniformScaling(double s);
	cml::matrix44d MatrixScaling(const cml::vector3d &s);
	cml::matrix44d MatrixScaling(double x, double y, double z);


	cml::matrix44f MatrixRotationEuler(const cml::vector3f &euler);
	cml::matrix44f MatrixRotationEuler(float x, float y, float z);
	cml::matrix44f MatrixRotationAxisAngle(const cml::vector3f &axis, float angle);

	cml::matrix44f MatrixTranslation(const cml::vector3f &t);
	cml::matrix44f MatrixTranslation(float x, float y, float z);

	cml::matrix44f MatrixUniformScaling(float s);
	cml::matrix44f MatrixScaling(const cml::vector3f &s);
	cml::matrix44f MatrixScaling(float x, float y, float z);

	cml::quaterniond QuaternionMatrix(const cml::matrix44d &m);
	cml::quaterniond QuaternionMatrix(const cml::matrix33d &m);
	cml::quaterniond QuaternionEuler(const cml::vector3d &euler);

	cml::vector3d TranslationMatrix(const cml::matrix44d &m);

	cml::vector3d Rotate(const cml::quaterniond &q, const cml::vectord &in);

	cml::quaterniond EXP(const cml::vector3d v);
	cml::vector3d LOG(const cml::quaterniond q);

	cml::vector3d ToRAD(const cml::vector3d v);
	cml::vector3d ToDEG(const cml::vector3d v);

	/**
	TRS = Translation * Rotation * Scale.
	*/
	void TRSdecompose(const cml::matrix44d &in, cml::vector3d &out_T, cml::quaterniond &out_R, cml::vector3d &out_S);
	void RSdecompose(const cml::matrix33d &in, cml::quaterniond &out_R, cml::vector3d &out_S);
	void Eigendecompose(const cml::matrixd &in, cml::vectord& out_eigenvalues, std::vector< cml::vectord > &out_eigenvectors, bool descending_sort = true);
	void SVdecompose(const cml::matrixd &in, cml::matrixd &out_U, cml::vectord& out_W, cml::matrixd &out_V);

	/**
	Least Squares Sove
	*/
	cml::vectord& solve( cml::matrixd const& a, cml::vectord const& b, double tolerance=1.0e-2);


	cml::vector3d operator*(const cml::vector3d &a, const cml::vector3d &b);
	cml::vector3d operator/(const cml::vector3d &a, const cml::vector3d &b);
	cml::vector3d operator/(const double &a, const cml::vector3d &b);
	cml::vector3d operator/(const float &a, const cml::vector3d &b);


	
};