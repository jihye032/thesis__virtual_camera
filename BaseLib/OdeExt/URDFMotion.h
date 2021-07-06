#ifdef ODE_EXT

#pragma once

#include <string>
#include "BaseLib/CmlExt/CmlExt.h"
#include <map>
#include <vector>
#include "BaseLib/tinyxml/tinyxml.h"
#include "BaseLib/Geometry/PrimitiveShape.h"
#include "PmQm/qmPolygon2D.h"
#include "BaseLib/OdeExt/URDF.h"


class UrdfConstraint
{
public:
	enum UrdfConType { CON_POSITION };

	int joint_id_;
	UrdfConType type_;
	::vector position_;
};

class UrdfPose
{
public:
	UrdfPose();
	UrdfPose(const UrdfPose &src_pose);
	UrdfPose(Urdf* a);

	Urdf* urdf() const { return urdf_; }
	void urdf(Urdf *a);

	double angle(int i) const { return angles_[i]; }
	double angle(std::string joint_name) const { return angles_[urdf_->joint_id(joint_name)]; }

	void angle(int i, double a) { angles_[i] = a; }
	void angle(std::string joint_name, double a) { angles_[urdf_->joint_id(joint_name)] = a; }

	::vector GetGlobalJointPosition(int id) const;
	::vector GetGlobalJointPosition(std::string joint_name) const;
	::vector GetGlobalJointPosition(UrdfJoint* j) const;

	::vector GetGlobalCenterOfMass(int link_id) const;
	::vector GetGlobalCenterOfMass(std::string link_name) const;
	::vector GetGlobalCenterOfMass(UrdfLink* link) const;

	::quater GetGlobalJointRotation(int id) const;
	::quater GetGlobalJointRotation(std::string joint_name) const;
	::quater GetGlobalJointRotation(UrdfJoint* j) const;

	void AddPositionConstraint(int joint_id, ::vector p);
	void AddPositionConstraint(std::string joint_name, ::vector p);
	void RemoveAllConstraint() { position_constraints_.clear(); }
	double SolveIKOnce(double ik_error_bound = 0.001, double ik_tolerance = 0.1);
	void SolveIK(double ik_error_bound = 0.001, double ik_tolerance = 0.1);

	void PushAnglesInBound();

	void Blend(const UrdfPose &a_pose, const UrdfPose &b_pose, double a_weight, double b_weight);
	void Blend(const UrdfPose &a_pose, const UrdfPose &b_pose, double a_weight);
	void BlendWith(const UrdfPose &b_pose, double b_weight);
	void CopyTo(UrdfPose &target_pose) const;
	UrdfPose& operator=(const UrdfPose &src_pose);

	void AddDisplacement(const ::vectorN &d);
	::transf root_transf() const { return root_transf_; }
	::vector root_translation() const { return root_transf_.getTranslation(); }
	::quater root_rotation() const { return root_transf_.getRotation(); }
	void root_transf(const ::vector &t, const ::quater &q) { root_transf_.setTranslation(t); root_transf_.setRotation(q); }
	void root_translation(::vector t) { root_transf_.setTranslation(t); }
	void root_rotation(::quater r) { root_transf_.setRotation(r); }

	void CalculSupportPolygon(QmPolygon2D &out_polygon) const;

protected:
	Urdf *urdf_;
	std::vector<double> angles_;

	transf root_transf_;

	std::vector< UrdfConstraint > position_constraints_;
};


class UrdfMotion
{
public:
	UrdfMotion();
	UrdfMotion(Urdf* a);

	Urdf* urdf() const { return urdf_; }
	void urdf(Urdf *a);

	int num_poses() const { return (int)poses_.size(); }
	void num_poses(int n);

	UrdfPose& pose(int f) { return poses_[f]; }
	void pose(int f, const UrdfPose &p) { poses_[f] = p; }
	void GetPose(double time, UrdfPose& out_pose) const;

	double time(int f) const { return times_[f]; }
	double last_time() const { return times_.back(); }
	double first_time() const { return times_.front(); }
	void time(int f, double t) { times_[f]=t; }

	void ApplySmoothFilter();
	void ApplySmoothFilterArms();
	void PushAnglesInBound();

	::vector CalculZMP(int frame, ::vector gravity=::vector(0, -9.8, 0)) const;
	bool CheckZMPSafety(int frame, ::vector &out_zmp, ::vector &out_safe_zmp, ::vector gravity=::vector(0, -9.8, 0)) const;
	::vector CalculGlobalVelAtCoM(int frame, int link_id) const;
	::vector CalculGlobalAccAtCoM(int frame, int link_id) const;
	double AdjustUpperBodyForBalance(::vector gravity=::vector(0, -9.8, 0), double param_k=0.05);
	double AdjustUpperBodyForBalance(int start_frame, int end_frame, ::vector gravity=::vector(0, -9.8, 0), double param_k=0.05);

	void LoadYaml(std::string yaml_file, std::string section_name="");
	void SaveYaml(std::string yaml_file, std::string section_name="");

protected:
	void ApplySmoothFilter(int joint_id);

protected:
	std::vector<double> times_;	// Actual time (millisecond) which each pose of poses_ should be appeared. 
								// The first time is usually 0.
	std::vector<UrdfPose> poses_;
	Urdf *urdf_;
};



#endif
