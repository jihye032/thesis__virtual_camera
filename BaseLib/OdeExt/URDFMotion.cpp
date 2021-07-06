#ifdef ODE_EXT

#include "BaseLib/OdeExt/UrdfMotion.h"
#include <sstream>
#include "BaseLib/CmlExt/CmlExt.h"
#include <set>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <BaseLib/Algorithm/Signal1D.h>
#include <BaseLib/Algorithm/Signal3D.h>
#include <BaseLib/Geometry/Line.h>



static double safe_double(double val)
{
	#ifdef WIN32
	if (_isnan(val) || !_finite(val))
		return 0.0;
	else
		return val;
	#else
	if (isnan(val) || isinf(val))
		return 0.0;
	else
		return val;

	#endif
};	




static std::string trim(const std::string& str,
                 const std::string& whitespace = " \r\n\t")
{
	const auto strBegin = str.find_first_not_of(whitespace);
	if (strBegin == std::string::npos)
    return ""; // no content

    const auto strEnd = str.find_last_not_of(whitespace);
    const auto strRange = strEnd - strBegin + 1;

    return str.substr(strBegin, strRange);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Class: UrdfPose
UrdfPose::UrdfPose()
{
	root_transf_.setTranslation(::vector(0, 0, 0));
	root_transf_.setRotation(::quater(1, 0, 0, 0));
	urdf_ = 0;
}

UrdfPose::UrdfPose(const UrdfPose &src)
{
	root_transf_.setTranslation(::vector(0, 0, 0));
	root_transf_.setRotation(::quater(1, 0, 0, 0));
	urdf_ = 0;
	src.CopyTo(*this);
}

UrdfPose::UrdfPose(Urdf* a)
{
	root_transf_.setTranslation(::vector(0, 0, 0));
	root_transf_.setRotation(::quater(1, 0, 0, 0));
	urdf(a);
}

void
UrdfPose::urdf(Urdf *a)
{
	urdf_ = a;

	while ( (int)angles_.size() < a->num_joints() )
	{
		angles_.push_back(0);
	}

	while ( (int)angles_.size() > a->num_joints() )
	{
		angles_.pop_back();
	}
}



::vector
UrdfPose::GetGlobalJointPosition(int i)  const
{
	if ( i < 0 || (int)urdf_->num_joints() <= (int)i ) return ::vector(0, 0, 0);

	return GetGlobalJointPosition(urdf_->joint(i));
}



::vector
UrdfPose::GetGlobalJointPosition(std::string joint_name) const
{
	return GetGlobalJointPosition(urdf_->joint(joint_name));
}

::vector
UrdfPose::GetGlobalJointPosition(UrdfJoint* joint) const
{
	if ( joint==0 ) return ::vector(0, 0, 0);

	::vector t = joint->translation_;

	joint = urdf_->GetParentJointOfJoint(joint);
	while ( joint != 0 )
	{
		t = rotate( joint->orientation_, t );
		t = rotate( exp((angle(joint->name_)/2.0) * joint->axis_), t );
		t += joint->translation_;
		joint = urdf_->GetParentJointOfJoint(joint);
	}

	t = rotate( root_transf_.getRotation(), t ) + root_transf_.getTranslation();

	return t;
}


::vector
UrdfPose::GetGlobalCenterOfMass(int link_id) const
{
	if ( link_id < 0 || (int)urdf_->num_links() <= (int)link_id ) return ::vector(0, 0, 0);

	return GetGlobalCenterOfMass(urdf_->link(link_id));
}


::vector
UrdfPose::GetGlobalCenterOfMass(std::string link_name) const
{
	return GetGlobalCenterOfMass(urdf_->link(link_name));
}

::vector
UrdfPose::GetGlobalCenterOfMass(UrdfLink* link) const
{
	if ( link==0 ) return ::vector(0, 0, 0);

	UrdfJoint *parent_joint = urdf_->joint(link->parent_joint_name_);
	
	::vector t = GetGlobalJointPosition(parent_joint);

	t += rotate(GetGlobalJointRotation(parent_joint), link->center_of_mass_);

	return t;
}

::quater
UrdfPose::GetGlobalJointRotation(int i) const
{
	if ( i < 0 || (int)urdf_->num_joints() <= (int)i ) return ::quater(1, 0, 0, 0);

	return GetGlobalJointRotation(urdf_->joint(i));
}



::quater
UrdfPose::GetGlobalJointRotation(std::string joint_name) const
{
	return GetGlobalJointRotation(urdf_->joint(joint_name));
}

::quater
UrdfPose::GetGlobalJointRotation(UrdfJoint* joint) const
{
	if ( joint==0 ) return ::quater(1, 0, 0, 0);

	::quater r = joint->orientation_;
	r = exp((angle(joint->name_)/2.0) * joint->axis_) * r;

	joint = urdf_->GetParentJointOfJoint(joint);
	while ( joint != 0 )
	{
		r = joint->orientation_ * r;
		r = exp((angle(joint->name_)/2.0) * joint->axis_) * r;
		joint = urdf_->GetParentJointOfJoint(joint);
	}

	r = root_transf_.getRotation() * r;

	return r;
}



void
UrdfPose::Blend(const UrdfPose &a_pose, const UrdfPose &b_pose, double a_weight, double b_weight)
{
	Blend(a_pose, b_pose, a_weight/(a_weight+b_weight));
}

void
UrdfPose::BlendWith(const UrdfPose &b_pose, double b_weight)
{
	UrdfPose a_pose;
	this->CopyTo(a_pose);
	Blend(a_pose, b_pose, b_weight);
}

void
UrdfPose::Blend(const UrdfPose &a_pose, const UrdfPose &b_pose, double b_weight)
{
	urdf(a_pose.urdf());
	
	for ( unsigned int i=0; i<a_pose.angles_.size(); i++ )
	{
		this->angles_[i] = (1-b_weight)*a_pose.angles_[i] 
							+ b_weight*b_pose.angles_[i];
	}

	this->root_transf_ = ::interpolate(b_weight, a_pose.root_transf_, b_pose.root_transf_);
}




void
UrdfPose::CopyTo(UrdfPose &target_pose) const
{
	target_pose.urdf(urdf_);

	target_pose.angles_.assign(this->angles_.begin(), this->angles_.end());
	target_pose.position_constraints_.assign(this->position_constraints_.begin(), this->position_constraints_.end());

	target_pose.root_transf_ = this->root_transf_;
}

UrdfPose&
UrdfPose::operator=(const UrdfPose &src_pose)
{
	src_pose.CopyTo(*this);
	return *this;
}


void
UrdfPose::CalculSupportPolygon(QmPolygon2D &out_polygon) const
{
	// Temporal Quick Implementation.
	// It will works only for the postures of which only both feet are contacting on the ground. 
	// Make just a rectangle.
	::vector l_foot_v = GetGlobalJointPosition("l_leg_lax").getXZ();
	::vector r_foot_v = GetGlobalJointPosition("r_leg_lax").getXZ();

	l_foot_v += -(l_foot_v-r_foot_v)*0.1;
	r_foot_v += -(r_foot_v-l_foot_v)*0.1;


	::position l_foot_p = vector2position(l_foot_v);
	::position r_foot_p = vector2position(r_foot_v);
	
	::vector long_side_v = r_foot_p - l_foot_p;
	::vector short_side_v(0, 0, 0);
	short_side_v[0] = -long_side_v[2];
	short_side_v[2] = long_side_v[0];
	short_side_v *= 0.8;

	out_polygon.setNumVertices(5);
	
	::position temp_p = r_foot_p + 0.4*short_side_v;
	out_polygon.setVertex(0, temp_p);
	out_polygon.setVertex(4, temp_p);
	
	temp_p = temp_p + -1*long_side_v;
	out_polygon.setVertex(1, temp_p);
	
	temp_p = temp_p + -1*short_side_v;
	out_polygon.setVertex(2, temp_p);
	
	temp_p = temp_p + long_side_v;
	out_polygon.setVertex(3, temp_p);
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////
// IK
void
UrdfPose::AddPositionConstraint(int joint_id, ::vector p)
{
	if ( joint_id < 0 || joint_id >= (int)angles_.size() ) return;

	UrdfConstraint con;
	con.joint_id_ = joint_id;
	con.type_ = UrdfConstraint::CON_POSITION;
	con.position_ = p;
	position_constraints_.push_back( con );
}

void
UrdfPose::AddPositionConstraint(std::string joint_name, ::vector p)
{
	AddPositionConstraint( urdf_->joint_id(joint_name), p );
}

void
UrdfPose::AddDisplacement(const ::vectorN &d)
{
	int d_i = 0;
	for ( unsigned int i=0; i<angles_.size(); i++ )
	{
		if ( urdf_->is_active_joint(i) )
		{
			if ( (int)d.size() <= d_i ) break;
			angles_[i] += d[d_i];
			d_i++;
		}
	}
}

void
UrdfPose::PushAnglesInBound()
{
	for ( int j=0; j<urdf_->num_joints(); j++ )
	{
		if ( angle(j) < urdf_->joint(j)->limit_lower_ )
			angle(j, urdf_->joint(j)->limit_lower_+0.01);

		if ( angle(j) > urdf_->joint(j)->limit_upper_ )
			angle(j, urdf_->joint(j)->limit_upper_-0.01);
	}
}




/////////////////////////////////////////////////////////////////////////////////////////////////////
// Class: UrdfMotion
UrdfMotion::UrdfMotion()
{
	urdf_ = 0;
}

UrdfMotion::UrdfMotion(Urdf* a)
{
	urdf(a);
}

void
UrdfMotion::urdf(Urdf* a)
{
	urdf_ = a;

	for ( unsigned int i=0; i<poses_.size(); i++ )
	{
		poses_[i].urdf(a);
	}
}

void
UrdfMotion::num_poses(int n)
{
	while ( (int)poses_.size() < n )
	{
		poses_.push_back(UrdfPose(urdf_));
	}
	
	while ( (int)poses_.size() > n )
	{
		poses_.pop_back();
	}

	while ( (int)times_.size() < n )
	{
		if ( times_.empty() ) times_.push_back(0);
		else times_.push_back(times_.back());
	}

	while ( (int)times_.size() > n )
	{
		times_.pop_back();
	}
}

void
UrdfMotion::GetPose(double time, UrdfPose& out_pose) const
{
	if ( poses_.empty() ) return;

	if ( time <= times_.front() ) 
	{
		poses_.front().CopyTo(out_pose);
	}
	else if ( time >= times_.back() )
	{
		poses_.back().CopyTo(out_pose);
	}
	else 
	{
		int a_pose_index = 0;
		double a_weight = 0.0;

		for ( unsigned int i=1; i<times_.size(); i++ )
		{
			if ( time < times_[i] )
			{
				a_pose_index = i-1;
				a_weight = 1 - ( (time-times_[i-1]) / (times_[i]-times_[i-1]) );
				break;
			}
		}

		out_pose.Blend( poses_[a_pose_index], poses_[a_pose_index+1], 1-a_weight );
	}
}


::vector
UrdfMotion::CalculGlobalVelAtCoM(int frame, int link_id) const
{
	if ( link_id < 0 || link_id >= urdf_->num_links() ) return ::vector(0, 0, 0);
	if ( num_poses() <= 1 ) return ::vector(0, 0, 0);

	UrdfLink *link = urdf_->link(link_id);
	if ( !link ) return ::vector(0, 0, 0);

	int f1, f2;

	if ( frame == 0 )
	{
		f1 = 0;
		f2 = 1;
	}
	else if ( frame == num_poses()-1 )
	{
		f1 = frame-1;
		f2 = frame;
	} 
	else if ( frame > 0 && frame < num_poses()-1 )
	{
		f1 = frame-1;
		f2 = frame+1;
	}
	else
	{
		return ::vector(0, 0, 0);
	}

	double dt;
	dt = times_[f2]-times_[f1];

	return ( poses_[f2].GetGlobalCenterOfMass(link) - poses_[f1].GetGlobalCenterOfMass(link) ) 
				/ dt;
}

::vector
UrdfMotion::CalculGlobalAccAtCoM(int frame, int link_id) const
{
	if ( link_id < 0 || link_id >= urdf_->num_links() ) return ::vector(0, 0, 0);
	if ( num_poses() <= 2 ) return ::vector(0, 0, 0);

	UrdfLink *link = urdf_->link(link_id);
	if ( !link ) return ::vector(0, 0, 0);

	int f1, f2;

	if ( frame == 0 )
	{
		f1 = 0;
		f2 = 1;
	}
	else if ( frame == num_poses()-1 )
	{
		f1 = frame-1;
		f2 = frame;
	}
	else if ( frame > 0 && frame < num_poses()-1 )
	{
		f1 = frame-1;
		f2 = frame+1;
	}
	else
	{
		return ::vector(0, 0, 0);
	}

	::vector v1, v2;
	v1 = CalculGlobalVelAtCoM(f1, link_id);
	v2 = CalculGlobalVelAtCoM(f2, link_id);

	double t1, t2;
	t1 = times_[f1];
	t2 = times_[f2];

	double dt;
	dt = t2-t1;

	::vector a = (v2-v1)/dt;

	return a;
}

::vector
UrdfMotion::CalculZMP(int frame, ::vector gravity) const
{
	// Ref:: Paper 'Physical Touch-Up of Human Motions.

	if ( frame < 0 || frame >= num_poses() ) return :: vector(0, 0, 0);

	static std::vector<::vector> com_accs_; // Accelerations of Center of Masses of linkes.
	static std::vector<::vector> com_ps_; // Position of Center of Masses of linkes.
	static std::vector<double> masses_; // Masses of linkes.
	unsigned int num_links = urdf_->num_links();

	if ( com_accs_.size() < num_links )
	{
		com_accs_.resize(num_links);
	}
	if ( com_ps_.size() < num_links )
	{
		com_ps_.resize(num_links);
	}
	if ( masses_.size() < num_links )
	{
		masses_.resize(num_links);
	}

	for ( unsigned int i=0; i<num_links; i++ )
	{
		com_accs_[i] = CalculGlobalAccAtCoM(frame, i);
		com_ps_[i] = poses_[frame].GetGlobalCenterOfMass(i);
		//masses_[i] = 100*urdf_->link(i)->mass_;
		masses_[i] = urdf_->link(i)->mass_;
		
	}

	double sum_of_y_force = 0;
	for ( unsigned int i=0; i<num_links; i++ )
	{
		sum_of_y_force += masses_[i]*(com_accs_[i].y()-gravity.y());
	}

	double zmp_x = 0;
	double zmp_z = 0;
	for ( unsigned int i=0; i<num_links; i++ )
	{
		zmp_x += masses_[i]	* 
			( ( com_accs_[i].y()-gravity.y() )*com_ps_[i].x() 
			- ( com_accs_[i].x()-gravity.x() )*com_ps_[i].y() );

		zmp_z += masses_[i]	* 
			( ( com_accs_[i].y()-gravity.y() )*com_ps_[i].z() 
			- ( com_accs_[i].z()-gravity.z() )*com_ps_[i].y() );
	}
	zmp_x /= sum_of_y_force;
	zmp_z /= sum_of_y_force;

	return ::vector(zmp_x, 0, zmp_z);
}

bool
UrdfMotion::CheckZMPSafety(int frame, ::vector &out_zmp, ::vector &out_safe_zmp, ::vector gravity) const
{
	if ( frame < 0 || frame >= num_poses() ) return false;

	out_zmp = CalculZMP(frame, gravity);
	::position zmp_p(out_zmp.x(), 0, out_zmp.z());

	QmPolygon2D support_poly;
	poses_[frame].CalculSupportPolygon(support_poly);

	if ( support_poly.inclusion(zmp_p) )
	{
		out_safe_zmp = position2vector(zmp_p);
		return true;
	}
	else
	{
		::position safe_zmp_p = support_poly.gradient(zmp_p) + zmp_p;
		out_safe_zmp = position2vector(safe_zmp_p);
		return false;
	}
}


void
UrdfMotion::ApplySmoothFilter()
{
	for ( int i=0; i<urdf_->num_joints(); i++ )
	{
		ApplySmoothFilter(i);
	}

	PushAnglesInBound();
}

void
UrdfMotion::PushAnglesInBound()
{
	// Check the Motion Ranges. 
	for ( int f=0; f<num_poses(); f++ )
	{
		poses_[f].PushAnglesInBound();
	}

}

void
UrdfMotion::ApplySmoothFilterArms()
{
	for ( int i=0; i<urdf_->num_joints(); i++ )
	{
		for ( int j=0; j<urdf_->num_joints(); j++ )
		{
			ApplySmoothFilter(j);
		}
	}

	// Check the Motion Ranges. 
	PushAnglesInBound();

}

void
UrdfMotion::ApplySmoothFilter(int joint_id)
{
	if ( joint_id < 0 ) return;
	if ( joint_id >= urdf_->num_joints() ) return;

	UrdfJoint *joint = urdf_->joint(joint_id);

	mg::Signal1D angles;

	for ( int i=0; i<num_poses(); i++ )
	{
		angles.push_back(poses_[i].angle(joint_id));
	}

	angles.makeSmooth();
	angles.makeSmooth();
	angles.makeSmooth();
	angles.makeSmooth();
	angles.makeSmooth();

	for ( int i=0; i<num_poses(); i++ )
	{
		poses_[i].angle(joint_id, angles[i]);
	}
}

double
UrdfMotion::AdjustUpperBodyForBalance(::vector gravity, double param_k)
{
	return AdjustUpperBodyForBalance(0, num_poses()-1, gravity, param_k);
}

double
UrdfMotion::AdjustUpperBodyForBalance(int start_frame, int end_frame, ::vector gravity, double param_k)
{
	//std::vector<bool> unsafe_poses;
	mg::Signal3D to_safe_zones;
	double error = 0;
	

	//unsafe_poses.resize(num_poses(), false);
	to_safe_zones.resize(num_poses());

	// Check Balance and Calculate safe zmp
	for ( int f=start_frame; f<=end_frame; f++ )
	{
		::vector safe_zmp;
		::vector zmp;
		bool safe = CheckZMPSafety(f, zmp, safe_zmp, gravity);
		//unsafe_poses[f] = !safe;
		if ( safe )
			to_safe_zones[f] = ::vector(0, 0, 0);
		else
		{
			to_safe_zones[f] = safe_zmp-zmp;
			error += len(safe_zmp-zmp);
		}
	}
	to_safe_zones.makeSmooth();

	// Solve Ik for all frames
	static const int num_ik_joint = 2;
	static const char *ik_joint_names[num_ik_joint] = {"l_arm_mwx", "r_arm_mwx"};
	//for ( int f=start_frame; f<=end_frame; f++ )
	for ( int f=0; f<num_poses(); f++ )
	{
		//if ( !unsafe_poses[f] ) continue;
		if ( to_safe_zones[f].length() < EPS ) continue;

		UrdfPose &cur_pose = pose(f);
		cur_pose.RemoveAllConstraint();
		UrdfPose backup_pose;
		cur_pose.CopyTo(backup_pose);

		for ( int j=0; j<urdf_->num_joints(); j++ )
		{
			if ( !urdf_->is_active_joint(j) ) continue;

			if ( (urdf_->joint_name(j).find("arm") != std::string::npos)
				|| ( urdf_->joint_name(j).compare("neck_ay") == 0 )
				|| ( urdf_->joint_name(j).compare("back_ubx") == 0 )
				|| ( urdf_->joint_name(j).compare("back_mby") == 0 )
				|| ( urdf_->joint_name(j).compare("back_lbz") == 0 )
				)
			{
				cur_pose.AddPositionConstraint(j, cur_pose.GetGlobalJointPosition(j)+to_safe_zones[f]*param_k);
			}
			else if ( 
				( urdf_->joint_name(j).compare("neck_ay") != 0 )
				&& ( urdf_->joint_name(j).compare("back_ubx") != 0 )
				&& ( urdf_->joint_name(j).compare("back_mby") != 0 )
				&& ( urdf_->joint_name(j).compare("back_lbz") != 0 )
				)
			{
				cur_pose.AddPositionConstraint(j, cur_pose.GetGlobalJointPosition(j));
			}
		}

		cur_pose.SolveIK();

		//cur_pose.root_transf(backup_pose.root_translation(), backup_pose.root_rotation());
		for ( int j=0; j<urdf_->num_joints(); j++ )
		{
			if ( urdf_->joint_name(j).find("_leg") != std::string::npos )
			{
				cur_pose.angle(j, backup_pose.angle(j));
			}
		}
	}

	return error;
}



void
UrdfMotion::SaveYaml(std::string yaml_file_name, std::string section_name)
{
	static char *urdf_joint_names[28] = {
	  "back_lbz",  "back_mby",  "back_ubx",  "neck_ay",
	  "l_leg_uhz", "l_leg_mhx", "l_leg_lhy", "l_leg_kny", "l_leg_uay", "l_leg_lax",
	  "r_leg_uhz", "r_leg_mhx", "r_leg_lhy", "r_leg_kny", "r_leg_uay", "r_leg_lax",
	  "l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx",
	  "r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx"};

	std::ofstream fout(yaml_file_name);

	fout << section_name << ":" << std::endl;
	for ( int f=0; f<num_poses(); f++ )
	{
		if ( f<num_poses()-1 )
			fout << "  - [" << times_[f+1]-times_[f] <<", \"";
		else 
			fout << "  - [" << 0 <<", \"";

		for ( int j=0; j<28; j++ )
		{
			fout << std::setiosflags(std::ios::fixed) << std::setprecision(4) << safe_double( pose(f).angle( urdf_joint_names[j] ) ) <<  " ";
		}
		fout << " \" ]" << std::endl;
	}
	fout.close();
}


void
UrdfMotion::LoadYaml(std::string yaml_file_name, std::string section_name)
{
	static const int urdf_joint_num = 28;
	static const char *urdf_joint_names[urdf_joint_num] = {
		  "back_lbz",  "back_mby",  "back_ubx",  "neck_ay",
		  "l_leg_uhz", "l_leg_mhx", "l_leg_lhy", "l_leg_kny", "l_leg_uay", "l_leg_lax",
		  "r_leg_uhz", "r_leg_mhx", "r_leg_lhy", "r_leg_kny", "r_leg_uay", "r_leg_lax",
		  "l_arm_usy", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_uwy", "l_arm_mwx",
		  "r_arm_usy", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_uwy", "r_arm_mwx"};



	std::ifstream fin(yaml_file_name);
	std::string word;
	char line[1024];
	std::vector<std::string> key_frame_strs;

	// Reading
	while ( !fin.eof() && !fin.fail() )
	{			fin >> word;
		if ( fin.fail() ) break;
		if ( word.empty() ) continue;
		if ( word.front() == '#' ) fin.getline(line, 1024);

		if ( word.back() == ':' )
		{
			word.erase(word.end()-1);

			if ( section_name.empty() || section_name.compare(word) == 0 )
			{
				fin.getline(line, 1024);
				while (1)
				{
					fin.getline(line, 1024);
					if ( fin.eof() || fin.fail() ) break;
					std::string line_str(line);
					std::size_t first_non_space_i = line_str.find_first_not_of(" \r\t");
					if ( first_non_space_i == std::string::npos ) continue;
					if ( line_str[first_non_space_i] != '-' ) break;
				
					key_frame_strs.push_back(line_str);
				}
			
				break;
			}
		}
	}




	fin.close();



	// Parsing
	num_poses((int)key_frame_strs.size());
	for ( std::size_t i=0; i<key_frame_strs.size(); i++ )
	{
		// time
		{
			std::size_t s = key_frame_strs[i].find_first_not_of(" \r\t-[");
			std::size_t e = key_frame_strs[i].find_first_of(",");
			if ( s==std::string::npos || e==std::string::npos
				|| e<=s )
			{
				std::cout << "Error while loading a Yaml 0"; 
				return;
			}
			std::string time_word(key_frame_strs[i].begin()+s, key_frame_strs[i].begin()+e);
			std::stringstream stream(time_word);
			double t;
			stream >> t; 
			if ( i==0 ) time(i, t);
			else time(i, time(i-1)+t);
		}

		// angles.
		{
			std::size_t s = key_frame_strs[i].find_first_of("\"");
			std::size_t e = key_frame_strs[i].find_last_of("\"");
			if ( s==std::string::npos || e==std::string::npos )
			{
				std::cout << "Error while loading a Yaml 1"; 
				return;
			}
			s = s+1;
			if  ( e<=s )
			{
				std::cout << "Error while loading a Yaml 2"; 
				return;
			}

			std::string angles_str(key_frame_strs[i].begin()+s, key_frame_strs[i].begin()+e);
			std::stringstream stream(angles_str);

			double a;
			for ( int j=0; j<urdf_joint_num; j++ )
			{
				stream >> a; 
				if ( stream.fail() )
				{
					std::cout << "Error while loading a Yaml 3"; 
					return;
				}
				poses_[i].angle(urdf_joint_names[j], a);
			}
		}
	}
		
}






#endif
