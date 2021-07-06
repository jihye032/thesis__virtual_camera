
#include "BaseLib/Geometry/PrimitiveShape.h"
#include "BaseLib/Geometry/Line.h"
#include "BaseLib/OdeExt/CollisionWrapper.h"

namespace mg
{
	static cml::vector3d x_axis(1., 0., 0.);
	static cml::vector3d y_axis(0., 1., 0.);
	static cml::vector3d z_axis(0., 0., 1.);

PrimitiveShape::PrimitiveShape()
: local_translation_(0, 0, 0)
, local_rotation_(1, 0, 0, 0)
, translation_(0, 0, 0)
, rotation_(1, 0, 0, 0)
, type_(NONE)
{
}

void
PrimitiveShape::TranslateLocally(cml::vector3d t)
{
	local_translation_ += t;
}

void
PrimitiveShape::RotateLocally(cml::quaterniond r)
{
	local_translation_ = cml::Rotate(r, local_translation_);
	local_rotation_ = r*local_rotation_;
}

void
PrimitiveShape::TranslateGlobally(cml::vector3d t)
{
	translation_ += t;
}

void
PrimitiveShape::RotateGlobally(cml::quaterniond r)
{
	translation_ = cml::Rotate(r, translation_);
	rotation_ = r*rotation_;
}

void
PrimitiveShape::ResetTransform()
{
	local_translation_ = cml::vector3d(0, 0, 0);
	local_rotation_ = cml::quaterniond(1, 0, 0, 0);

	translation_ = cml::vector3d(0, 0, 0);;
	rotation_ = cml::quaterniond(1, 0, 0, 0);;
}

cml::vector3d 
PrimitiveShape::global_translation() const
{
	cml::vector3d t = local_translation_;
	t = cml::Rotate(rotation_, t);
	t += translation_;
	return t;
}

cml::quaterniond 
PrimitiveShape::global_rotation() const
{
	return rotation_ * local_rotation_;
}


////////////////////////////////////////////////////////////////
// Class: BoxShape
PrimitiveBox::PrimitiveBox()
{
	type_ = BOX;
}

PrimitiveBox::PrimitiveBox(double width, double height, double depth)
{
	type_ = BOX;
	width_  = width;
	height_ = height;
	depth_  = depth;
}

void
PrimitiveBox::BuildMesh(Mesh &out_mesh)
{
	out_mesh.CreateBox( width_
					, height_
					, depth_);

	out_mesh.RotateVertices(local_rotation_);
	out_mesh.TranslateVertices(local_translation_);
	out_mesh.RotateVertices(rotation_);
	out_mesh.TranslateVertices(translation_);
}

void
PrimitiveBox::BuildSolidObject(SolidObject &out_obj)
{
	out_obj.editable_mesh()->CreateBox(width_, height_, depth_);

	out_obj.editable_mesh()->RotateVertices(local_rotation_);
	out_obj.editable_mesh()->TranslateVertices(local_translation_);
	out_obj.rotation(rotation_);
	out_obj.translation(translation_);
}

PrimitiveShape*
PrimitiveBox::CreateClone() const
{
	PrimitiveBox *box = new PrimitiveBox;
	*box = *this;
	return box;
}

void
PrimitiveBox::CopyFrom(PrimitiveShape* p)
{
	if ( type_ != p->type() ) return;

	PrimitiveBox *pp = (PrimitiveBox *)p;
	*this = *pp;
}

PrimitiveSphere
PrimitiveBox::CalculBoundingSphere() const
{
	double r =cml::length(cml::vector3d(width_/2, height_/2, depth_/2));

	PrimitiveSphere s(r);
	s.translation( global_translation() );

	return s;
}

void
PrimitiveBox::ScaleShape(double s)
{
	width_ *= s;
	height_ *= s;
	depth_ *= s;
}


////////////////////////////////////////////////////////////////
// Class: SphereShape
PrimitiveSphere::PrimitiveSphere()
{
	type_ = SPHERE;
}

PrimitiveSphere::PrimitiveSphere(double r)
{
	type_ = SPHERE;
	radius_ = r;
}

void
PrimitiveSphere::BuildMesh(Mesh &out_mesh)
{
	out_mesh.CreateSphere(radius_);

	out_mesh.RotateVertices(local_rotation_);
	out_mesh.TranslateVertices(local_translation_);
	out_mesh.RotateVertices(rotation_);
	out_mesh.TranslateVertices(translation_);
}

void
PrimitiveSphere::BuildSolidObject(SolidObject &out_obj)
{
	out_obj.editable_mesh()->CreateSphere(radius_);

	out_obj.editable_mesh()->RotateVertices(local_rotation_);
	out_obj.editable_mesh()->TranslateVertices(local_translation_);
	out_obj.rotation(rotation_);
	out_obj.translation(translation_);
}

PrimitiveShape*
PrimitiveSphere::CreateClone() const
{
	PrimitiveSphere *p = new PrimitiveSphere;
	*p = *this;
	return p;
}

void
PrimitiveSphere::CopyFrom(PrimitiveShape* p)
{
	if ( type_ != p->type() ) return;

	PrimitiveSphere *pp = (PrimitiveSphere *)p;
	*this = *pp;
}

PrimitiveSphere
PrimitiveSphere::CalculBoundingSphere() const
{
	return *this;
}

void
PrimitiveSphere::ScaleShape(double s)
{
	radius_ *= s;
}


////////////////////////////////////////////////////////////////
// Class: PrimitiveCylinder
PrimitiveCylinder::PrimitiveCylinder()
{
	type_ = CYLINDER;
	direction_ = 1; // lay on y-axis.
}

PrimitiveCylinder::PrimitiveCylinder(double h, double r)
{
	type_ = CYLINDER;
	height_ = h;
	radius_ = r;
	direction_ = 1; // lay on y-axis.
}

void
PrimitiveCylinder::BuildMesh(Mesh &out_mesh)
{
	out_mesh.CreateCylinder(height_, radius_);

	if ( direction_ == 0 )
		out_mesh.RotateVertices(cml::EXP(-1.0*M_PI/2.0*z_axis));
	else if ( direction_ == 2 )
		out_mesh.RotateVertices(cml::EXP(M_PI/2.0*x_axis));

	out_mesh.RotateVertices(local_rotation_);
	out_mesh.TranslateVertices(local_translation_);
	out_mesh.RotateVertices(rotation_);
	out_mesh.TranslateVertices(translation_);
}

void
PrimitiveCylinder::BuildSolidObject(SolidObject &out_obj)
{
	out_obj.editable_mesh()->CreateCylinder(height_, radius_);

	if ( direction_ == 0 )
		out_obj.editable_mesh()->RotateVertices(cml::EXP(-1.0*M_PI/2.0*z_axis));
	else if ( direction_ == 2 )
		out_obj.editable_mesh()->RotateVertices(cml::EXP(M_PI/2.0*x_axis));

	out_obj.editable_mesh()->RotateVertices(local_rotation_);
	out_obj.editable_mesh()->TranslateVertices(local_translation_);
	out_obj.rotation(rotation_);
	out_obj.translation(translation_);
}

PrimitiveShape*
PrimitiveCylinder::CreateClone() const
{
	PrimitiveCylinder *p = new PrimitiveCylinder;
	*p = *this;
	return p;
}

void
PrimitiveCylinder::CopyFrom(PrimitiveShape* p)
{
	if ( type_ != p->type() ) return;

	PrimitiveCylinder *pp = (PrimitiveCylinder *)p;
	*this = *pp;
}

PrimitiveSphere
PrimitiveCylinder::CalculBoundingSphere() const
{
	double r =cml::length(cml::vector3d(radius_/2, height_/2, radius_/2));

	PrimitiveSphere s(r);
	s.translation( global_translation() );

	return s;
}

void
PrimitiveCylinder::ScaleShape(double s)
{
	radius_ *= s;
	height_ *= s;
}



////////////////////////////////////////////////////////////////
// Class: PrimitiveCapsule
PrimitiveCapsule::PrimitiveCapsule()
{
	type_ = CAPSULE;
	cylinder_height_ = 1;
	radius_ = 1;
	direction_ = 1; // lay on y-axis.
}

PrimitiveCapsule::PrimitiveCapsule(double cylinder_h, double r)
{
	type_ = CAPSULE;
	cylinder_height_ = cylinder_h;
	radius_ = r;
	direction_ = 1; // lay on y-axis.
}

void
PrimitiveCapsule::BuildMesh(Mesh &out_mesh)
{
	out_mesh.CreateCapsule(cylinder_height_, radius_);

	
	if ( direction_ == 0 )
		out_mesh.RotateVertices(cml::EXP(-1.0*M_PI/2.0*z_axis));
	else if ( direction_ == 2 )
		out_mesh.RotateVertices(cml::EXP(M_PI/2.0*x_axis));

	out_mesh.RotateVertices(local_rotation_);
	out_mesh.TranslateVertices(local_translation_);
	out_mesh.RotateVertices(rotation_);
	out_mesh.TranslateVertices(translation_);
}

void
PrimitiveCapsule::BuildSolidObject(SolidObject &out_obj)
{
	out_obj.editable_mesh()->CreateCapsule(cylinder_height_, radius_);

	
	if ( direction_ == 0 )
		out_obj.editable_mesh()->RotateVertices(cml::EXP(-1.0*M_PI/2.0*z_axis));
	else if ( direction_ == 2 )
		out_obj.editable_mesh()->RotateVertices(cml::EXP(M_PI/2.0*x_axis));

	out_obj.editable_mesh()->RotateVertices(local_rotation_);
	out_obj.editable_mesh()->TranslateVertices(local_translation_);
	out_obj.rotation(rotation_);
	out_obj.translation(translation_);
}

void
PrimitiveCapsule::GetGlobalCylinderEndPoints(cml::vector3d &p0, cml::vector3d &p1) const
{
	p0 = cml::vector3d(0, 0, 0);
	p1 = cml::vector3d(0, 0, 0);

	p0[direction_] = -1*cylinder_height_/2;
	p1[direction_] = 1*cylinder_height_/2;

	p0 = cml::Rotate(local_rotation_, p0);
	p0 += local_translation_;
	p0 = cml::Rotate(rotation_, p0);
	p0 += translation_;

	p1 = cml::Rotate(local_rotation_, p1);
	p1 += local_translation_;
	p1 = cml::Rotate(rotation_, p1);
	p1 += translation_;
}

void
PrimitiveCapsule::SetGlobalCylinderEndPoints(cml::vector3d p0, cml::vector3d p1) 
{
	cml::vector3d v = p1-p0;
	cylinder_height_ =cml::length(v);
	direction_ = 1;

	rotation_ = local_rotation_ = cml::quaterniond(1, 0, 0, 0);
	translation_ = local_translation_ = cml::vector3d(0, 0, 0);

	TranslateGlobally(cml::vector3d(0, cylinder_height_*0.5, 0));
	RotateGlobally(cml::EXP(0.5 * atan2(cml::length(cml::cross(y_axis, v)), cml::dot(y_axis,v) ) * cml::normalize(cml::cross(y_axis, v))));
	TranslateGlobally(p0);
}

PrimitiveShape*
PrimitiveCapsule::CreateClone() const
{
	PrimitiveCapsule *p = new PrimitiveCapsule;
	*p = *this;
	return p;
}

void
PrimitiveCapsule::CopyFrom(PrimitiveShape* p)
{
	if ( type_ != p->type() ) return;

	PrimitiveCapsule *pp = (PrimitiveCapsule *)p;
	*this = *pp;
}

PrimitiveSphere
PrimitiveCapsule::CalculBoundingSphere() const
{
	//double r =cml::length(cml::vector3d(radius_/2, cylinder_height_/2+radius_, radius_/2));
	double r =cml::length(cml::vector3d(0, cylinder_height_/2+radius_, 0));

	PrimitiveSphere s(r);
	s.translation( global_translation() );

	return s;
}

void
PrimitiveCapsule::ScaleShape(double s)
{
	radius_ *= s;
	cylinder_height_ *= s;
}



bool
PrimitiveShape::CheckCollision(PrimitiveShape *target, ContactInfo &out_contact_info)
{
#ifdef ODE_EXT
	int contact_num = dCollide(this, target, out_contact_info);
	if ( contact_num>0 ) return true;
	else return false;
#else
	return mg::CheckCollision(this, target, out_contact_info);
	
#endif
}


bool
PrimitiveShape::CheckCollision(PrimitiveShape *target, cml::vector3d &out_penetration_depth)
{
#ifdef ODE_EXT
	ContactInfo c_info;
	int contact_num = dCollide(this, target, c_info);
	if ( contact_num<=0 ) return false;

	out_penetration_depth = c_info.normal_ * c_info.depth_;
	return true;

#else

	return mg::CheckCollision(this, target, out_penetration_depth);
#endif
}


bool
PrimitiveShape::CheckCollision(PrimitiveShape *target)
{
	cml::vector3d tmp;
	
	return CheckCollision(target, tmp);
}





//////////////////////////////////////////////////////////////////////////////
// class PrimitiveComposition

PrimitiveComposition::PrimitiveComposition()
: local_translation_(0, 0, 0)
, local_rotation_(1, 0, 0, 0)
, translation_(0, 0, 0)
, rotation_(1, 0, 0, 0)
{
}

PrimitiveComposition::PrimitiveComposition(const PrimitiveComposition &o)
{
	CopyFrom(o);
}

void
PrimitiveComposition::CopyFrom(const PrimitiveComposition &o)
{
	local_translation_ = o.local_translation_;
	local_rotation_ = o.local_rotation_;
	translation_ = o.translation_;
	rotation_ = o.rotation_;

	for ( unsigned int i=0; i<shapes_.size(); i++ )
	{
		delete shapes_[i];
	}
	shapes_.clear();

	for ( unsigned int i=0; i<o.shapes_.size(); i++ )
	{
		shapes_.push_back(o.shapes_[i]->CreateClone());
	}
}

PrimitiveComposition& 
PrimitiveComposition::operator= (const PrimitiveComposition &o)
{
	CopyFrom(o);
	return *this;
}


PrimitiveComposition::~PrimitiveComposition()
{
	for ( unsigned int i=0; i<shapes_.size(); i++ )
	{
		delete shapes_[i];
	}
	shapes_.clear();
}


void
PrimitiveComposition::AddPrimitiveShape(const mg::PrimitiveShape &p)
{
	shapes_.push_back(p.CreateClone());
}

PrimitiveSphere 
PrimitiveComposition::CalculBoundingSphere() const
{
	if ( shapes_.empty() ) return PrimitiveSphere();

	// This is not a perfect alorithm..

	std::vector<PrimitiveSphere> sub_spheres;

	for ( unsigned int i=0; i<shapes_.size(); i++ )
	{
		sub_spheres.push_back(shapes_[i]->CalculBoundingSphere());
	}


	// center
	cml::vector3d center(0, 0, 0);
	for ( unsigned int i=0; i<sub_spheres.size(); i++ )
	{
		center += sub_spheres[i].global_translation();
	}

	center /= shapes_.size();


	double radi = 0.0;
	for ( unsigned int i=0; i<sub_spheres.size(); i++ )
	{
		double d =cml::length(center-sub_spheres[i].global_translation())+sub_spheres[i].radius();
		if ( i==0 || radi < d )
		{
			radi = d;
		}

	}


	PrimitiveSphere bounding_s(radi);
	bounding_s.translation(center + global_translation());

	return  bounding_s;



}


bool 
PrimitiveComposition::CheckCollision(int primitive_id, PrimitiveShape *target, ContactInfo &out_contact_info) const
{
	PrimitiveShape *tmp_s = shapes_[primitive_id]->CreateClone();
	tmp_s->RotateGlobally(this->local_rotation());
	tmp_s->TranslateGlobally(this->local_translation());
	tmp_s->RotateGlobally(this->rotation());
	tmp_s->TranslateGlobally(this->translation());

	bool flag_collided = tmp_s->CheckCollision(target, out_contact_info);
	delete tmp_s;

	return flag_collided;
}



bool
PrimitiveComposition::CheckCollision(PrimitiveShape *target, cml::vector3d &out_penetration_depth) const
{
	ContactInfo contact_info;

	if ( CheckCollision(target, contact_info) )
	{
		out_penetration_depth = contact_info.normal_ * contact_info.depth_;
		return true;
	}
	

	return false;
}

bool
PrimitiveComposition::CheckCollision(PrimitiveShape *target) const
{
	ContactInfo contact_info;
	if ( CheckCollision(target, contact_info) )
	{
		return true;
	}

	return false;
}


bool
PrimitiveComposition::CheckCollision(PrimitiveShape *target, ContactInfo &out_contact_info) const
{
	std::vector<ContactInfo> contact_infos;
	CheckCollision(target, contact_infos);

	if ( contact_infos.empty() ) return false;

	// Find the deepest penetration.
	double max_depth = 0;
	int max_depth_i = 0;
	for ( unsigned int i=0; i<contact_infos.size(); i++ )
	{
		if ( max_depth < contact_infos[i].depth_ )
		{
			max_depth = contact_infos[i].depth_;
			max_depth_i = i;
		}
	}

	out_contact_info = contact_infos[max_depth_i];

	return true;
}

int
PrimitiveComposition::CheckCollision(PrimitiveShape *target, std::vector<ContactInfo> &out_contact_infos) const
{
	for ( unsigned int i=0; i<shapes_.size(); i++ )
	{
		ContactInfo contact_info;

		if ( CheckCollision(i, target, contact_info) )
		{
			out_contact_infos.push_back(contact_info);
		}
	}

	return (int)out_contact_infos.size();
}

void
PrimitiveComposition::TranslateLocally(cml::vector3d t)
{
	local_translation_ += t;
}

void
PrimitiveComposition::RotateLocally(cml::quaterniond r)
{
	local_translation_ = cml::Rotate(r, local_translation_);
	local_rotation_ = r*local_rotation_;
}

void
PrimitiveComposition::TranslateGlobally(cml::vector3d t)
{
	translation_ += t;
}

void
PrimitiveComposition::RotateGlobally(cml::quaterniond r)
{
	translation_ = cml::Rotate(r, translation_);
	rotation_ = r*rotation_;
}

void
PrimitiveComposition::ResetTransform()
{
	local_translation_ = cml::vector3d(0, 0, 0);
	local_rotation_ = cml::quaterniond(1, 0, 0, 0);

	translation_ = cml::vector3d(0, 0, 0);;
	rotation_ = cml::quaterniond(1, 0, 0, 0);;
}

cml::vector3d 
PrimitiveComposition::global_translation() const
{
	cml::vector3d t = local_translation_;
	t = cml::Rotate(rotation_, t);
	t += translation_;
	return t;
}

cml::quaterniond 
PrimitiveComposition::global_rotation() const
{
	return rotation_ * local_rotation_;
}



void 
PrimitiveComposition::WriteBinary(std::string &filename) 
{ 
	std::ofstream out(filename, std::ios::binary); 
	WriteBinary(out); 
	out.close(); 
}

void 
PrimitiveComposition::WriteBinary(std::ostream &out)     
{ 
	out.write( (char*)&local_translation_, sizeof(local_translation_)); 
	out.write( (char*)&local_rotation_,    sizeof(local_rotation_)); 
	out.write( (char*)&translation_,       sizeof(translation_)); 
	out.write( (char*)&rotation_,          sizeof(rotation_)); 

	int shapes_size = (int)shapes_.size();
	out.write((char*)&shapes_size, sizeof(int));


	for ( int i=0; i<shapes_size; i++ )
	{
		PrimitiveShape::Type t = shapes_[i]->type();
		out.write( (char*)&t, sizeof(PrimitiveShape::Type)); 
		
		shapes_[i]->WriteBinary(out);
	}

	
}

void 
PrimitiveComposition::ReadBinary(std::string &filename)  
{
	std::ifstream in(filename, std::ios::binary); 
	ReadBinary(in); 
	in.close(); 
}

void 
PrimitiveComposition::ReadBinary(std::istream &in)       
{ 
	
	in.read( (char*)&local_translation_, sizeof(local_translation_)); 
	in.read( (char*)&local_rotation_,    sizeof(local_rotation_)); 
	in.read( (char*)&translation_,       sizeof(translation_)); 
	in.read( (char*)&rotation_,          sizeof(rotation_)); 

	int shapes_size;
	in.read((char*)&shapes_size, sizeof(int));



	shapes_.resize(shapes_size);
	for ( int i=0; i<shapes_size; i++ )
	{
		PrimitiveShape::Type t;
		in.read((char*)&t, sizeof(PrimitiveShape::Type));

		if ( t == PrimitiveShape::BOX )
		{
			shapes_[i] = new PrimitiveBox;
			((PrimitiveBox*)shapes_[i])->ReadBinary(in);
		}
		else if ( t == PrimitiveShape::SPHERE )
		{
			shapes_[i] = new PrimitiveSphere;
			((PrimitiveSphere*)shapes_[i])->ReadBinary(in);
		}
		else if ( t == PrimitiveShape::CYLINDER )
		{
			shapes_[i] = new PrimitiveCylinder;
			((PrimitiveCylinder*)shapes_[i])->ReadBinary(in);
		}
		else if ( t == PrimitiveShape::CAPSULE )
		{
			shapes_[i] = new PrimitiveCapsule;
			((PrimitiveCapsule*)shapes_[i])->ReadBinary(in);
		}
		else
		{
		}

	}

	
}


};