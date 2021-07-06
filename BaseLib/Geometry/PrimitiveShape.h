
#pragma once

#include "BaseLib/CmlExt/CmlExt.h"
#include "BaseLib/Geometry/Mesh.h"
#include "BaseLib/Geometry/PrimitiveCollision.h"
#include "BaseLib/Geometry/SolidObject.h"
#include <fstream>
#include <string>

namespace mg
{

class PrimitiveSphere;
class PrimitiveCapsule;
class PrimitiveBox;

class PrimitiveShape
{
public:
	enum Type {NONE, BOX, SPHERE, CYLINDER, CAPSULE};
	PrimitiveShape();

	virtual void BuildMesh(Mesh &out_mesh) = 0;
	virtual void BuildSolidObject(SolidObject &out_obj) = 0;

	/// Collision Check.
	// The direction of out_penetration_depth is toward to 'this' from the 'target'.
	bool CheckCollision(PrimitiveShape *target, ContactInfo &out_collision_info);
	bool CheckCollision(PrimitiveShape *target, cml::vector3d &out_penetration_depth);
	bool CheckCollision(PrimitiveShape *target);
	
	virtual PrimitiveShape* CreateClone() const = 0;
	virtual void CopyFrom(PrimitiveShape *p) = 0;

	virtual void ScaleShape(double s) = 0;

	virtual void TranslateLocally(cml::vector3d t);
	virtual void RotateLocally(cml::quaterniond t);

	virtual void TranslateGlobally(cml::vector3d t);
	virtual void RotateGlobally(cml::quaterniond t);

	void ResetTransform();

	virtual PrimitiveSphere CalculBoundingSphere() const = 0;

	void local_translation(cml::vector3d t) { local_translation_ = t; }
	void local_rotation(cml::quaterniond r)    { local_rotation_ = r; }
	void translation(cml::vector3d t)       { translation_ = t; }
	void rotation(cml::quaterniond q)          { rotation_ = q; }

	cml::vector3d local_translation() const { return local_translation_; }
	cml::quaterniond local_rotation() const    { return local_rotation_; }
	cml::vector3d translation() const       { return translation_; }
	cml::quaterniond rotation() const          { return rotation_; }
	cml::vector3d global_translation() const;
	cml::quaterniond global_rotation() const;


	virtual Type type() const { return type_; }

	virtual void WriteBinary(std::string &filename) { std::ofstream out(filename, std::ios::binary); WriteBinary(out); out.close(); }
	virtual void WriteBinary(std::ostream &out)
	{
		out.write((char*)&local_translation_, sizeof(local_translation_));
		out.write((char*)&local_rotation_,    sizeof(local_rotation_));
		out.write((char*)&translation_,       sizeof(translation_));
		out.write((char*)&rotation_,          sizeof(rotation_));
		out.write((char*)&type_,              sizeof(type_));
	}

	virtual void ReadBinary(std::string &filename)  { std::ifstream in(filename, std::ios::binary); ReadBinary(in); in.close(); }
	virtual void ReadBinary(std::istream &in)
	{
		in.read((char*)&local_translation_, sizeof(local_translation_));
		in.read((char*)&local_rotation_,    sizeof(local_rotation_));
		in.read((char*)&translation_,       sizeof(translation_));
		in.read((char*)&rotation_,          sizeof(rotation_));
		in.read((char*)&type_,              sizeof(type_));
	}

protected:
	cml::vector3d local_translation_;
	cml::quaterniond local_rotation_;

	cml::vector3d translation_;
	cml::quaterniond rotation_;

	Type type_;
};


class PrimitiveBox : public PrimitiveShape
{
public:
	PrimitiveBox();
	PrimitiveBox(double width, double height, double depth);

	virtual void BuildMesh(Mesh &out_mesh);
	virtual void BuildSolidObject(SolidObject &out_obj);
	virtual PrimitiveShape* CreateClone() const;
	virtual void CopyFrom(PrimitiveShape *p);

	virtual PrimitiveSphere CalculBoundingSphere() const;

	double width()  const { return width_; }
	double height() const { return height_; }
	double depth()  const { return depth_; }

	void width(double w)  { width_ = w; }
	void height(double h) { height_ = h; }
	void depth(double d)  { depth_ = d; }

	void ScaleShape(double d);

	virtual void WriteBinary(std::ostream &out)     
	{ 
		PrimitiveShape::WriteBinary(out);
		out.write( (char*)&width_,  sizeof(width_)); 
		out.write( (char*)&height_, sizeof(height_)); 
		out.write( (char*)&depth_,  sizeof(depth_)); 
	}

	virtual void ReadBinary(std::istream &in) 
	{
		PrimitiveShape::ReadBinary(in);
		in.read( (char*)&width_,  sizeof(width_)); 
		in.read( (char*)&height_, sizeof(height_)); 
		in.read( (char*)&depth_,  sizeof(depth_)); 
	}

protected:
	double width_;
	double height_;
	double depth_;
};



class PrimitiveSphere : public PrimitiveShape
{
public:
	PrimitiveSphere();
	PrimitiveSphere(double radius);

	virtual void BuildMesh(Mesh &out_mesh);
	virtual void BuildSolidObject(SolidObject &out_obj);
	virtual PrimitiveShape* CreateClone()  const;
	virtual void CopyFrom(PrimitiveShape *p);

	virtual PrimitiveSphere CalculBoundingSphere() const;

	double radius()  const { return radius_; }
	void radius(double r)  { radius_ = r; }

	void ScaleShape(double d);

	virtual void WriteBinary(std::ostream &out)     
	{ 
		PrimitiveShape::WriteBinary(out);
		out.write( (char*)&radius_,  sizeof(radius_)); 
	}

	virtual void ReadBinary(std::istream &in) 
	{
		PrimitiveShape::ReadBinary(in);
		in.read( (char*)&radius_,  sizeof(radius_)); 
	}

protected:
	double radius_;
};


class PrimitiveCylinder : public PrimitiveShape
{
public:
	PrimitiveCylinder();
	PrimitiveCylinder(double height, double radius);

	virtual void BuildMesh(Mesh &out_mesh);
	virtual void BuildSolidObject(SolidObject &out_obj);
	virtual PrimitiveShape* CreateClone()  const;
	virtual void CopyFrom(PrimitiveShape *p);

	virtual PrimitiveSphere CalculBoundingSphere() const;

	double radius()  const { return radius_; }
	double height()  const { return height_; }
	int direction()  const { return direction_; }

	void radius(double r)  { radius_ = r; }
	void height(double h)  { height_ = h; }
	void direction(int d) { direction_ = d; }

	void ScaleShape(double d);

	virtual void WriteBinary(std::ostream &out)     
	{ 
		PrimitiveShape::WriteBinary(out);
		out.write( (char*)&radius_,  sizeof(radius_)); 
		out.write( (char*)&height_,  sizeof(height_)); 
		out.write( (char*)&direction_,  sizeof(direction_)); 
	}

	virtual void ReadBinary(std::istream &in) 
	{
		PrimitiveShape::ReadBinary(in);
		in.read( (char*)&radius_,  sizeof(radius_)); 
		in.read( (char*)&height_,  sizeof(height_)); 
		in.read( (char*)&direction_,  sizeof(direction_)); 
	}

protected:
	double radius_;
	double height_;
	int direction_; // 0 -> x axis, 1 -> y axis, 2 -> z axis
};



class PrimitiveCapsule : public PrimitiveShape
{
public:
	PrimitiveCapsule();
	PrimitiveCapsule(double clinder_height, double radius);

	virtual void BuildMesh(Mesh &out_mesh);
	virtual void BuildSolidObject(SolidObject &out_obj);
	virtual PrimitiveShape* CreateClone()  const;
	virtual void CopyFrom(PrimitiveShape *p);

	virtual PrimitiveSphere CalculBoundingSphere() const;

	void GetGlobalCylinderEndPoints(cml::vector3d &out_p0, cml::vector3d &out_p1) const;
	void SetGlobalCylinderEndPoints(cml::vector3d in_p0, cml::vector3d in_p1);

	double radius()  const { return radius_; }
	double cylinder_height()  const { return cylinder_height_; }
	double full_height() const { return cylinder_height_+2*radius_; }
	int direction()  const { return direction_; }

	void radius(double r)  { radius_ = r; }
	void cylinder_height(double h)  { cylinder_height_ = h; }
	void direction(int d) { direction_ = d; }

	void ScaleShape(double d);


	virtual void WriteBinary(std::ostream &out)     
	{ 
		PrimitiveShape::WriteBinary(out);
		out.write( (char*)&radius_,  sizeof(radius_)); 
		out.write( (char*)&cylinder_height_,  sizeof(cylinder_height_)); 
		out.write( (char*)&direction_,  sizeof(direction_)); 
	}

	virtual void ReadBinary(std::istream &in) 
	{
		PrimitiveShape::ReadBinary(in);
		in.read( (char*)&radius_,  sizeof(radius_)); 
		in.read( (char*)&cylinder_height_,  sizeof(cylinder_height_)); 
		in.read( (char*)&direction_,  sizeof(direction_)); 
	}


protected:
	double radius_;
	double cylinder_height_;
	int direction_; // 0 -> x axis, 1 -> y axis, 2 -> z axis
};



class PrimitiveComposition
{
public:
	PrimitiveComposition();
	PrimitiveComposition(const PrimitiveComposition &o);
	virtual ~PrimitiveComposition();

	virtual void AddPrimitiveShape(const mg::PrimitiveShape &p);
	int CountPrimitives() const { return shapes_.size(); }
	const PrimitiveShape* primitive(int i) const { return shapes_[i]; }

	//virtual void BuildMesh(Mesh &out_mesh) = 0;
	//virtual void BuildSolidObject(SolidObject &out_obj) = 0;

	virtual PrimitiveSphere CalculBoundingSphere() const;

	/// Collision Check.
	// The direction of out_penetration_depth is toward to 'this' from the 'target'.
	bool CheckCollision(PrimitiveShape *target, cml::vector3d &out_penetration_depth) const;
	bool CheckCollision(PrimitiveShape *target, ContactInfo &out_contact_info) const;
	bool CheckCollision(PrimitiveShape *target) const;
	int CheckCollision(PrimitiveShape *target, std::vector<ContactInfo> &out_contact_infos) const;
		
	virtual void TranslateLocally(cml::vector3d t);
	virtual void RotateLocally(cml::quaterniond t);

	virtual void TranslateGlobally(cml::vector3d t);
	virtual void RotateGlobally(cml::quaterniond t);

	void ResetTransform();

	void local_translation(cml::vector3d t) { local_translation_ = t; }
	void local_rotation(cml::quaterniond r)    { local_rotation_ = r; }
	void translation(cml::vector3d t)       { translation_ = t; }
	void rotation(cml::quaterniond q)          { rotation_ = q; }

	cml::vector3d local_translation() const { return local_translation_; }
	cml::quaterniond local_rotation() const    { return local_rotation_; }
	cml::vector3d translation() const       { return translation_; }
	cml::quaterniond rotation() const          { return rotation_; }
	cml::vector3d global_translation() const;
	cml::quaterniond global_rotation() const;

	void CopyFrom(const PrimitiveComposition &o);
	PrimitiveComposition& operator= (const PrimitiveComposition &o);


	virtual void WriteBinary(std::string &filename);
	virtual void WriteBinary(std::ostream &out);

	virtual void ReadBinary(std::string &filename);
	virtual void ReadBinary(std::istream &in);

protected:
	bool CheckCollision(int primitive_id, PrimitiveShape *target, ContactInfo &out_contact_info) const;



protected:
	std::vector<PrimitiveShape*> shapes_;

	cml::vector3d local_translation_;
	cml::quaterniond local_rotation_;

	cml::vector3d translation_;
	cml::quaterniond rotation_;
};


};
