
#pragma once

#include "BaseLib/Geometry/Mesh.h"
#include <string>
#include <vector>
#include <iostream>

namespace mg
{
class SolidObject
{
public:
	SolidObject();
	void Clear();

	void WriteObjFormatFile(std::string filename) const;
	void ReadObjFormatFile(std::string filename);
	void WriteObjFormatStream(std::ostream &out) const;
	void ReadObjFormatStream(std::istream &in);
	
	void  center(const cml::vector3d &t) { center_ = t; }
	void  name(const std::string &name) { name_ = name; }
	Mesh* editable_mesh() { return &mesh_; }

	cml::vector3d    center() const { return center_; }
	std::string name() const { return name_; }
	const Mesh* mesh() const { return &mesh_; }

	void TranslateGlobally(cml::vector3d t); 
	void RotateGlobally(cml::quaterniond r); 
	void ScaleGlobally(cml::vector3d t); 

	void translation(cml::vector3d t)   { translation_ = t; }
	void rotation(cml::quaterniond q)      { rotation_ = q; }
	void scaling(cml::vector3d s)       { scaling_ = s; }
	void uniform_scaling(double s) { scaling_.set(s, s, s); }

	cml::vector3d translation() const { return translation_; }
	cml::quaterniond rotation() const    { return rotation_; }
	cml::vector3d scaling() const     { return scaling_; }

protected:
	cml::vector3d translation_;
	cml::quaterniond rotation_;
	cml::vector3d scaling_;

	cml::vector3d center_;
	Mesh mesh_;
	std::string name_;
};


class CompositeSolidObject
{
public:
	CompositeSolidObject();

	void AddObject(SolidObject *s);
	void RevmoeObject(SolidObject *s);
	void RevmoeAllObjects();
	int  CountObjects() const { return (int)solid_objs_.size(); }

	void WriteObjFormatFile(std::string filename) const;
	void ReadObjFormatFile(std::string filename);
	void WriteObjFormatStream(std::ostream &out) const;
	void ReadObjFormatStream(std::istream &in);

	void name(const std::string &name) { name_ = name; }
	std::vector<SolidObject*>& solid_objs() { return solid_objs_; }
	
	std::string  name() const { return name_; }
	SolidObject* solid_obj(int i) const { return solid_objs_[i]; }

protected:
	std::string name_;

	std::vector<SolidObject*> solid_objs_;
};

};