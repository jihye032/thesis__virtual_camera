
#include "SolidObject.h"
#include <fstream>
#include <string>

namespace mg
{
///////////////////////////////////////////////////////////////
// Class: SolidObject
SolidObject::SolidObject()
: center_(0, 0, 0)
, translation_(0, 0, 0)
, rotation_(1, 0, 0, 0)
, scaling_(1, 1, 1)
{
	 
}

void
SolidObject::Clear()
{
	center_ = cml::vector3d(0, 0, 0);
	translation_ = cml::vector3d(0, 0, 0);
	rotation_ = cml::quaterniond(1, 0, 0, 0);
	scaling_ = cml::vector3d(1, 1, 1);
	name_ = "";
	mesh_.Clear();
}

void 
SolidObject::WriteObjFormatFile(std::string filename) const
{
	std::ofstream fout(filename.c_str());
	WriteObjFormatStream(fout);
	fout.close();
}

void 
SolidObject::ReadObjFormatFile(std::string filename)
{
	std::ifstream fin(filename.c_str());
	ReadObjFormatStream(fin);
	fin.close();
}

void 
SolidObject::WriteObjFormatStream(std::ostream &out) const
{
	mesh_.WriteObjFormatStreamV(out);
	mesh_.WriteObjFormatStreamVT(out);
	mesh_.WriteObjFormatStreamVN(out);
	mesh_.WriteObjFormatStreamG(out, name_);
	mesh_.WriteObjFormatStreamF(out);
}

void 
SolidObject::ReadObjFormatStream(std::istream &in)
{
	// Read Group Name.
	std::string name;
	{
		std::string read_line;
		while ( !in.eof() )
		{
			if ( in.fail() ) break;
			std::getline(in, read_line);
			if ( read_line.compare(0, 2, "g ") == 0 )
			{
				name.assign(read_line.begin()+2, read_line.end());
				break;
			}
		}
		in.clear();
		in.seekg(0);
	}

	// Truck Front and Rear Space.
	while ( !name.empty() && ((*name.begin()) == ' ' || (*name.begin()) == '\t' || (*name.begin()) == '\n') )
	{
		name.erase(name.begin());
	}
	while ( !name.empty() && (*(name.end()-1) == ' ' || *(name.end()-1) == '\t' || *(name.end()-1) == '\n') )
	{
		name.erase(name.end()-1);
	}

	this->name(name);

	mesh_.ReadObjFormatStream(in);
}

void
SolidObject::TranslateGlobally(cml::vector3d t)
{
	translation_ += t;
}

void
SolidObject::RotateGlobally(cml::quaterniond r)
{
	translation_ = cml::Rotate(r, translation_);
	rotation_ = r*rotation_;
}

void
SolidObject::ScaleGlobally(cml::vector3d s)
{
	for ( int i=0; i<3; i++ )
	{
		translation_[i] *= s[i];
		scaling_[i] *= s[i];
	}
}


///////////////////////////////////////////////////////////////
// Class: CompositeSolidObject
CompositeSolidObject::CompositeSolidObject()
{
}

void
CompositeSolidObject::AddObject(SolidObject *o)
{
	solid_objs_.push_back(o);
}

void
CompositeSolidObject::RevmoeObject(SolidObject *o)
{
	for ( unsigned int i=0; i<solid_objs_.size(); i++ )
	{
		if ( solid_objs_[i] == o )
		{
			solid_objs_.erase( solid_objs_.begin()+i );
			break;
		}
	}
}

void
CompositeSolidObject::RevmoeAllObjects()
{
	solid_objs_.clear();
}


void 
CompositeSolidObject::WriteObjFormatFile(std::string filename) const
{
	std::ofstream fout(filename.c_str());
	WriteObjFormatStream(fout);
	fout.close();
}

void 
CompositeSolidObject::ReadObjFormatFile(std::string filename)
{
	std::ofstream fout(filename.c_str());
	WriteObjFormatStream(fout);
	fout.close();
}

void 
CompositeSolidObject::WriteObjFormatStream(std::ostream &out) const
{

}

void 
CompositeSolidObject::ReadObjFormatStream(std::istream &in)
{
}


};









