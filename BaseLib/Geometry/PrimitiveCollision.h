
#pragma once

#include "BaseLib/CmlExt/CmlExt.h"

namespace mg
{


class PrimitiveShape;
class PrimitiveSphere;
class PrimitiveCapsule;
class PrimitiveBox;



class ContactInfo
{
public:
	double depth_; // penetrate depth;
	cml::vector3d pos_;
	cml::vector3d normal_;
};

bool CheckCollision(PrimitiveShape *A, PrimitiveShape *B, ContactInfo &out_contact_info);
bool CheckCollision(PrimitiveShape *A, PrimitiveShape *B, cml::vector3d &out_penetration_depth);
bool CheckCollisionCapCap(PrimitiveCapsule *A_cap, PrimitiveCapsule *B_cap, cml::vector3d &out_penetration_depth);
bool CheckCollisionSphSph(PrimitiveSphere *A_sph, PrimitiveSphere *B_sph, cml::vector3d &out_penetration_depth);
bool CheckCollisionCapSph(PrimitiveCapsule *A_cap, PrimitiveSphere *B_sph, cml::vector3d &out_penetration_depth);
bool CheckCollisionCapBox(PrimitiveCapsule *A_cap, PrimitiveBox *B_box, cml::vector3d &out_penetration_depth);
bool CheckCollisionSphBox(PrimitiveSphere *A_sph, PrimitiveBox *B_box, cml::vector3d &out_penetration_depth);

};



