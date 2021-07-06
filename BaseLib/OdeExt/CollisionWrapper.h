#ifdef ODE_EXT

#pragma once

#include "BaseLib/Geometry/PrimitiveShape.h"
#include "BaseLib/Geometry/PrimitiveCollision.h"
#include <vector>

struct dContactGeom;

bool dCollide(const mg::PrimitiveShape* a, const mg::PrimitiveShape *b);
int dCollide(const mg::PrimitiveShape* a, const mg::PrimitiveShape *b, dContactGeom &out_contact);
int dCollide(const mg::PrimitiveShape* a, const mg::PrimitiveShape *b, int flags, dContactGeom *contact, int skip);


int dCollide(const mg::PrimitiveShape* a, const mg::PrimitiveShape *b, mg::ContactInfo &out_contact);
int dCollide(const mg::PrimitiveShape* a, const mg::PrimitiveShape *b, std::vector<mg::ContactInfo> &out_contacts, unsigned int max_contacts_num=8);


#endif