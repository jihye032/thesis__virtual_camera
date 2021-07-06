#ifdef ODE_EXT


#include <ode/ode.h>
#include "BaseLib/CmlExt/CmlExt.h"
#include "BaseLib/OdeExt/CollisionWrapper.h"
#include "BaseLib/OdeExt/OdeExt.h"

static dQuaternion g_dq_a;
static dQuaternion g_dq_b;

static cml::vector3d x_axis(1., 0., 0.);
static cml::vector3d y_axis(0., 1., 0.);
static cml::vector3d z_axis(0., 0., 1.);

bool dCollide(const mg::PrimitiveShape* a, const mg::PrimitiveShape *b)
{
	dContactGeom out_contact;
	int r = dCollide(a, b, out_contact);

	if ( r == 0 ) return false;
	return true;
}

int dCollide(const mg::PrimitiveShape* a, const mg::PrimitiveShape *b, dContactGeom &out_contact)
{
	return dCollide(a, b, 0x00000001, &out_contact, sizeof(dContactGeom));
}

int dCollide(const mg::PrimitiveShape* a, const mg::PrimitiveShape *b, int flags, dContactGeom *contact, int skip)
{
	/// dGeom
	dGeomID d_geom_a = OdeCreateDGeom(0, a);
	dGeomID d_geom_b = OdeCreateDGeom(0, b);


	/// Rotation
	cml::quaterniond q_a = a->global_rotation();

	if ( a->type() == mg::PrimitiveShape::CYLINDER )
	{
		mg::PrimitiveCylinder *cyl = (mg::PrimitiveCylinder *)a;

		if ( cyl->direction() == 1 )
		{
			q_a = q_a * cml::EXP(x_axis*M_PI/4);
		}
		else if ( cyl->direction() == 0 )
		{
			q_a = q_a * cml::EXP(y_axis*M_PI/4);
		}
	}
	else if ( a->type() == mg::PrimitiveShape::CAPSULE )
	{
		mg::PrimitiveCapsule *cap = (mg::PrimitiveCapsule *)a;

		if ( cap->direction() == 1 )
		{
			q_a = q_a * cml::EXP(x_axis*M_PI/4);
		}
		else if ( cap->direction() == 0 )
		{
			q_a = q_a * cml::EXP(y_axis*M_PI/4);
		}
	}


	cml::quaterniond q_b = b->global_rotation();

	if ( b->type() == mg::PrimitiveShape::CYLINDER )
	{
		mg::PrimitiveCylinder *cyl = (mg::PrimitiveCylinder *)b;

		if ( cyl->direction() == 1 )
		{
			q_b = q_b * cml::EXP(x_axis*M_PI/4);
		}
		else if ( cyl->direction() == 0 )
		{
			q_b = q_b * cml::EXP(y_axis*M_PI/4);
		}
	}
	else if ( b->type() == mg::PrimitiveShape::CAPSULE )
	{
		mg::PrimitiveCapsule *cap = (mg::PrimitiveCapsule *)b;

		if ( cap->direction() == 1 )
		{
			q_b = q_b * cml::EXP(x_axis*M_PI/4);
		}
		else if ( cap->direction() == 0 )
		{
			q_b = q_b * cml::EXP(y_axis*M_PI/4);
		}
	}

	for ( int i=0; i<4; i++ )
	{
		g_dq_a[i] = q_a[i];
		g_dq_b[i] = q_b[i];
	}

	dGeomSetQuaternion(d_geom_a, g_dq_a);
	dGeomSetQuaternion(d_geom_b, g_dq_b);


	/// Position
	const cml::vector3d &p_a = a->global_translation();
	const cml::vector3d &p_b = b->global_translation();
				
	dGeomSetPosition(d_geom_a, p_a[0], p_a[1], p_a[2]);
	dGeomSetPosition(d_geom_b, p_b[0], p_b[1], p_b[2]);



	int result = dCollide(d_geom_a, d_geom_b, flags, contact, skip);

	dGeomDestroy(d_geom_a);
	dGeomDestroy(d_geom_b);

	return result;

}




int dCollide(const mg::PrimitiveShape* a, const mg::PrimitiveShape *b, mg::ContactInfo &out_contact)
{
	dContactGeom dcontact;
	int contacts_num = dCollide(a, b, dcontact);

	out_contact.depth_ = dcontact.depth;
	out_contact.pos_ = ode2cml(dcontact.pos);
	out_contact.normal_ = ode2cml(dcontact.normal);

	return contacts_num;
}

int dCollide(const mg::PrimitiveShape* a, const mg::PrimitiveShape *b, std::vector<mg::ContactInfo> &out_contacts, unsigned int max_contacts_num)
{
	dContactGeom *dcontacts = new dContactGeom[max_contacts_num];


	int contacts_num = dCollide(a, b, max_contacts_num, dcontacts, sizeof(dContactGeom));

	out_contacts.resize(contacts_num);
	for ( int i=0; i<contacts_num; i++ )
	{
		out_contacts[i].depth_ = dcontacts[i].depth;
		out_contacts[i].pos_ = ode2cml(dcontacts[i].pos);
		out_contacts[i].normal_ = ode2cml(dcontacts[i].normal);
	}

	delete[] dcontacts;

	return contacts_num;
}






#endif