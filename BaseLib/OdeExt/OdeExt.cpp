#ifdef ODE_EXT


#include "BaseLib/OdeExt/OdeExt.h"


dGeomID 
OdeCreateDGeom(dSpaceID dspace_id, const mg::PrimitiveShape *primit)
{
	dGeomID geom_id = 0;
	if ( primit->type() == mg::PrimitiveShape::BOX )
	{
		mg::PrimitiveBox *box = (mg::PrimitiveBox *)primit;
		geom_id = dCreateBox(dspace_id, box->width(), box->height(), box->depth());
		
	}
	else if ( primit->type() == mg::PrimitiveShape::CYLINDER )
	{
		mg::PrimitiveCylinder *cyl = (mg::PrimitiveCylinder *)primit;
		geom_id = dCreateCylinder(dspace_id, cyl->radius(), cyl->height());
	}
	else if ( primit->type() == mg::PrimitiveShape::SPHERE )
	{
		mg::PrimitiveSphere *sph = (mg::PrimitiveSphere *)primit;
		geom_id = dCreateSphere(dspace_id, sph->radius());
	}
	else if ( primit->type() == mg::PrimitiveShape::CAPSULE )
	{
		mg::PrimitiveCapsule *cap = (mg::PrimitiveCapsule *)primit;
		geom_id = dCreateCapsule(dspace_id, cap->radius(), cap->cylinder_height());
	}

	return geom_id;
}

dGeomID 
OdeCreateDGeom(dSpaceID dspace_id, const mg::PrimitiveShape *input_primitive, double in_density, dMass &out_mass)
{
	dGeomID dgeom_id = 0;
	if ( input_primitive->type() == mg::PrimitiveShape::BOX )
	{
		mg::PrimitiveBox *box = (mg::PrimitiveBox *)input_primitive;
		dgeom_id = dCreateBox(dspace_id, box->width(), box->height(), box->depth());
		dMassSetBox(&out_mass, in_density, box->width(), box->height(), box->depth());
	}
	else if ( input_primitive->type() == mg::PrimitiveShape::CYLINDER )
	{
		mg::PrimitiveCylinder *cyl = (mg::PrimitiveCylinder *)input_primitive;
		dgeom_id = dCreateCylinder(dspace_id, cyl->radius(), cyl->height());
		dMassSetCylinder(&out_mass, in_density, 3, cyl->radius(), cyl->height());
	}
	else if ( input_primitive->type() == mg::PrimitiveShape::SPHERE )
	{
		mg::PrimitiveSphere *sph = (mg::PrimitiveSphere *)input_primitive;
		dgeom_id = dCreateSphere(dspace_id, sph->radius());
		dMassSetSphere(&out_mass, in_density, sph->radius());
	}
	else if ( input_primitive->type() == mg::PrimitiveShape::CAPSULE )
	{
		mg::PrimitiveCapsule *cap = (mg::PrimitiveCapsule *)input_primitive;
		dgeom_id = dCreateCapsule(dspace_id, cap->radius(), cap->cylinder_height());
		dMassSetCapsule(&out_mass, in_density, 3, cap->radius(), cap->cylinder_height());
	}

	return dgeom_id;
}

cml::vector3d
OdeBodyDynamicState::dRealToVector(const dReal *d) const
{
	return cml::vector3d(d[0], d[1], d[2]);
}

cml::quaterniond
OdeBodyDynamicState::dRealToQuater(const dReal *d) const
{
	return cml::quaterniond(d[0], d[1], d[2], d[3]);
}

void
OdeBodyDynamicState::Set(const OdeBodyExt &body)
{
	::dCopyVector3(p_, dBodyGetPosition(body.id()));
	::dCopyVector4(q_, dBodyGetQuaternion(body.id()));
	::dCopyVector3(linear_v_, dBodyGetLinearVel(body.id()));
	::dCopyVector3(angular_v_, dBodyGetAngularVel(body.id()));
	::dCopyVector3(force_, dBodyGetForce(body.id()));
	::dCopyVector3(torque_, dBodyGetTorque(body.id()));
}













OdeBodyExt::OdeBodyExt(dWorldID dworld_id)
{
	offset_position_ = cml::vector3d(0, 0, 0);
	global_position_ = cml::vector3d(0, 0, 0);
	create(dworld_id);
}

OdeBodyExt::~OdeBodyExt()
{
	destroy();
}


void
OdeBodyExt::SetOrietationExt(cml::quaterniond q)
{
	dQuaternion dq; 
	dq[0]=q[0]; dq[1]=q[1]; dq[2]=q[2]; dq[3]=q[3]; 
	this->setQuaternion(dq);

	cml::vector3d g_o_p = global_position_ + cml::Rotate(q, offset_position_);
	setPosition(g_o_p[0], g_o_p[1], g_o_p[2]);
}

void
OdeBodyExt::SetPositionExt(cml::vector3d p)
{
	global_position_ = p;

	cml::vector3d g_o_p = global_position_ + cml::Rotate(orietation(), offset_position_);
	setPosition(g_o_p[0], g_o_p[1], g_o_p[2]);
}

void
OdeBodyExt::SetOffestPosition(cml::vector3d p)
{
	offset_position_ = p;

	cml::vector3d g_o_p = global_position_ + cml::Rotate(orietation(), offset_position_);
	setPosition(g_o_p[0], g_o_p[1], g_o_p[2]);
}

void
OdeBodyExt::SetCurrentAsInitialDynamics()
{
	init_dynamic_state_.Set(*this);
}

void
OdeBodyExt::ResetDynamics()
{
	this->setPosition(init_dynamic_state_.p_);
	this->setQuaternion(init_dynamic_state_.q_);
	this->setAngularVel(init_dynamic_state_.angular_v_);
	this->setLinearVel(init_dynamic_state_.linear_v_);
	this->setForce(init_dynamic_state_.force_);
	this->setTorque(init_dynamic_state_.torque_);
}

void
OdeBodyExt::CreateGeometry(dSpaceID dspace_id, mg::PrimitiveShape* s, double mass)
{
	std::vector<mg::PrimitiveShape*> ss;
	std::vector<double> ms;
	ss.push_back(s);
	ms.push_back(mass);
	CreateGeometries(dspace_id, ss, ms);
}

void
OdeBodyExt::CreateGeometries(dSpaceID dspace_id, const std::vector<mg::PrimitiveShape*> &primitives, const std::vector<double> &masses)
{
	DeleteGeometries();
	if ( primitives.empty() ) return;


	// The translation of the first primitive in the world coordinate becomes the global translation of this body.
	cml::vector3d body_translation;
	body_translation = primitives.front()->translation();


	dMass total_mass;
	dMassSetZero(&total_mass);

	std::vector<dGeomID> dgeom_t_ids;

	// Create Geom by combining multiple Bounding Volumes.
	for ( unsigned int i=0; i<primitives.size(); i++ )
	{
		// Create a Geom and a Mass
		dMass dmass;
		dGeomID dgeom_id = OdeCreateDGeom(0, primitives[i], 1, dmass);
		dMassAdjust(&dmass, masses[i]);

		// Set Positional and Rotational Offset of Geometry
		cml::vector3d p = primitives[i]->global_translation()-body_translation;
		cml::quaterniond q = primitives[i]->global_rotation();
		dQuaternion dq;	
		dq[0]=q[0];		dq[1]=q[1];		dq[2]=q[2];		dq[3]=q[3];
		dMatrix3 dr;
		dRfromQ(dr, dq);
		
		// set the transformation (adjust the mass too)
		dGeomSetPosition(dgeom_id, p[0], p[1], p[2]);
		dGeomSetRotation(dgeom_id, dr);
		dMassRotate(&dmass, dr);
		dMassTranslate(&dmass, p[0], p[1], p[2]);

		// enclose by outer transform geom object.
		dGeomID dgeom_t_id = dCreateGeomTransform(dspace_id);
		dGeomTransformSetCleanup(dgeom_t_id, 1);
		dGeomTransformSetGeom(dgeom_t_id, dgeom_id);

		dMassAdd(&total_mass, &dmass);

		dgeom_t_ids.push_back(dgeom_t_id);
	}

	// Adjust total Mass
	/*
	double t_mass=0.0;
	for ( unsigned int i=0; i<masses.size(); i++ )
	{
		t_mass += masses[i];
	}
	dMassAdjust(&total_mass, t_mass);
	*/

	cml::vector3d center_of_mass;
	center_of_mass[0] = total_mass.c[0];
	center_of_mass[1] = total_mass.c[1];
	center_of_mass[2] = total_mass.c[2];

	// move all encapsulated objects to make the center of mass to be (0,0,0).
	// This is required by ODE system to initialize Model geometry. 
	// To recover this, we will set the offset_position to the original center of mass postion. 
	for ( unsigned int i=0; i<dgeom_t_ids.size(); i++ )
	{
		dGeomID dgeom_id = dGeomTransformGetGeom( dgeom_t_ids[i] );
		const dReal *p = dGeomGetPosition(dgeom_id);
		dGeomSetPosition(dgeom_id, 
			p[0]-center_of_mass[0],
			p[1]-center_of_mass[1],
			p[2]-center_of_mass[2]);
	}
	dMassTranslate (&total_mass,-center_of_mass[0],-center_of_mass[1],-center_of_mass[2]);
	SetOffestPosition(center_of_mass);

	// Set Geoms and Mass to this Body
	for ( unsigned int i=0; i<dgeom_t_ids.size(); i++ )
	{
		dGeomSetBody(dgeom_t_ids[i], this->id());
	}
	this->setMass(total_mass);


	// We move this body globally.
	SetPositionExt(body_translation);
}


void
OdeBodyExt::DeleteGeometries()
{
	dGeomID g = dBodyGetFirstGeom(this->id());
	while ( g != 0 )
	{
		dGeomID tmp_g = g;
		g = dBodyGetNextGeom(g);
		dGeomDestroy(tmp_g);
	}
}




void
OdeBodyGroup::Disable()
{
	for ( unsigned int i=0; i<size(); i++ )
	{
		(*this)[i]->disable();
	}
}

void
OdeBodyGroup::Enable()
{
	for ( unsigned int i=0; i<size(); i++ )
	{
		(*this)[i]->enable();
	}
}


void
OdeBodyGroup::SetGravityMode(bool mode)
{
	for ( unsigned int i=0; i<size(); i++ )
	{
		(*this)[i]->setGravityMode(mode);
	}
}


void
OdeBodyGroup::SetCurrentAsInitialDynamics()
{
	for ( unsigned int i=0; i<size(); i++ )
	{
		(*this)[i]->SetCurrentAsInitialDynamics();
	}
}

void
OdeBodyGroup::ResetDynamics()
{
	for ( unsigned int i=0; i<size(); i++ )
	{
		(*this)[i]->ResetDynamics();
	}
}






OdeJointExt::OdeJointExt(dJoint *j)
{
	d_joint_ = j;
}

OdeJointExt::~OdeJointExt()
{
	delete d_joint_;
}






cml::vector3d ode2cml(const dVector3 v)
{
	return cml::vector3d(v[0], v[1], v[2]);
}

#endif
