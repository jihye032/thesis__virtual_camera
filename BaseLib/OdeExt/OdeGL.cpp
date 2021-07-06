#ifdef ODE_EXT

#include "BaseLib/OdeExt/OdeGL.h"
#include "BaseLib/GLUU/gluu.h"
#include "BaseLib/Geometry/GeometryGL.h"


void DrawOdeGeom(dGeomID dgeom_id)
{
	glPushMatrix();

	while (1)
	{
		// Get Geom Transformation.
		const dReal* dt = dGeomGetPosition(dgeom_id);
		dQuaternion dr;
		dGeomGetQuaternion(dgeom_id, dr);
		::vector t(dt[0], dt[1], dt[2]); 
		::quater r(dr[0], dr[1], dr[2], dr[3]);

		mg::mgluTranslateV(t);
		mg::mgluRotateQ(r);

		// Offest
		dt = dGeomGetOffsetPosition(dgeom_id);
		dGeomGetOffsetQuaternion(dgeom_id, dr);

		t = ::vector(dt[0], dt[1], dt[2]); 
		r = quater(dr[0], dr[1], dr[2], dr[3]);

		mg::mgluTranslateV(t);
		mg::mgluRotateQ(r);


		if ( dGeomGetClass(dgeom_id) == dGeomTransformClass )
		{
			dgeom_id = dGeomTransformGetGeom(dgeom_id);
		}
		else
		{
			break;
		}
	}


	// Draw Geomatry
	if ( dCylinderClass == dGeomGetClass(dgeom_id) )
	{
		dReal r;
		dReal l;
		dGeomCylinderGetParams(dgeom_id, &r, &l);
		mg::mgluCylinder(l, r, 2);
	}
	else if ( dBoxClass == dGeomGetClass(dgeom_id) )
	{
		dVector3 box;
		dGeomBoxGetLengths(dgeom_id, box);
		mg::mgluBox(box[0], box[1], box[2]);
	}
	else if ( dCapsuleClass == dGeomGetClass(dgeom_id) )
	{
		dReal r;
		dReal l;
		dGeomCapsuleGetParams(dgeom_id, &r, &l);
		mg::mgluCapsule(l, r, 2);
	}
	else if ( dSphereClass == dGeomGetClass(dgeom_id) )
	{
		dReal r;
		r = dGeomSphereGetRadius(dgeom_id);
		mg::mgluSphere(r);
	}



	glPopMatrix();
}


void DrawOdeBody(dBodyID dbody_id, bool colored)
{
	glPushMatrix();

	dGeomID geom_id = dBodyGetFirstGeom(dbody_id);

	while ( geom_id != 0 )
	{
		if ( colored )
		{
			mg::mgluColorMaterial( mg::mgluGetRecentlyUsedColorIndex() +1 );
		}
		DrawOdeGeom(geom_id);
		geom_id = dBodyGetNextGeom(geom_id);
	}
	glPopMatrix();
}

















#endif
