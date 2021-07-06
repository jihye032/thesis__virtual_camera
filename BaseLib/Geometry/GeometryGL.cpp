
#include "BaseLib/Geometry/GeometryGL.h"
#include "BaseLib/Geometry/SolidObject.h"
#include "BaseLib/Geometry/PrimitiveShape.h"
#include "BaseLib/GLUU/gluu.h"

namespace mg
{

void DrawMesh(const Mesh& mesh)
{
	const double *vertices = mesh.vertices();
	const double *uvs = mesh.uvs();
	const double *normals = mesh.normals();
	const int    *face_sizes = mesh.face_sizes();

	if ( vertices == 0 ) return;
	if ( face_sizes == 0 ) return;

	if ( normals == 0 && uvs == 0 )
	{
		for ( int f=0; f<mesh.num_faces(); f++ )
		{
			const int *vertex_ids = mesh.face_vertex_ids(f);

			glBegin(GL_POLYGON);
			for ( int p=0; p<face_sizes[f]; p++ )
			{
				glVertex3dv( vertices + (vertex_ids[p] * 3) );
			}
			glEnd();
		}
	}
	else if ( normals != 0 && uvs == 0 )
	{
		for ( int f=0; f<mesh.num_faces(); f++ )
		{
			const int *vertex_ids = mesh.face_vertex_ids(f);
			const int *normal_ids = mesh.face_normal_ids(f);

			glBegin(GL_POLYGON);
			for ( int p=0; p<face_sizes[f]; p++ )
			{
				glNormal3dv( normals + (normal_ids[p] * 3) );
				glVertex3dv( vertices + (vertex_ids[p] * 3) );
			}
			glEnd();
		}
	}
	else if ( normals == 0 && uvs != 0 )
	{
		for ( int f=0; f<mesh.num_faces(); f++ )
		{
			const int *vertex_ids = mesh.face_vertex_ids(f);
			const int *uv_ids = mesh.face_uv_ids(f);

			glBegin(GL_POLYGON);
			for ( int p=0; p<face_sizes[f]; p++ )
			{
				glTexCoord2dv( uvs + (uv_ids[p] * 2) );
				glVertex3dv( vertices + (vertex_ids[p] * 3) );
			}
			glEnd();
		}
	}
	else if ( normals != 0 && uvs != 0 )
	{
		for ( int f=0; f<mesh.num_faces(); f++ )
		{
			const int *vertex_ids = mesh.face_vertex_ids(f);
			const int *uv_ids = mesh.face_uv_ids(f);
			const int *normal_ids = mesh.face_normal_ids(f);

			glBegin(GL_POLYGON);
			for ( int p=0; p<face_sizes[f]; p++ )
			{
				glTexCoord2dv( uvs + (uv_ids[p] * 2) );
				glNormal3dv( normals + (normal_ids[p] * 3) );
				glVertex3dv( vertices + (vertex_ids[p] * 3) );
			}
			glEnd();
		}
	}
}

void DrawSolidObject(const SolidObject& obj)
{
	glPushMatrix();
	mg::mgluTranslateV(obj.translation());
	mg::mgluRotateQ(obj.rotation());
	mg::mgluScaleV(obj.scaling());
	DrawMesh(*(obj.mesh()));
	glPopMatrix();
}


void DrawPrimitiveShape(const PrimitiveShape& p)
{
	switch ( p.type() )
	{
	case PrimitiveShape::CAPSULE :
		DrawPrimitiveCapsule((PrimitiveCapsule&)p);
		break;
	case PrimitiveShape::SPHERE :
		DrawPrimitiveSphere((PrimitiveSphere&)p);
		break;
	case PrimitiveShape::BOX :
		DrawPrimitiveBox((PrimitiveBox&)p);
		break;
	case PrimitiveShape::CYLINDER :
		DrawPrimitiveCylinder((PrimitiveCylinder&)p);
		break;
	default:
		break;
	};
}


void 
DrawPrimitiveCapsule(const PrimitiveCapsule& p)
{
	glPushMatrix();

	mgluTranslateV(p.global_translation());
	mgluRotateQ(p.global_rotation());
	
	mgluCapsule(p.cylinder_height(), p.radius(), p.direction());

	glPopMatrix();
}

void 
DrawPrimitiveCylinder(const PrimitiveCylinder& p)
{
	glPushMatrix();

	mgluTranslateV(p.global_translation());
	mgluRotateQ(p.global_rotation());
	
	mgluCylinder(p.height(), p.radius(), p.direction());

	glPopMatrix();
}

void 
DrawPrimitiveBox(const PrimitiveBox& p)
{
	glPushMatrix();

	mgluTranslateV(p.global_translation());
	mgluRotateQ(p.global_rotation());
	
	mgluBox(p.width(), p.height(), p.depth());

	glPopMatrix();
}

void 
DrawPrimitiveSphere(const PrimitiveSphere& p)
{
	glPushMatrix();

	mgluTranslateV(p.global_translation());
	mgluRotateQ(p.global_rotation());
	
	mgluSphere(p.radius());

	glPopMatrix();
}

void 
DrawPrimitiveComposition(const PrimitiveComposition& p)
{
	glPushMatrix();
	mgluTranslateV(p.global_translation());
	mgluRotateQ(p.global_rotation());

	for ( int i=0; i<p.CountPrimitives(); i++ )
	{
		DrawPrimitiveShape(*p.primitive(i));
	}
	glPopMatrix();
}


};