


#include "BaseLib/CmlExt/CmlCamera.h"
#include "BaseLib/CmlExt/CmlExt.h"

static cml::vector3d rotate(cml::quaterniond q, cml::vector3d v)
{
	cml::matrix33d m;
	cml::matrix_rotation_quaternion(m, q);

	return cml::transform_vector(m, v);
}

static const cml::vector3d x_axis(1, 0, 0);
static const cml::vector3d y_axis(0, 1, 0);
static const cml::vector3d z_axis(0, 0, 1);

// camera가 초기 상태일때 z축의 양의 방향으로 보고 있게 하기 위해서 사용된다.
static const cml::quaterniond y180(cos(M_PI/2), 0, sin(M_PI/2), 0);

namespace cml
{

Camera::Camera()
{
	cameraZoom.set(1, 1, 1);
	cameraP.set(0, 0, 0);
	cml::quaterniond q(1, 0, 0, 0);
	setRotation(q);
	
	
	fov = 45;
	aspect = 1;
	_near = 0.1f;
	_far = 10000.0f;

	trackBallR = 0.8f;

	flag_ortho = false;
	setOrthVolume(cml::vector3d(200, 200, 200));

	pivot = cml::vector3d(0, 0, 0);
}

Camera& 
Camera::operator=(const Camera& a)
{
	this->cameraP = a.cameraP;

	this->cameraQ = a.cameraQ;			

	this->cameraZoom = a.cameraZoom;

	this->fov = a.fov;
	this->aspect = a.aspect;
	this->flag_ortho = a.flag_ortho;

	this->_near = a._near;
	this->_far = a._far;

	this->trackBallR = a.trackBallR;
	this->orth_viewport_volume_min = a.orth_viewport_volume_min; 
	this->orth_viewport_volume_max = a.orth_viewport_volume_max; 

	this->pivot = a.pivot;

	return *this;
}


void Camera::setGLMatrix(double m[16])
{
}

void Camera::getGLMatrix(double m[16])
{

}




void Camera::setTranslation(const cml::vector3d &v)
{
	cameraP = v;
}




void Camera::setRotation(const cml::quaterniond &r)
{
	cameraQ = r;
//	cameraQ_y180 = quater(cos(M_PI/2), 0, sin(M_PI/2), 0) * r;
}

void Camera::setRotation(const cml::vector3d &euler_xyz)
{
	cml::quaternion_rotation_euler(cameraQ, euler_xyz[0], euler_xyz[1], euler_xyz[2], cml::euler_order_xyz);
//	cameraQ_y180 = quater(cos(M_PI/2), 0, sin(M_PI/2), 0) * r;
}

void Camera::setZoom(double z)
{
	cameraZoom.set(z, z, z);
}


void Camera::setZoom(cml::vector3d z)
{
	cameraZoom = z;
}

void Camera::setZoom(double sx, double sy, double sz)
{
	setZoom(cml::vector3d(sx, sy, sz));
}




void Camera::setFov(double f)
{
	fov = f;
}

double Camera::getFov() const
{
	if ( flag_ortho ) return 0;
	return fov;
}




void Camera::setAspectRatio(double r)
{
	aspect = r;
}

double Camera::getAspectRatio() const
{
	return aspect;
}

void Camera::setNearFar(double n, double f)
{
	_near = n;
	_far = f;
}

double Camera::getNear() const
{
	if ( flag_ortho ) return orth_viewport_volume_min[2];
	return _near;
}

double Camera::getFar() const
{
	if ( flag_ortho ) return orth_viewport_volume_max[2];
	return _far;
}



cml::vector3d Camera::getTranslation() const
{
	return cameraP;
}

cml::quaterniond Camera::getRotation() const
{
	return cameraQ;
}

cml::vector3d Camera::getViewDirection() const
{
	return cml::Rotate(cameraQ, cml::z_axis_3D());
}

cml::vector3d Camera::getZoom() const
{
	return cameraZoom;
}

cml::vector3d Camera::getTranslationForGL() const
{
	return -1*cameraP;
}

cml::quaterniond Camera::getRotationForGL() const
{
	return y180 * cml::inverse(cameraQ);
}


cml::vector3d Camera::projectToTrackBall(double x, double y)
{
	double d, t, z;
	double r = trackBallR;

	d = (float) sqrt(x*x + y*y);
	if( d < r * 0.70710678118654752440f ) // Inside sphere 
	{		
		z = (float)sqrt( r*r-d*d );
	} 
	else // On hyperbola
	{
		t = r / 1.41421356237309504880f;
		z = t*t / d;
	}
	return cml::vector3d(x, y, z);
}



void Camera::inputMouse(int button, double x1, double y1, double x2, double y2, double speedScale)
{
	double t_scale = 700;
	//x1 = x2 = 0.5;

	if ( speedScale > 0 )
	{
		t_scale = t_scale * speedScale;
	}

	if ( button == IN_TRANS )
	{
		cml::vector3d t(0, 0, 0);
		t[0] = t_scale * (x2 - x1);
		t[1] = -1 * t_scale * (y2 - y1);

		cameraP += rotate(cameraQ, t);
	}
	else if ( button == IN_ZOOM )
	{
		double s;
		if ( speedScale > 0 ) 
			s = ::exp((y2 - y1)*speedScale);
		else
			s = ::exp(y2 - y1);

		cameraP[0] -= pivot[0]*cameraZoom[0] ;
		cameraP[1] -= pivot[1]*cameraZoom[1] ;
		cameraP[2] -= pivot[2]*cameraZoom[2] ;
		cameraZoom *= s;
		cameraP[0] += pivot[0]*cameraZoom[0] ;
		cameraP[1] += pivot[1]*cameraZoom[1] ;
		cameraP[2] += pivot[2]*cameraZoom[2] ;
	}
	else if ( button == IN_TRANS_Z )
	{
		cml::vector3d t(0, 0, 0);
		t[2] = -1 * t_scale * (y2 - y1);

		cameraP += rotate( cameraQ, t);
	}
	else if ( button == IN_ROTATION )
	{
		cml::vector3d p1 = projectToTrackBall( x1*2-1, y1*2-1 );
		cml::vector3d p2 = projectToTrackBall( x2*2-1, y2*2-1 );

		/*
		double tmpF = p1[0];
		p1[0] = p2[0];
		p2[0] = tmpF;

		p1[0] *= -1;
		p2[0] *= -1;
		*/
		
		// Figure out how much to rotate around that axis
		cml::vector3d d = p2 - p1;
		double dLen = d.length() / (2.0f * trackBallR);

		// Avoid problems with out-of-control values
		if ( dLen > 1.0f ) 
		{
			dLen = 1.0f;
		}
		else if ( dLen < -1.0f )
		{
			dLen = -1.0f;
		}

		double phi = 2.0f * (float)asin(dLen);

		cml::vector3d cross = cml::cross(p1, p2);
		cross[1] *= -1;
		cml::quaterniond q;
		cml::quaternion_rotation_axis_angle(q, normalize(cross), phi);


		cml::quaterniond tmpQ = cameraQ*q*cml::inverse(cameraQ);
		tmpQ = tmpQ.normalize();
		cml::vector3d s_pivot = pivot;
		s_pivot[0] *= cameraZoom[0];
		s_pivot[1] *= cameraZoom[1];
		s_pivot[2] *= cameraZoom[2];
		cameraP = rotate(tmpQ, cameraP-s_pivot)+s_pivot;
		cameraQ = cameraQ*q;


		/*cml::quaterniond tmpQ = cameraQ*q*cameraQ.inverse();
		tmpQ = tmpQ.normalize();
		cameraP = rotate(tmpQ, cameraP);
		cameraQ = cameraQ*q;*/
	
	}
	else if ( button == IN_ROTATION_Y_UP )
	{
		double dy = y2-y1;
		double dx = x2-x1;

		cml::quaterniond rot_x;// = exp(-1*dy*x_axis);
		cml::quaternion_rotation_axis_angle(rot_x, x_axis, -1*dy*2);


		cml::quaterniond rot_y;// = exp(-1*dx*y_axis);
		cml::quaternion_rotation_axis_angle(rot_y, y_axis, -1*dx*2);


		cml::quaterniond q = rot_y * rot_x;

		cml::quaterniond tmpQ = rot_y*cameraQ*rot_x*cml::inverse(cameraQ);
		tmpQ = tmpQ.normalize();
		cml::vector3d s_pivot = pivot;
		s_pivot[0] *= cameraZoom[0];
		s_pivot[1] *= cameraZoom[1];
		s_pivot[2] *= cameraZoom[2];
		cameraP = rotate(tmpQ, cameraP-s_pivot)+s_pivot;
		cameraQ = rot_y*cameraQ*rot_x;


		/*cml::quaterniond tmpQ = cameraQ*q*cameraQ.inverse();
		tmpQ = tmpQ.normalize();
		cameraP = rotate(tmpQ, cameraP);
		cameraQ = cameraQ*q;*/
	
	}
	else if ( button == IN_FOV )
	{
		setFov(getFov() + (y2 - y1)*10);
	}

	
}

void Camera::inputMouse(int button, double x1, double y1, double x2, double y2, cml::vector3d center, double speedScale)
{
	double t_scale = 700;
	//x1 = x2 = 0.5;

	if ( speedScale > 0 )
	{
		t_scale = t_scale * speedScale;
	}

	if ( button == IN_TRANS )
	{
		cml::vector3d t(0, 0, 0);
		t[0] = t_scale * (x2 - x1);
		t[1] = -1 * t_scale * (y2 - y1);

		cameraP += rotate(cameraQ, t);
	}
	else if ( button == IN_ZOOM )
	{
		double s;
		if ( speedScale > 0 ) 
			s = ::exp((y2 - y1)*speedScale);
		else
			s = ::exp(y2 - y1);

		cameraP[0] -= center[0]*cameraZoom[0] ;
		cameraP[1] -= center[1]*cameraZoom[1] ;
		cameraP[2] -= center[2]*cameraZoom[2] ;
		cameraZoom *= s;
		cameraP[0] += center[0]*cameraZoom[0] ;
		cameraP[1] += center[1]*cameraZoom[1] ;
		cameraP[2] += center[2]*cameraZoom[2] ;
	}
	else if ( button == IN_TRANS_Z )
	{
		cml::vector3d t(0, 0, 0);
		t[2] = -1 * t_scale * (y2 - y1);

		cameraP += rotate(cameraQ, t);
	}
	else if ( button == IN_ROTATION )
	{
		cml::vector3d p1 = projectToTrackBall( x1*2-1, y1*2-1 );
		cml::vector3d p2 = projectToTrackBall( x2*2-1, y2*2-1 );

		/*
		double tmpF = p1[0];
		p1[0] = p2[0];
		p2[0] = tmpF;

		p1[0] *= -1;
		p2[0] *= -1;
		*/
		
		// Figure out how much to rotate around that axis
		cml::vector3d d = p2 - p1;
		double dLen = d.length() / (2.0f * trackBallR);

		// Avoid problems with out-of-control values
		if ( dLen > 1.0f ) 
		{
			dLen = 1.0f;
		}
		else if ( dLen < -1.0f )
		{
			dLen = -1.0f;
		}

		double phi = 2.0f * (float)asin(dLen);

		cml::vector3d cross = cml::cross(p1, p2);
		cross[1] *= -1;
		cml::quaterniond q;
		cml::quaternion_rotation_axis_angle(q, normalize(cross), phi);


		cml::quaterniond tmpQ = cameraQ*q*cml::inverse(cameraQ);
		tmpQ = tmpQ.normalize();
		cml::vector3d s_center = center;
		s_center[0] *= cameraZoom[0];
		s_center[1] *= cameraZoom[1];
		s_center[2] *= cameraZoom[2];
		cameraP = rotate(tmpQ, cameraP-s_center)+s_center;
		cameraQ = cameraQ*q;
		//cameraP = cml::length(cameraP) * rotate(cameraQ.inverse(), z_axis);
	
	}
	else if ( button == IN_FOV )
	{
		setFov(getFov() + (y2 - y1)*10);
	}

	
}



void Camera::inputMouse(int button, int dx, int dy, double speedScale)
{
	if ( button == IN_TRANS )
	{
		cml::vector3d t(0, 0, 0);
		t[0] = speedScale * (dx);
		t[1] = -1 * speedScale * (dy);

		cameraP += rotate(cameraQ, t);
	}
	else if ( button == IN_TRANS_Z )
	{
		double moveScale = 50;

		if ( speedScale > 0 )
		{
			moveScale *= speedScale;
		}

		cml::vector3d t(0, 0, 0);
		t[2] = -1 * moveScale * dy;
		cameraP += rotate(cameraQ, t);
	}
	else if ( button == IN_ZOOM )
	{
		double s;
		if ( speedScale > 0 ) 
			s = ::exp(((float)dy/10)*speedScale);
		else
			s = ::exp((float)dy/10);

		cameraP[0] -= pivot[0]*cameraZoom[0] ;
		cameraP[1] -= pivot[1]*cameraZoom[1] ;
		cameraP[2] -= pivot[2]*cameraZoom[2] ;
		cameraZoom *= s;
		cameraP[0] += pivot[0]*cameraZoom[0] ;
		cameraP[1] += pivot[1]*cameraZoom[1] ;
		cameraP[2] += pivot[2]*cameraZoom[2] ;
	}
	else if ( button == IN_NONUNIFORM_ZOOM )
	{
		if ( speedScale > 0 ) 
		{
			cameraZoom[0] *= ::exp(((float)dx/10)*speedScale);
			cameraZoom[1] *= ::exp(((float)dy/10)*speedScale);
			cameraZoom[2] *= ::exp(((float)dx/10)*speedScale);
		}
		else
		{
			cameraZoom[0] *= ::exp((float)dx/10);
			cameraZoom[1] *= ::exp((float)dy/10);
			cameraZoom[2] *= ::exp((float)dx/10);
		}
	}
	else if ( button == IN_FOV )
	{
		setFov(getFov() + dy);
	}
}	


void Camera::inputMouse(int button, int dx, int dy, int dz, double speedScale)
{
	if ( button == IN_NONUNIFORM_ZOOM )
	{
		if ( speedScale > 0 ) 
		{
			cameraZoom[0] *= ::exp(((float)dx/10)*speedScale);
			cameraZoom[1] *= ::exp(((float)dy/10)*speedScale);
			cameraZoom[2] *= ::exp(((float)dz/10)*speedScale);
		}
		else
		{
			cameraZoom[0] *= ::exp((float)dx/10);
			cameraZoom[1] *= ::exp((float)dy/10);
			cameraZoom[2] *= ::exp((float)dz/10);
		}
	}
	else 
	{
		inputMouse(button, dx, dy, speedScale);
	}
}	






void Camera::lookAt(cml::vector3d eye, cml::vector3d center, cml::vector3d up)
{
	cameraP = eye;

	cml::vector3d F = center - eye;
	cml::vector3d f = F / cml::length(F);
	cml::vector3d upu = up / cml::length(up);
	cml::vector3d s = cml::cross(f, upu);
	cml::vector3d u  = cml::cross(s, f);

	/*
	matrix rm( s[0], s[1], s[2],
			u[0], u[1], u[2],
			-1*f[0], -1*f[1], -1*f[2]);
			*/
	
	cml::matrix33d rm( s[0], u[0], -1*f[0],
			s[1], u[1], -1*f[1],
			s[2], u[2], -1*f[2]);
	rm.transpose();

	cml::quaterniond rq;
	cml::quaternion_rotation_matrix(rq, rm);

	cameraQ = cml::inverse(rq) * cml::inverse(y180);
}

cml::matrix44d Camera::GetGLProjectionMatrix() const
{
	cml::matrix44d m;
	if ( flag_ortho )
	{
		//glOrtho((orth_viewport_volume_min[0]), (orth_viewport_volume_max[0]), 
		//	(orth_viewport_volume_min[1]), (orth_viewport_volume_max[1]), 
		//	(orth_viewport_volume_min[2]), (orth_viewport_volume_max[2]));
		cml::matrix_orthographic_RH(m, (double)(orth_viewport_volume_min[0]), (double)(orth_viewport_volume_max[0]), 
			(double)(orth_viewport_volume_min[1]), (double)(orth_viewport_volume_max[1]), 
			(double)(orth_viewport_volume_min[2]), (double)(orth_viewport_volume_max[2]), cml::z_clip_neg_one);
	}
	else
	{
		//gluPerspective(getFov(), getAspectRatio(), _near, _far);
		cml::matrix_perspective_yfov_RH(m, (double)getFov(), (double)getAspectRatio(), (double)_near, (double)_far, cml::z_clip_neg_one);
		//cml::matrix_perspective_yfov_RH(m, (float)cml::rad(30.0f), (float)aspect, 1.0f, 1000.0f, cml::z_clip_neg_one);
	}

	return m;
}

cml::matrix44d Camera::GetGLViewMatrix() const
{
	cml::matrix44d m;


	// camera가 초기 상태일때 z축의 양의 방향으로 보고 있게 하기 위해 항성 y180을 먼저 적용한다.
	//cml::vector3d qv =  
	double angle;
	cml::vector3d axis;
	cml::quaternion_to_axis_angle( y180 * inverse(cameraQ), axis, angle );
	// glRotatef(180.0*angle/cml::constantsd::pi(), axis[0], axis[1], axis[2]);
	m = cml::MatrixRotationAxisAngle((cml::vector3f)axis, (float)angle);

	cml::vector3d t = -1 * cameraP;
	//glTranslatef(t[0], t[1], t[2]);
	m = m*cml::MatrixTranslation((cml::vector3f)t);

	//glScalef(cameraZoom[0], cameraZoom[1], cameraZoom[2]);
	m =  m*cml::MatrixScaling((cml::vector3f)cameraZoom);

	return m;
}


cml::matrix44d Camera::GetGLProjectionViewMatrix() const
{
	return GetGLProjectionMatrix()*GetGLViewMatrix();
}


cml::matrix44d Camera::GetGLViewMatrixInverse() const
{
	return GetGLViewMatrix().inverse();

}

};


