#include "BaseLib/Motion/ml.h"
#include "BaseLib/Algorithm/Optimize.h"



#define PENV_IK_ANALYTICAL				0
#define PENV_IK_NUMERICAL				1
#define PENV_IK_HYBRID					2

#define PENV_IK_CG_METHOD				0
#define PENV_IK_LS_SVD_METHOD			1
#define PENV_IK_LS_DAMP_METHOD			2

bool			PenvDampingEnabled		= true;
bool			PenvJointLimitEnabled	= false;

static int PenvIKsolverMethod = PENV_IK_LS_DAMP_METHOD;
//static int PenvIKsolverMethod = PENV_IK_LS_SVD_METHOD;
//static int PenvIKsolverMethod = PENV_IK_CG_METHOD;
static int PenvIKsolverType = PENV_IK_NUMERICAL;

static double			PenvIKerrorBound = 0.01;
static double			PenvIKtolerance = 0.1;

//----------------------------------------------------------//
//															//
//			  Inverse kinematics for the full body 			//
//															//
//----------------------------------------------------------//
using namespace ml;

#define DAMPING			1.0

static double  energyFunc( cml::vectord const& );
static double  gradientFuncLS( cml::vectord const&, cml::vectord& );
static double  gradientFunc( cml::vectord const&, cml::vectord& );
static double  computeJacobian();

static int				num_dof,
						num_equations,
						num_unknowns;
//static PmMaskType		link_mask;
static const Body*			oBody;
static Posture		oPosture;
static Posture		tPosture;
static Constraint		constraint;

static cml::matrixd			J;
static cml::vectord			b;



void
Posture::AddDisplacement( cml::vectord const& disp )
{
	int i, j = 0;
	cml::vector3d p, v;

	// PmMaskType mask = getMask();
	for( i=0; i<(int)m_body->num_joint(); i++ )
		//if ( mask & MaskBit(i) )
	{
		if ( m_body->GetDOF(i)==6 )
		{
			p = cml::vector3d( disp[j  ], disp[j+1], disp[j+2] );
			v = cml::vector3d( disp[j+3], disp[j+4], disp[j+5] );
			//m_rotates[i] = m_rotates[i] * cml::mat3( cml::MatrixRotationQuaternion(cml::EXP(v)) );
			m_rotates[i] = cml::mat3( cml::MatrixRotationQuaternion(cml::EXP(v)) ) * m_rotates[i];
			m_trans += p;
			j += 6;
		}
		else if ( m_body->GetDOF(i)==3 )
		{
			v = cml::vector3d( disp[j  ], disp[j+1], disp[j+2] );
			//m_rotates[i] = m_rotates[i] * cml::mat3( cml::MatrixRotationQuaternion(cml::EXP(v)) );
			m_rotates[i] = cml::mat3( cml::MatrixRotationQuaternion(cml::EXP(v)) ) * m_rotates[i];
			j += 3;
		}
		else if ( m_body->GetDOF(i)==1 )
		{
			v = cml::vector3d( disp[j  ], 0, 0 );
			//m_rotates[i] = m_rotates[i] * cml::mat3( cml::MatrixRotationQuaternion(cml::EXP(v)) );
			m_rotates[i] = cml::mat3( cml::MatrixRotationQuaternion(cml::EXP(v)) ) * m_rotates[i];
			j ++;
		}
	}

	/*if ( mask & MaskBit(PmHuman::CHEST) )
	{
		quater q = getRotation( PmHuman::CHEST );

		for( i=0; i<getNumSpineLinks(); i++ )
			setRotation( PmHuman::CHEST-i-1, q );
	}	*/
}


void Posture::IkFullBody(Constraint const& c)
{
	
	constraint = c;

	//link_mask = this->getMask() & m;
	oBody = body();

	oPosture = (*this);
	tPosture = (*this);


	//oPosture.setMask( link_mask );
	//tPosture.setMask( link_mask );

	//num_dof = oPosture.getDOF();
	num_dof = oBody->GetDOF();

	static cml::vectord d; 
	d.resize(num_dof);

	int i;
	for( i=0; i<num_dof; i++ ) d[i] = 0.0;

	double f;
	int iter = 0;
	double tmp;
	if ( tmp=energyFunc(d) > PenvIKerrorBound )
	{
		//printf("PENV_IK_CG_METHOD %d\n", PenvIKsolverMethod);
		if ( PenvIKsolverMethod == PENV_IK_CG_METHOD )
		{
			frprmn( d, num_dof, PenvIKtolerance, iter, f, energyFunc, gradientFunc );
		}
		else
		{
			gradient_descent( d, num_dof, PenvIKtolerance, iter, f, energyFunc, gradientFuncLS );
		}
	}



	oPosture.AddDisplacement( d );
	//this->copyUnder( oPosture );
	*this = oPosture;

	cml::transf t;
	for( i=0; i<(int)c.num_entity(); i++ )
		if ( c.entities()[i].mask & C_ORIENTATION )
		{
			SetGlobalRotation(c.entities()[i].joint, cml::mat3(c.entities()[i].value));
			// t = c.entities[i].value * getBaseTransf(c.entity[i].link).inverse();
			// rotate[c.entity[i].link] = t.getRotation();
		}

	// Limb IK
	{
		std::vector<int> end_joints;
		end_joints.push_back(oBody->joint_index(L_WRIST));
		end_joints.push_back(oBody->joint_index(L_PALM));
		end_joints.push_back(oBody->joint_index(L_ANKLE));
		end_joints.push_back(oBody->joint_index(L_FOOT));
		end_joints.push_back(oBody->joint_index(R_WRIST));
		end_joints.push_back(oBody->joint_index(R_PALM));
		end_joints.push_back(oBody->joint_index(R_ANKLE));
		end_joints.push_back(oBody->joint_index(R_FOOT));
		
		for ( auto &j : end_joints )
		{
			const ml::ConstraintEntity *e = c.GetConstraintEntity( j );
			if ( e && e->mask & C_POSITION ) 
			{
				IkLimb(j, cml::trans(e->value));
			}

			if ( e && e->mask & C_ORIENTATION ) 
			{
				SetGlobalRotation(j, cml::mat3(e->value));
			}
			
		}
	}
}




//------------------------------------------------------//
//														//
//	 			Computing Energy Function				//
//														//
//------------------------------------------------------//

static double
energyFunc( cml::vectord const&d )
{
	tPosture = oPosture;
	tPosture.AddDisplacement( d );

	double	dist = 0.0;
	cml::vector3d	dv, dq, dc;
	cml::quaterniond	q;
	int		i;//, j, jj;

	for( i=0; i<(int)constraint.num_entity(); i++ )
	{
		const ConstraintEntity &c = constraint.entities()[i];

		if ( c.mask & C_POSITION )
		{
			dv = tPosture.GetGlobalTranslation( c.joint ) -
				cml::trans(c.value);
			dist += cml::dot(dv, dv);
		}

		if ( c.mask & C_ORIENTATION )
		{
			//dq = difference( tPosture.GetGlobalRotation( c.joint ),
			//	c.value.getRotation() );
			//dist += cml::dot(dq, dq);
			double tmp = cml::angle_distance( 
				tPosture.GetGlobalRoation( c.joint ),
				mat3(c.value) );
			dist += tmp*tmp;
		}

		/*
		if ( c.mask & PM_C_COG )
		{
			dc = tPosture.getCOG() - c.value.getTranslation();
			dist += dc%dc;
		}

		if ( c.mask & PM_C_BALANCE )
		{
			dc = tPosture.getCOG() - c.value.getTranslation();
			dc.set_y( 0.0 );
			dist += dc%dc;
		}
		*/
	}

	/*if ( PenvDampingEnabled )
		for( j=0, jj=0; j<PM_HUMAN_NUM_LINKS; j++ )
			if ( link_mask & MaskBit(j) )
			{
				double c1 = PenvDampingCoeff[j+1];

				if ( tPosture.getDOF( j ) == 6 )
				{
					dist += c1*d[jj]*d[jj]; jj++;
					dist += c1*d[jj]*d[jj]; jj++;
					dist += c1*d[jj]*d[jj]; jj++;

					dist += c1*d[jj]*d[jj]; jj++;
					dist += c1*d[jj]*d[jj]; jj++;
					dist += c1*d[jj]*d[jj]; jj++;
				}
				else if ( tPosture.getDOF( j ) == 3 )
				{
					dist += c1*d[jj]*d[jj]; jj++;
					dist += c1*d[jj]*d[jj]; jj++;
					dist += c1*d[jj]*d[jj]; jj++;
				}
				else if ( tPosture.getDOF( j ) == 1 )
				{
					dist += c1*d[jj]*d[jj]; jj++;
				}
			}

	if ( PenvJointLimitEnabled )
		for( j=1; j<PM_HUMAN_NUM_LINKS; j++ )
			if ( link_mask & MaskBit(j) )
			{
				QmComplex *angle_bound = oBody->getAngleBound(j);

				if ( angle_bound )
				{
					q = tPosture.getRotation( j );
					double a = angle_bound->distance(q);
					dist += PenvJointLimitCoeff[j+1] * a * a;
				}
			}*/

	return dist;
}



//------------------------------------------------------//
//														//
// 				Computing Jacobian matrix				//
//														//
//------------------------------------------------------//

static double
computeJacobian()
{
	double	dist = 0.0;
	cml::vector3d	dv, dq, dc;
	cml::transf	t;
	cml::vector3d	endTrans;
	cml::quaterniond	endRot;

	int		i, j;
	cml::vector3d	w1, w2, w3;

	num_equations = std::max<int>( (int)constraint.GetDOC(), (int)num_dof );
	num_unknowns  = num_dof;


	J.resize( num_equations, num_unknowns );
	b.resize( num_equations );

	static std::vector<double> mass(oBody->num_joint());
	static std::vector<cml::vector3d> cog(oBody->num_joint());

	for( i=0; i<num_equations; i++ )
	{
		b[i] = 0.0;
		for( j=0; j<num_unknowns; j++ )
			J(i,j) = 0.0;
	}

	int	ii, iii, jj;

	for( i=0, ii=0; i<(int)constraint.num_entity(); i++ )
	{
		const ConstraintEntity &c = constraint.entities()[i];

		endTrans = tPosture.GetGlobalTranslation( c.joint );
		endRot   = tPosture.GetGlobalRotationQ( c.joint );

		int temp = ii;

		if ( c.mask & C_POSITION )
		{
			dv = cml::trans(c.value) - endTrans;
			dist += cml::dot(dv, dv);

			b[ii++] = dv[0];
			b[ii++] = dv[1];
			b[ii++] = dv[2];
		}

		if ( c.mask & C_ORIENTATION )
		{
			dq = cml::LOG ( cml::quaternion_rotation_difference( cml::QuaternionMatrix(c.value), endRot ) );
			dist += cml::dot(dq, dq);

			b[ii++] = dq[0];
			b[ii++] = dq[1];
			b[ii++] = dq[2];
		}

		/*if ( c.mask & PM_C_COG )
		{
			dc = tPosture.getCOG() - c.value.getTranslation();
			dist += dc%dc;

			b[ii++] = dc.x();
			b[ii++] = dc.y();
			b[ii++] = dc.z();

			tPosture.getCOGlist( mass, cog );
		}

		if ( c.mask & PM_C_BALANCE )
		{
			dc = tPosture.getCOG() - c.value.getTranslation();
			dc.set_y( 0.0 );
			dist += dc%dc;

			b[ii++] = dc.x();
			b[ii++] = dc.z();

			tPosture.getCOGlist( mass, cog );
		}*/

		for( j=0, jj=0; j<(int)oBody->num_joint(); j++ )
			//if ( link_mask & MaskBit(j) )
			{
			if ( oBody->IsAncestor( j, c.joint ) )
				{
					iii = temp;

					if ( oBody->GetDOF( j ) == 6 )
					{
						if ( c.mask & C_POSITION )
						{
							J(iii  , jj  ) = 100;
							J(iii+1, jj+1) = 100;
							J(iii+2, jj+2) = 100;

							t  = tPosture.GetGlobalTransf(j);
							dv = endTrans - cml::trans(t);

							w1 = cml::cross(cml::mat3(t)*cml::vector3d(1,0,0), dv);
							w2 = cml::cross(cml::mat3(t)*cml::vector3d(0,1,0), dv);
							w3 = cml::cross(cml::mat3(t)*cml::vector3d(0,0,1), dv);

							J(iii  , jj+3) = w1[0];
							J(iii  , jj+4) = w2[0];
							J(iii  , jj+5) = w3[0];
									 
							J(iii+1, jj+3) = w1[1];
							J(iii+1, jj+4) = w2[1];
							J(iii+1, jj+5) = w3[1];
									 
							J(iii+2, jj+3) = w1[2];
							J(iii+2, jj+4) = w2[2];
							J(iii+2, jj+5) = w3[2];

							iii += 3;
						}

						if ( c.mask & C_ORIENTATION )
						{
							J(iii  , jj+3) = 1;
							J(iii+1, jj+4) = 1;
							J(iii+2, jj+5) = 1;
						}

						/*if ( c.mask & PM_C_COG )
						{
							J[iii  ][jj  ] = 1;
							J[iii+1][jj+1] = 1;
							J[iii+2][jj+2] = 1;

							t  = tPosture.getGlobalTransf(j);
							dv = cog[j] - t.getTranslation();

							double m = mass[j] / mass[0];

							w1 = m * vector(1,0,0) * t * dv;
							w2 = m * vector(0,1,0) * t * dv;
							w3 = m * vector(0,0,1) * t * dv;

							J[iii  ][jj+3] = w1[0];
							J[iii  ][jj+4] = w2[0];
							J[iii  ][jj+5] = w3[0];

							J[iii+1][jj+3] = w1[1];
							J[iii+1][jj+4] = w2[1];
							J[iii+1][jj+5] = w3[1];

							J[iii+2][jj+3] = w1[2];
							J[iii+2][jj+4] = w2[2];
							J[iii+2][jj+5] = w3[2];
						}

						if ( c.mask & PM_C_BALANCE )
						{
							J[iii  ][jj  ] = 1;
							J[iii+1][jj+2] = 1;

							t  = tPosture.getGlobalTransf(j);
							dv = cog[j] - t.getTranslation();

							double m = mass[j] / mass[0];

							w1 = m * vector(1,0,0) * t * dv;
							w2 = m * vector(0,1,0) * t * dv;
							w3 = m * vector(0,0,1) * t * dv;

							J[iii  ][jj+3] = w1[0];
							J[iii  ][jj+4] = w2[0];
							J[iii  ][jj+5] = w3[0];

							J[iii+1][jj+3] = w1[2];
							J[iii+1][jj+4] = w2[2];
							J[iii+1][jj+5] = w3[2];
						}*/
					}
					else if ( oBody->GetDOF( j ) == 3 )
					{
						if ( c.mask & C_POSITION )
						{
							t  = tPosture.GetGlobalTransf(j);
							dv = endTrans - cml::trans(t);

							w1 = cml::cross(cml::mat3(t)*cml::vector3d(1,0,0), dv);
							w2 = cml::cross(cml::mat3(t)*cml::vector3d(0,1,0), dv);
							w3 = cml::cross(cml::mat3(t)*cml::vector3d(0,0,1), dv);

							J(iii  , jj  ) = w1[0];
							J(iii  , jj+1) = w2[0];
							J(iii  , jj+2) = w3[0];

							J(iii+1, jj  ) = w1[1];
							J(iii+1, jj+1) = w2[1];
							J(iii+1, jj+2) = w3[1];

							J(iii+2, jj  ) = w1[2];
							J(iii+2, jj+1) = w2[2];
							J(iii+2, jj+2) = w3[2];



							iii += 3;
						}

						if ( c.mask & C_ORIENTATION )
						{
							J(iii  , jj  ) = 1;
							J(iii+1, jj+1) = 1;
							J(iii+2, jj+2) = 1;
						}

						/*if ( c.mask & PM_C_COG )
						{
							t  = tPosture.getGlobalTransf(j);
							dv = cog[j] - t.getTranslation();

							double m = mass[j] / mass[0];

							w1 = m * vector(1,0,0) * t * dv;
							w2 = m * vector(0,1,0) * t * dv;
							w3 = m * vector(0,0,1) * t * dv;

							J[iii  ][jj+1] = w1[0];
							J[iii  ][jj+2] = w2[0];
							J[iii  ][jj+3] = w3[0];

							J[iii+1][jj+1] = w1[1];
							J[iii+1][jj+2] = w2[1];
							J[iii+1][jj+3] = w3[1];

							J[iii+2][jj+1] = w1[2];
							J[iii+2][jj+2] = w2[2];
							J[iii+2][jj+3] = w3[2];
						}

						if ( c.mask & PM_C_BALANCE )
						{
							t  = tPosture.getGlobalTransf(j);
							dv = cog[j] - t.getTranslation();

							double m = mass[j] / mass[0];

							w1 = m * vector(1,0,0) * t * dv;
							w2 = m * vector(0,1,0) * t * dv;
							w3 = m * vector(0,0,1) * t * dv;

							J[iii  ][jj+1] = w1[0];
							J[iii  ][jj+2] = w2[0];
							J[iii  ][jj+3] = w3[0];

							J[iii+1][jj+1] = w1[2];
							J[iii+1][jj+2] = w2[2];
							J[iii+1][jj+3] = w3[2];
						}*/
					}
					else if ( oBody->GetDOF( j ) == 1 )
					{
						if ( c.mask & C_POSITION )
						{
							t  = tPosture.GetGlobalTransf(j);
							dv = cml::cross(cml::mat3(t)*cml::vector3d(1,0,0), endTrans - cml::trans(t));

							J(iii  , jj) = dv[0];
							J(iii+1, jj) = dv[1];
							J(iii+2, jj) = dv[2];

							iii += 3;
						}

						if ( c.mask & C_ORIENTATION )
						{
							J(iii  , jj) = 1;
							J(iii+1, jj) = 0;
							J(iii+2, jj) = 0;
						}

						/*if ( c.mask & PM_C_COG )
						{
							t  = tPosture.getGlobalTransf(j);
							dv = cog[j] - t.getTranslation();

							double m = mass[j] / mass[0];

							w1 = m * vector(1,0,0) * t * dv;

							J[iii  ][jj  ] = w1[0];
							J[iii+1][jj  ] = w1[1];
							J[iii+2][jj  ] = w1[2];
						}

						if ( c.mask & PM_C_BALANCE )
						{
							t  = tPosture.getGlobalTransf(j);
							dv = cog[j] - t.getTranslation();

							double m = mass[j] / mass[0];

							w1 = m * vector(1,0,0) * t * dv;

							J[iii  ][jj  ] = w1[0];
							J[iii+1][jj  ] = w1[2];
						}*/
					}
				}

				jj += oBody->GetDOF( j );
			}
	}

	return dist;
}


//------------------------------------------------------//
//														//
// 		Computing Gradient of Energy Function			//
//														//
//------------------------------------------------------//

static void
scalingJacobian( cml::matrixd &J )
{
	int i, j, jj;
	double  c;

	for( j=0, jj=0; j<(int)oBody->num_joint(); j++ )
		//if ( link_mask & MaskBit(j) )
		{
			if ( oBody->GetDOF(j)==6 )
			{
				c = oBody->joint(j).ik_rigidity_coeff;
				for( i=0; i<num_equations; i++ )
				{
					J(i, jj  ) *= c;
					J(i, jj+1) *= c;
					J(i, jj+2) *= c;
				}

				c = oBody->joint(j).ik_rigidity_coeff;
				for( i=0; i<num_equations; i++ )
				{
					J(i, jj+3) *= c;
					J(i, jj+4) *= c;
					J(i, jj+5) *= c;
				}
			}
			else if ( oBody->GetDOF(j)==3 )
			{
				c = oBody->joint(j).ik_rigidity_coeff;
				for( i=0; i<num_equations; i++ )
				{
					J(i, jj  ) *= c;
					J(i, jj+1) *= c;
					J(i, jj+2) *= c;
				}
			}
			else if ( oBody->GetDOF(j)==1 )
			{
				c = oBody->joint(j).ik_rigidity_coeff;
				for( i=0; i<num_equations; i++ )
				{
					J(i, jj  ) *= c;
				}
			}

			jj += oBody->GetDOF(j);
		}
}


static void
scalingCoefficients( cml::vectord &x )
{
	double c;

	for( int j=0, jj=0; j<(int)oBody->num_joint(); j++ )
		//if ( link_mask & MaskBit(j) )
		{
			if ( oBody->GetDOF(j)==6 )
			{
				c = oBody->joint(j).ik_rigidity_coeff;
				x[jj++] *= c;
				x[jj++] *= c;
				x[jj++] *= c;

				c = oBody->joint(j).ik_rigidity_coeff;
				x[jj++] *= c;
				x[jj++] *= c;
				x[jj++] *= c;
			}
			else if ( oBody->GetDOF(j)==3 )
			{
				c = oBody->joint(j).ik_rigidity_coeff;
				x[jj++] *= c;
				x[jj++] *= c;
				x[jj++] *= c;
			}
			else if ( oBody->GetDOF(j)==1 )
			{
				c = oBody->joint(j).ik_rigidity_coeff;
				x[jj++] *= c;
			}
		}
}

static double
incorporateDamping( cml::vectord const& d, cml::vectord& dp )
{
	double dist = 0;
	double c1, c2;

	for( int j=0, jj=0; j<(int)oBody->num_joint(); j++ )
		//if ( link_mask & MaskBit(j) )
		{
			c1 = oBody->joint(j).ik_damping_coeff;
			c2 = 2.0 * c1;

			if ( oBody->GetDOF( j ) == 6 )
			{
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;

				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
			}
			else if ( oBody->GetDOF( j ) == 3 )
			{
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
			}
			else if ( oBody->GetDOF( j ) == 1 )
			{
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
			}
		}

	return dist;
}

/**
Not Implemented!!!!!!!
*/
static double
incorporateJointLimit( cml::vectord const& d, cml::vectord& dp )
{
	double dist = 0.0;
	double a, c;
	cml::quaterniond q;
	cml::vector3d v;

	for( int j=0, jj=0; j<(int)oBody->num_joint(); j++ )
		//if ( link_mask & MaskBit(j) )
	{
			if ( oBody->GetDOF( j ) == 3 ||
				oBody->GetDOF( j ) == 1 )
			{
				// 				Not Implemented!!!!!!!
				// QmComplex *angle_bound = oBody->getAngleBound(j); ??

				// if ( angle_bound ) ???
				if ( false )
				{
					c = oBody->joint(j).ik_joint_limit_coeff;

					cml::quaternion_rotation_matrix(q, tPosture.m_rotates[ j ] );
					/*a = angle_bound->distance(q);
					v = angle_bound->gradient(q);*/

					dist += c * (a*a);
				}
			}

			if ( oBody->GetDOF( j ) == 3 )
			{
				dp[jj  ] -= 2.0 * c * v[0];
				dp[jj+1] -= 2.0 * c * v[1];
				dp[jj+2] -= 2.0 * c * v[2];
			}
			else if ( oBody->GetDOF( j ) == 1 )
			{
				dp[jj] -= 2.0 * c * v.length();
			}

			jj += oBody->GetDOF( j );
		}

	return dist;
}

static double  gradientFuncLS( cml::vectord const& d, cml::vectord& dp )
{
	tPosture = oPosture;
	tPosture.AddDisplacement( d );

	double	dist = computeJacobian();
	scalingJacobian( J );

	if ( PenvIKsolverMethod == PENV_IK_LS_SVD_METHOD )
	{
		dp = cml::solve(J, b, 1.0e-2);
		////dp.solve( J, b, 1.0e-2 );
	}
	else
	{
		static cml::matrixd Jt; Jt = cml::transpose( J );
		static cml::matrixd Jp; Jp = Jt * J;
		static cml::vectord bp; bp = Jt * b;

		for( int k=0; k<(int)Jp.rows(); k++ ) Jp(k, k) += DAMPING;

		dp = cml::lu_solve(Jp, bp);
		//dp.solve( Jp, bp );
	}

	scalingCoefficients( dp );

	if ( PenvDampingEnabled )	 dist += incorporateDamping( d, dp );
	if ( PenvJointLimitEnabled ) dist += incorporateJointLimit( d, dp );

	return dist;
}

static double  gradientFunc( cml::vectord const& d, cml::vectord& dp )
{
	int i, j;

	tPosture = oPosture;
	tPosture.AddDisplacement( d );

	double dist = computeJacobian();

	for( i=0; i<num_dof; i++ )
	{
		dp[i] = 0;

		for( j=0; j<constraint.GetDOC(); j++ )
			dp[i] -= 2.0 * J(j, i) * b[j];
	}

	if ( PenvDampingEnabled )	 dist += incorporateDamping( d, dp );
	if ( PenvJointLimitEnabled ) dist += incorporateJointLimit( d, dp );

	return dist;
}