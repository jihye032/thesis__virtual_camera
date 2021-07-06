#ifdef ODE_EXT

#include "BaseLib/OdeExt/UrdfMotion.h"
#include "BaseLib/CmlExt/CmlExt.h"
#include "PmQm/qmComplex.h"
#include <algorithm>

//----------------------------------------------------------//
//															//
//			  Inverse kinematics for the full body 			//
//															//
//----------------------------------------------------------//

#define DAMPING			1.0

static m_real  energyFunc( vectorN const& );
static m_real  gradientFuncLS( vectorN const&, vectorN& );
static m_real  gradientFunc( vectorN const&, vectorN& );
static m_real  computeJacobian();

static int				g_num_dof,
						g_num_equations,
						g_num_unknowns;
static Urdf*			g_oBody;
static UrdfPose		g_oPosture;
static UrdfPose		g_tPosture;
static std::vector<UrdfConstraint>		g_constraint;

static matrixN			g_J;
static vectorN			g_b;

// static m_real g_ik_error_bound = 0.001;
// static m_real g_ik_tolerance = 0.1;

static const int g_IK_CG_METHOD = 0;
static const int g_IK_LS_SVD_METHOD = 1;
static const int g_IK_LS_DAMP_METHOD = 2;
static int g_ik_solver_method = g_IK_LS_DAMP_METHOD;

static bool g_damping_enabled = false;
static bool g_joint_limit_enabled = true;

static m_real g_PenvJointLimitCoeff = 100.0;

void
UrdfPose::SolveIK(double ik_error_bound, double ik_tolerance)
{
	int max = 200;

	for ( int i=0; i<max; i++ )
	{
		if ( SolveIKOnce(ik_error_bound, ik_tolerance) <ik_error_bound ) return;
	}
}

double
UrdfPose::SolveIKOnce(double ik_error_bound, double ik_tolerance)
{
	double error=0;
	if ( position_constraints_.empty() ) return error;
	// constraint = c;
	// link_mask = this->getMask() & m;
	// oBody = getBody();

	g_constraint.assign( this->position_constraints_.begin(), this->position_constraints_.end() );
	g_oBody = urdf_;

	g_oPosture = (*this);
	g_tPosture = (*this);

	g_num_dof = g_oPosture.urdf()->degree_of_freedom();

	static vectorN d; d.setSize(g_num_dof);

	int i;
	for( i=0; i<g_num_dof; i++ ) d[i] = 0.0;

	m_real f;
	int iter = 0;

	if ( (error=energyFunc(d)) > ik_error_bound )
	{
		//printf("PENV_IK_CG_METHOD %d\n", PenvIKsolverMethod);
		if ( g_ik_solver_method == g_IK_CG_METHOD )
			frprmn( d, g_num_dof, ik_tolerance, iter, f, energyFunc, gradientFunc );
		else
			gradient_descent( d, g_num_dof, ik_tolerance, iter, f, energyFunc, gradientFuncLS );
	}

	g_oPosture.AddDisplacement( d );
	g_oPosture.PushAnglesInBound();
	g_oPosture.CopyTo( *this );


	return error;

	// if (c.getMask(PmHuman::RIGHT_PALM)) ik_arm_new( *this, PmHuman::RIGHT_PALM, c.get(PmHuman::RIGHT_PALM).getTranslation() );
	// if (c.getMask(PmHuman::LEFT_PALM))  ik_arm_new( *this, PmHuman::LEFT_PALM, c.get(PmHuman::LEFT_PALM).getTranslation() );
	// if (c.getMask(PmHuman::RIGHT_FOOT)) ik_arm_new( *this, PmHuman::RIGHT_FOOT, c.get(PmHuman::RIGHT_FOOT).getTranslation() );
	// if (c.getMask(PmHuman::LEFT_FOOT))  ik_arm_new( *this, PmHuman::LEFT_FOOT, c.get(PmHuman::LEFT_FOOT).getTranslation() );
}

static double
getRigidityCoeff(int j)
{
	return 1.0;
}

static double
getDampingCoeff(int j)
{
	return 1.0;
}

//------------------------------------------------------//
//														//
//	 			Computing Energy Function				//
//														//
//------------------------------------------------------//

static m_real
energyFunc( vectorN const&d )
{
	g_tPosture = g_oPosture;
	g_tPosture.AddDisplacement( d );

	m_real	dist = 0.0;
	vector	dv, dq, dc;
	quater	q;

	for( unsigned int i=0; i<g_constraint.size(); i++ )
	{
		UrdfConstraint &c = g_constraint[i];

		if ( c.type_ == UrdfConstraint::CON_POSITION )
		{
			dv = g_tPosture.GetGlobalJointPosition( c.joint_id_ ) - c.position_;
			dist += dv%dv;
		}
	}


	if ( g_damping_enabled )
		for( int j=0, jj=0; j<g_oBody->num_joints(); j++ )
		if ( g_oBody->is_active_joint(j) )
		{
			m_real c1 = getDampingCoeff(j+1);
/*
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
			else if ( tPosture.getDOF( j ) == 1 )*/
			{
				dist += c1*d[jj]*d[jj]; jj++;
			}
		}

	
		/*
	if ( g_joint_limit_enabled )
		for( int j=0; j<g_oBody->num_joints(); j++ )
		if ( g_oBody->is_active_joint(j) )
		{
			QmComplex *angle_bound = g_oBody->getAngleBound(j);

			if ( angle_bound )
			{
				q = tPosture.getRotation( j );
				m_real a = angle_bound->distance(q);
				dist += PenvJointLimitCoeff[j+1] * a * a;
			}
		}
		*/

	if ( g_joint_limit_enabled )
		for( int j=0; j<g_oBody->num_joints(); j++ )
		if ( g_oBody->is_active_joint(j) )
		{
			double angle = g_tPosture.angle(j);
			double dist_from_boundary = 0.0;

			if ( g_oBody->joint(j)->limit_lower_
				< g_oBody->joint(j)->limit_upper_ )
			{
				if ( angle < g_oBody->joint(j)->limit_lower_ )
				{
					dist_from_boundary = angle - g_oBody->joint(j)->limit_lower_;
				}
				else if ( angle > g_oBody->joint(j)->limit_upper_ )
				{
					dist_from_boundary = g_oBody->joint(j)->limit_upper_ - angle;
				}

				dist += g_PenvJointLimitCoeff * dist_from_boundary * dist_from_boundary;
			}
		}
	
	return dist;
}


//------------------------------------------------------//
//														//
// 				Computing Jacobian matrix				//
//														//
//------------------------------------------------------//

static m_real
computeJacobian()
{
	m_real	dist = 0.0;
	vector	dv, dq, dc;
	transf	t;
	vector	endTrans;
	quater	endRot;

	//int		i, j;
	vector	w1, w2, w3;

	g_num_equations = std::max( (int)g_constraint.size()*3, g_num_dof );
	g_num_unknowns  = g_num_dof;

	g_J.setSize( g_num_equations, g_num_unknowns );
	g_b.setSize( g_num_equations );

	//static m_real mass[PM_HUMAN_NUM_LINKS];
	//static vector cog[PM_HUMAN_NUM_LINKS];

	for( int i=0; i<g_num_equations; i++ )
	{
		g_b[i] = 0.0;
		for( int j=0; j<g_num_unknowns; j++ )
			g_J[i][j] = 0.0;
	}

	// int	ii, iii, jj;

	for( int i=0, ii=0, iii=0; i<(int)g_constraint.size(); i++ )
	{
		UrdfConstraint &c = g_constraint[i];

		endTrans = g_tPosture.GetGlobalJointPosition( c.joint_id_ );
		// endRot   = g_tPosture.getGlobalRotation( c.link );

		int temp = ii;

		if ( c.type_ == UrdfConstraint::CON_POSITION )
		{
			dv = c.position_ - endTrans;
			dist += dv%dv;

			g_b[ii++] = dv.x();
			g_b[ii++] = dv.y();
			g_b[ii++] = dv.z();
		}


		for( int j=0, jj=0; j<g_oBody->num_joints(); j++ )
		{
			if ( g_oBody->is_active_joint(j) )
			{
				if ( g_oBody->IsAncestorOf( j, c.joint_id_ ) )
				{
					iii = temp;

					if ( true )//g_tPosture.getDOF( j ) == 1 )
					{
						if ( c.type_ == UrdfConstraint::CON_POSITION )
						{
							//t  = g_tPosture.getGlobalTransf(j);
							//dv = vector(1,0,0) * t * (endTrans - t.getTranslation());
							dv = rotate(g_tPosture.GetGlobalJointRotation(j), g_oBody->joint(j)->axis_) * (endTrans - g_tPosture.GetGlobalJointPosition(j));

							g_J[iii  ][jj] = dv[0];
							g_J[iii+1][jj] = dv[1];
							g_J[iii+2][jj] = dv[2];

							iii += 3;
						}

				
					}
				}

				jj ++;
			}
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
scalingJacobian( matrixN &J )
{
	int i, j, jj;
	m_real  c;

	for( j=0, jj=0; j<g_oBody->num_joints(); j++ )
		if ( g_oBody->is_active_joint(j) )
		{
			/*if ( tPosture.getDOF(j)==6 )
			{
				c = PenvRigidityCoeff[j];
				for( i=0; i<num_equations; i++ )
				{
					J[i][jj  ] *= c;
					J[i][jj+1] *= c;
					J[i][jj+2] *= c;
				}

				c = PenvRigidityCoeff[j+1];
				for( i=0; i<num_equations; i++ )
				{
					J[i][jj+3] *= c;
					J[i][jj+4] *= c;
					J[i][jj+5] *= c;
				}
			}
			else if ( tPosture.getDOF(j)==3 )
			{
				c = PenvRigidityCoeff[j+1];
				for( i=0; i<num_equations; i++ )
				{
					J[i][jj  ] *= c;
					J[i][jj+1] *= c;
					J[i][jj+2] *= c;
				}
			}
			else */ if ( true )//g_tPosture.getDOF(j)==1 )
			{
				c = getRigidityCoeff(j);
				for( i=0; i<g_num_equations; i++ )
				{
					J[i][jj  ] *= c;
				}
			}

			//jj += tPosture.getDOF(j);
			jj ++;//= tPosture.getDOF(j);
		}
}

static void
scalingCoefficients( vectorN &x )
{
	m_real c;

	for( int j=0, jj=0; j<g_oBody->num_joints(); j++ )
		if ( g_oBody->is_active_joint(j) )
		{
			/*if ( tPosture.getDOF(j)==6 )
			{
				c = PenvRigidityCoeff[j];
				x[jj++] *= c;
				x[jj++] *= c;
				x[jj++] *= c;

				c = PenvRigidityCoeff[j+1];
				x[jj++] *= c;
				x[jj++] *= c;
				x[jj++] *= c;
			}
			else if ( tPosture.getDOF(j)==3 )
			{
				c = PenvRigidityCoeff[j+1];
				x[jj++] *= c;
				x[jj++] *= c;
				x[jj++] *= c;
			}
			else if ( tPosture.getDOF(j)==1 )*/
			{
				// c = PenvRigidityCoeff[j+1];
				c = getRigidityCoeff(j);
				x[jj++] *= c;
			}
		}
}

static m_real
incorporateDamping( vectorN const& d, vectorN& dp )
{
	m_real dist = 0;
	m_real c1, c2;

	for( int j=0, jj=0; j<g_oBody->num_joints(); j++ )
		if ( g_oBody->is_active_joint(j) )
		{
			c1 = getDampingCoeff(j+1);
			c2 = 2.0 * c1;

			/*if ( g_tPosture.getDOF( j ) == 6 )
			{
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;

				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
			}
			else if ( tPosture.getDOF( j ) == 3 )
			{
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
			}
			else if ( tPosture.getDOF( j ) == 1 )*/
			{
				dist += c1*d[jj]*d[jj]; dp[jj] -= c2*d[jj]; jj++;
			}
		}

	return dist;
}

static m_real
incorporateJointLimit( vectorN const& d, vectorN& dp )
{
	m_real dist = 0.0;

	for( int j=0, jj=0; j<g_oBody->num_joints(); j++ )
		if ( g_oBody->is_active_joint(j) )
		{
			double angle = g_tPosture.angle(j);

			double dist_from_broundary = 0.0;
			if ( g_oBody->joint(j)->limit_lower_
				< g_oBody->joint(j)->limit_upper_ )
			{
				if ( angle < g_oBody->joint(j)->limit_lower_ )
				{
					dist_from_broundary = angle - g_oBody->joint(j)->limit_lower_;
					//dist_from_broundary = g_oBody->joint(j)->limit_lower_ - angle;
				}
				else if ( angle > g_oBody->joint(j)->limit_upper_ )
				{
					//dist_from_broundary = g_oBody->joint(j)->limit_upper_ - angle;
					dist_from_broundary = angle-g_oBody->joint(j)->limit_upper_;
				}
			}

			dist += g_PenvJointLimitCoeff * SQR(dist_from_broundary);
			//dp[jj] -= 2.0 * g_PenvJointLimitCoeff * abs(dist_from_broundary);
			//dp[jj] -= 2.0 * g_PenvJointLimitCoeff * (dist_from_broundary);
			dp[jj] -=  2.0 * (dist_from_broundary);
			
			jj++;
		}

	return dist;
}

static m_real
gradientFuncLS( vectorN const&d, vectorN& dp )
{
	g_tPosture = g_oPosture;
	g_tPosture.AddDisplacement( d );

	m_real	dist = computeJacobian();
	scalingJacobian( g_J );

	if ( g_ik_solver_method == g_IK_LS_SVD_METHOD )
		dp.solve( g_J, g_b, 1.0e-2 );
	else
	{
		static matrixN Jt; Jt.transpose( g_J );
		static matrixN Jp; Jp.mult( Jt, g_J );
		static vectorN bp; bp.mult( Jt, g_b );

		for( int k=0; k<Jp.row(); k++ ) Jp[k][k] += DAMPING;

		dp.solve( Jp, bp );
	}

	scalingCoefficients( dp );

	if ( g_damping_enabled )	 dist += incorporateDamping( d, dp );
	if ( g_joint_limit_enabled ) dist += incorporateJointLimit( d, dp );

	return dist;
}

static m_real
gradientFunc( vectorN const& d, vectorN& dp )
{
	int i, j;

	g_tPosture = g_oPosture;
	g_tPosture.AddDisplacement( d );

	m_real dist = computeJacobian();

	for( i=0; i<g_num_dof; i++ )
	{
		dp[i] = 0;

		for( j=0; j<(int)g_constraint.size()*3; j++ )
			dp[i] -= 2.0 * g_J[j][i] * g_b[j];
	}

	if ( g_damping_enabled )	 dist += incorporateDamping( d, dp );
	if ( g_joint_limit_enabled ) dist += incorporateJointLimit( d, dp );

	return dist;
}

#endif
