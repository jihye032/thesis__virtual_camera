

#include "MotionDTW.h"
#include <algorithm>
#include <list>

namespace ml
{

MotionDTW::MotionDTW()
{
}

MotionDTW::MotionDTW(const Motion *target, const Motion *input)
{
	SetMotions(target, input);
}

//
//int MotionDTW::getCorrespAtoB(int i)
//{
//	if ( i < 0 ) return 0;
//	if ( i >= (int)correspAtoB.size() ) return motionB->getSize() - 1;
//
//	return correspAtoB[i];
//}
//
//int MotionDTW::getCorrespBtoA(int i)
//{
//	if ( i < 0 ) return 0;
//	if ( i >= (int)correspBtoA.size() ) return motionA->getSize() - 1;
//
//	return correspBtoA[i];
//}


void MotionDTW::SetMotions(const Motion *target, const Motion *input)//, PmLinearMotion &outMotion)
{

	this->motion_A_ = target;
	this->motion_B_ = input;


	const Motion &ma = *motion_A_;
	const Motion &mb = *motion_B_;
	int ma_size = ma.size();
	int mb_size = mb.size();

	double **dist_table  = new double*[ma_size];
	int **trace_table  = new int*[ma_size];
	for ( int i=0; i<ma_size; i++ )
	{
		dist_table[i] = new double[mb_size];
		trace_table[i] = new int[mb_size];
	}

	const int AA = 0;	// increase one frame of motionA. 
	const int BB = 1;	// increase one frame of motionB.
	const int AB = 2;	// increase one frame of both motionA and B.

	// fill distance table
	for ( int i=0; i<ma_size; i++ )
	for ( int j=0; j<mb_size; j++ )
	{
		dist_table[i][j] = DiffPoseLocalPos(ma, i, mb, j);
	}

	// from the first frame of B to every A
	for ( int i=1; i<ma_size; i++ )
	{
		dist_table[i][0] += dist_table[i-1][0];
		trace_table[i][0] = AA;
	}

	// from the first frame of A to every B
	for ( int i=1; i<mb_size; i++ )
	{
		dist_table[0][i] += dist_table[0][i-1];
		trace_table[0][i] = BB;
	}

	// dynamic programming
	for ( int i=1; i<ma_size; i++ )
	for ( int j=1; j<mb_size; j++ )
	{
		double moving_a = dist_table[i][j] + dist_table[i-1][j];
		double moving_b = dist_table[i][j] + dist_table[i][j-1];
		double moving_ab = dist_table[i][j] + dist_table[i-1][j-1];

		if ( moving_a < moving_b && moving_a < moving_ab )
		{
			dist_table[i][j] = moving_a;
			trace_table[i][j] = AA;
		}
		else if ( moving_b < moving_a && moving_b < moving_ab )
		{
			dist_table[i][j] = moving_b;
			trace_table[i][j] = BB;
		}
		else
		{
			dist_table[i][j] = moving_ab;
			trace_table[i][j] = AB;
		}
		
	}

	std::list<std::pair<int, int> > trace;
	int af = ma_size-1;
	int bf = mb_size-1;

	trace.push_front(std::make_pair(af, bf));
	while ( af>0 || bf>0 )
	{
		if ( trace_table[af][bf] == AA ) 
		{
			af -= 1;
		}
		else if ( trace_table[af][bf] == BB )
		{
			bf -= 1;
		}
		else if ( trace_table[af][bf] == AB )
		{
			af -= 1;
			bf -= 1;
		}
		trace.push_front(std::make_pair(af, bf));
	}
	
	dist = dist_table[ma_size-1][mb_size-1] / trace.size();

	dist += exp((double)std::abs((int)ma.size()-(int)mb.size()) / 10);

	// print correlations.
	if ( false )
	{
		printf("\n");
		std::list<std::pair<int, int> >::iterator iter;
		for ( iter=trace.begin(); iter!=trace.end(); iter++ )
		{
			printf("[%d,%d, %0.2f] ", iter->first, iter->second, DiffPoseLocalPos(ma, iter->first, mb, iter->second));
		}
		printf("\n");
	}

	for ( int i=0; i<(int)ma.size(); i++ ) 
	{
		delete[] dist_table[i];
		delete[] trace_table[i];
	}
	delete[] dist_table;
	delete[] trace_table;

}



//
//void MotionDTW::setMotions2(const PmLinearMotion *motionA, const PmLinearMotion *motionB)//, PmLinearMotion &outMotion)
//{
//
//	this->motionA = motionA;
//	this->motionB = motionB;
//
//	const PmLinearMotion &ma = *motionA;
//	const PmLinearMotion &mb = *motionB;
//
//	int i=0, j=0;
//
//	float** dT;
//	float** dT2;
//	short** rT;
//	short** rT2;
//
//	int sizeA = ma.getSize();
//	int sizeB = mb.getSize();
//
//	static const short UP = -1;
//	static const short DA = 0;
//	static const short LT = 1;
//
//
//	// alloc
//	{
//		dT = new float*[sizeA];
//		rT = new short*[sizeA];
//
//		for( i=0; i<sizeA; i++ )
//		{
//			dT[i] = new float[sizeB];
//			rT[i] = new short[sizeB];
//		}
//
//		dT2 = new float*[sizeB];
//		rT2 = new short*[sizeB];
//
//		for( i=0; i<sizeB; i++ )
//		{
//			dT2[i] = new float[sizeA];
//			rT2[i] = new short[sizeA];
//		}
//
//	}
//
//
//	// calcul distances
//	for ( i=0; i<sizeA; i++ )
//	{
//		for ( j=0; j<sizeB; j++ )
//		{
//			dT[i][j] = getPoseDistance(&ma, i, &mb, j);
//			dT2[j][i] = dT[i][j];
//		}
//
//	}
//
//	// fill rT
//	{
//		rT[0][0] = -100;
//
//		// left virtical
//		for ( i=1; i<sizeA; i++ )
//		{
//			dT[i][0] += dT[i-1][0];
//			rT[i][0] = UP;
//		}
//
//		// top horizontal
//		for ( j=1; j<sizeB; j++ )
//		{
//			//dT[0][j] += dT[0][j-1];
//			rT[0][j] = LT;
//		}
//
//		for ( i=1; i<sizeA; i++ )
//		{
//			for ( j=1; j<sizeB; j++ )
//			{
//				float sumDA = dT[i][j] + dT[i-1][j-1];
//				float sumUP = dT[i][j] + dT[i-1][j];
//				float sumLT = dT[i][j] + dT[i][j-1];
//
//				if ( sumDA < sumUP && sumDA < sumLT )
//				{
//					dT[i][j] = sumDA;
//					rT[i][j] = DA;
//				}
//				else if ( sumUP < sumDA && sumUP < sumLT )
//				{
//					dT[i][j] = sumUP;
//					rT[i][j] = UP;
//				}
//				else
//				{
//					dT[i][j] = sumLT;
//					rT[i][j] = LT;
//				}
//			}
//		}
//	}
//
//
//	// fill rT2
//	{
//		rT2[0][0] = -100;
//
//		// left virtical
//		for ( i=1; i<sizeB; i++ )
//		{
//			dT2[i][0] += dT2[i-1][0];
//			rT2[i][0] = UP;
//		}
//
//		// top horizontal
//		for ( j=1; j<sizeA; j++ )
//		{
//			//dT2[0][j] += dT2[0][j-1];
//			rT2[0][j] = LT;
//		}
//
//		for ( i=1; i<sizeB; i++ )
//		{
//			for ( j=1; j<sizeA; j++ )
//			{
//				float sumDA = dT2[i][j] + dT2[i-1][j-1];
//				float sumUP = dT2[i][j] + dT2[i-1][j];
//				float sumLT = dT2[i][j] + dT2[i][j-1];
//
//				if ( sumDA < sumUP && sumDA < sumLT )
//				{
//					dT2[i][j] = sumDA;
//					rT2[i][j] = DA;
//				}
//				else if ( sumUP < sumDA && sumUP < sumLT )
//				{
//					dT2[i][j] = sumUP;
//					rT2[i][j] = UP;
//				}
//				else
//				{
//					dT2[i][j] = sumLT;
//					rT2[i][j] = LT;
//				}
//			}
//		}
//	}
//	
//	correspAtoB.clear();
//	correspBtoA.clear();
//
//	// set correspAtoB
//	{
//		i = sizeA-1;
//		j = sizeB-1;
//		double min = 0.0;
//		for ( int jj=0; jj<sizeB; jj++ )
//		{
//			if ( jj==0 || min > dT[i][jj] )
//			{
//				min = dT[i][jj];
//				j = jj;
//			}
//		}
//		while ( i>0 || j>0 )
//		{
//			correspAtoB.push_back(j);
//			
//			while (true)
//			{
//				switch ( rT[i][j] )
//				{
//				case LT:
//					j--;
//					break;
//				case UP:
//					i--;
//					break;
//				default:
//					i--; j--;
//					break;
//				}
//
//				if ( rT[i][j] == UP || rT[i][j] == DA ) break;
//				if ( i==0 && j==0 ) break;
//			}
//		}
//		
//		while ( (int)correspAtoB.size() < sizeA )
//		{
//			correspAtoB.push_back(0);
//		}
//
//		reverse(correspAtoB.begin(), correspAtoB.end());
//	}
//
//	// set correspBtoA
//	{
//		i = sizeB-1;
//		j = sizeA-1;
//		double min = 0.0;
//		for ( int jj=0; jj<sizeA; jj++ )
//		{
//			if ( jj==0 || min > dT2[i][jj] )
//			{
//				min = dT2[i][jj];
//				j = jj;
//			}
//		}
//		while ( i>0 || j>0 )
//		{
//			correspBtoA.push_back(j);
//			
//			while (true)
//			{
//				switch ( rT2[i][j] )
//				{
//				case LT:
//					j--;
//					break;
//				case UP:
//					i--;
//					break;
//				default:
//					i--; j--;
//					break;
//				}
//
//				if ( rT2[i][j] == UP || rT2[i][j] == DA ) break;
//				if ( i==0 && j==0 ) break;
//			}
//		}
//		
//		while ( (int)correspBtoA.size() < sizeB )
//		{
//			correspBtoA.push_back(0);
//		}
//
//		reverse(correspBtoA.begin(), correspBtoA.end());
//	}
//
//	
//
//	// delete
//	{
//		for ( i=0; i<sizeA; i++ )
//		{
//			delete[] dT[i];
//			delete[] rT[i];
//		}
//		delete[] dT;
//		delete[] rT;
//
//		for ( i=0; i<sizeB; i++ )
//		{
//			delete[] dT2[i];
//			delete[] rT2[i];
//		}
//		delete[] dT2;
//		delete[] rT2;
//	}
//
//}

//
//
//void MotionDTW::getWarpedA(PmLinearMotion &outMotion)
//{
//	// make result-motion.
//	outMotion.setBody( motionA->getBody() );
//    outMotion.setSize( (int)correspBtoA.size() );
//
//
//	for ( int i=0; i<(int)correspBtoA.size(); i++ )
//	{
//		outMotion.setPosture(i, motionA->getPosture( correspBtoA[i] ));
//	}
//}
//
//void MotionDTW::getWarpedB(PmLinearMotion &outMotion)
//{
//	// make result-motion.
//	outMotion.setBody( motionB->getBody() );
//    outMotion.setSize( (int)correspAtoB.size() );
//
//
//	for ( int i=0; i<(int)correspAtoB.size(); i++ )
//	{
//		outMotion.setPosture(i, motionB->getPosture( correspAtoB[i] ));
//	}
//}
//
//
//

};