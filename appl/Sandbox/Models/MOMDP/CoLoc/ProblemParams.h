#ifndef ProblemParams_H
#define ProblemParams_H


#include "MObject.h"
using namespace momdp;
namespace momdp 
{

	class ProblemParams : public MObject
	{
	public:
		double AUVtoSUVProb; // r
		double SUVtoAUVProb; // p
		double NoisyAUVObsRadius; // D , not a circular radius, it is the greatest delta in x and/or y
		double NoisyAUVObsProb; // s
		double AUVBlindMoveProb; // q
		double MovementCost;
		double OnPathReward;
		double OnGoalReward;
		double ModemRange;
		double DiscountFactor;
		double OffPathPenalty;

		ProblemParams()
		{
			AUVtoSUVProb = 0.9; // r
			SUVtoAUVProb = 0.9; // p
			NoisyAUVObsRadius = 1; // D , not a circular radius, it is the greatest delta in x and/or y
			NoisyAUVObsProb = 0.8; // s
			AUVBlindMoveProb = 0.85; // q
			MovementCost = -10;
			OnPathReward = 10;
			OnGoalReward = 100;
			ModemRange = 1.5;
			DiscountFactor = 0.95;
			OffPathPenalty = -10;
		}

	};
}

#endif



