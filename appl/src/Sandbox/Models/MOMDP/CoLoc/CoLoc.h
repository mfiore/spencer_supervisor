#ifndef CoLoc_H
#define CoLoc_H

#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <stdexcept>
#include "MOMDP.h"
#include "Observations.h"
#include "Actions.h"
#include "States.h"

#include "Rewards.h"

#include "ObservationProbabilities.h"
#include "StateTransitionX.h"
#include "StateTransitionXY.h"

#include "Belief.h"
#include "BeliefTransition.h"

#include "MObject.h"
#include "MathLib.h"

#include "Map2D.h"
#include "Map2DPosVar.h"
#include "Map2DPosValue.h"
#include "Variable.h"
#include "VariableValue.h"
#include "Map2DPath.h"

using namespace std;
using namespace momdp;

namespace momdp 
{

	class CoLoc: public MOMDP
	{
	private:
		Map2D baseMap;
		SharedPointer<Map2DPath> baseMapPath;

	public:
				
		CoLoc(void); // default constructor

		virtual ~CoLoc(void);

		virtual string ToString();

		virtual REAL_VALUE getDiscount ()
		{
			return discount;
		}

		virtual bool hasPOMDPMatrices();
		
		virtual obsState_prob_vector& getObsStateProbVector(obsState_prob_vector& result, BeliefWithState& b, int a);
		virtual SparseVector& getJointUnobsStateProbVector(SparseVector& result, SharedPointer<BeliefWithState> b,	int a, int Xn);

		virtual int getNumActions();
		virtual int getBeliefSize();

		//virtual bool getIsTerminalState(BeliefWithState &b); inherit from MOMDP's version

		virtual void getObsProbVectorFast(obs_prob_vector& result, int a, int Xn, SparseVector& tmp1);
		virtual void getObsProbVector(obs_prob_vector& result, const BeliefWithState& b, int a, int Xn);


		//
		virtual map<string, string> getActionsSymbols(int actionNum);
		virtual map<string, string> getFactoredObservedStatesSymbols(int stateNum) ;
		virtual map<string, string> getFactoredUnobservedStatesSymbols(int stateNum) ;
		virtual map<string, string> getObservationsSymbols(int observationNum) ;
	};
}
#endif
