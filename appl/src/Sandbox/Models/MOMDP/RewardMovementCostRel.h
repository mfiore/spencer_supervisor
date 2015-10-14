#ifndef RewardMovementCostRel_H
#define RewardMovementCostRel_H


#include <vector>
#include <string>
#include "MObject.h"
#include "IVariableValue.h"
#include "IVariable.h"
#include "VariableValue.h"
#include "Variable.h"
#include "VariableRelation.h"
#include "Map2DPosValue.h"
#include "Map2DPosVar.h"
#include "ProblemParams.h"

using namespace std;
using namespace momdp;

namespace momdp
{
	// generic class for variable, holds user defined variable value
	class RewardMovementCostRel: public VariableRelation
	{
	private:
		SharedPointer<Variable> rewardVar;
		SharedPointer<Variable> actionVar;
		SharedPointer<ProblemParams> params;

	public:
		RewardMovementCostRel(SharedPointer<Variable> rewardVar, SharedPointer<Variable> actionVar, SharedPointer<ProblemParams> params);
		virtual ~RewardMovementCostRel(void);
		
		virtual vector<SharedPointer<RelEntry> > getProb(map<string, SharedPointer<IVariableValue> > sourceVals);

	};

}

#endif

