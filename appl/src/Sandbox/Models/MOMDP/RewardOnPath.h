#ifndef RewardOnPath_H
#define RewardOnPath_H


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
#include "Map2DPath.h"
#include "ProblemParams.h"

using namespace std;
using namespace momdp;

namespace momdp
{
	// generic class for variable, holds user defined variable value
	class RewardOnPath: public VariableRelation
	{
	private:
		SharedPointer<Variable> rewardVar;
		SharedPointer<Map2DPosVar> auvPos;
		SharedPointer<ProblemParams> params;
		SharedPointer<Map2DPath> path;

	public:
		RewardOnPath(SharedPointer<Variable> rewardVar, SharedPointer<Map2DPosVar> auvPos, SharedPointer<Map2DPath> path, SharedPointer<ProblemParams> params);
		virtual ~RewardOnPath(void);
		
		virtual vector<SharedPointer<RelEntry> > getProb(map<string, SharedPointer<IVariableValue> > sourceVals);

	};

}

#endif

