#ifndef Obs2DBeaconNoisy_H
#define Obs2DBeaconNoisy_H


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
#include "BooleanVariable.h"
#include "InRangeAllNto1VarRel.h"
using namespace std;
using namespace momdp;

namespace momdp
{
	// generic class for variable, holds user defined variable value
	class Obs2DBeaconNoisy: public VariableRelation
	{
	private:

		SharedPointer<Map2DPosVar> suv1PosVar;
		SharedPointer<Map2DPosVar> suv2PosVar;
		SharedPointer<Map2DPosVar> auvPosVar;
		SharedPointer<Map2DPosVar> auvEstPosVar;
		SharedPointer<InRangeAllNto1VarRel> auvPosEst;
		SharedPointer<ProblemParams> params;
		SharedPointer<BooleanVariable> auvGetsGPSPosVar;
		SharedPointer<BooleanVariable> isCal;
		SharedPointer<Variable> obsVar;

	public:
		Obs2DBeaconNoisy(SharedPointer<Variable> obsVar, SharedPointer<Map2DPosVar> suv1PosVar, SharedPointer<Map2DPosVar> suv2PosVar, SharedPointer<Map2DPosVar> auvPosVar, SharedPointer<BooleanVariable> auvGetsGPSPos, SharedPointer<ProblemParams> params );
		virtual ~Obs2DBeaconNoisy(void);
		
		virtual vector<SharedPointer<RelEntry> > getProb(map<string, SharedPointer<IVariableValue> > sourceVals);

	};

}

#endif

