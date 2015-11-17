#ifndef InRangeAllNto1VarRel_H
#define InRangeAllNto1VarRel_H


#include <vector>
#include <string>
#include "MObject.h"
#include "IVariableValue.h"
#include "IVariable.h"
#include "VariableValue.h"
#include "Variable.h"
#include "BooleanVariable.h"
#include "VariableRelation.h"
#include "Map2DPosValue.h"
#include "Map2DPosVar.h"
using namespace std;
using namespace momdp;

namespace momdp
{
	// generic class for variable, holds user defined variable value
	class InRangeAllNto1VarRel: public VariableRelation
	{
	private:
		 vector<SharedPointer<Map2DPosVar> > mapSrcVars;
		SharedPointer<Map2DPosVar> mapDestVar;
		SharedPointer<BooleanVariable> resultVar;
		double range;

	public:
		
		InRangeAllNto1VarRel(SharedPointer<BooleanVariable> resultVar, SharedPointer<Map2DPosVar> mapDestVar, double range);
		virtual ~InRangeAllNto1VarRel(void);
		
		void addSrcVar(SharedPointer<Map2DPosVar> srcVar);
		virtual vector<SharedPointer<RelEntry> > getProb(map<string, SharedPointer<IVariableValue> > sourceVals) ;

	};

}

#endif

