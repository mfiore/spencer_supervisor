#ifndef NoisyVariableRel_H
#define NoisyVariableRel_H


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
using namespace std;
using namespace momdp;

namespace momdp
{
	// generic class for variable, holds user defined variable value
	class NoisyVariableRel: public VariableRelation
	{
	private:
		SharedPointer<IVariableValue> noisyValue;
		SharedPointer<IVariableValue> otherValue;
		double noisyProbability;
		SharedPointer<VariableRelation> origRel;

	public:
		
		NoisyVariableRel(SharedPointer<VariableRelation> origRel, SharedPointer<IVariableValue> noisyValue, double noisyProbability, SharedPointer<IVariableValue> otherValue);
			
		virtual ~NoisyVariableRel(void);
		
	
		virtual vector<SharedPointer<RelEntry> > getProb(map<string, SharedPointer<IVariableValue> > sourceVals) ;

	};

}

#endif

