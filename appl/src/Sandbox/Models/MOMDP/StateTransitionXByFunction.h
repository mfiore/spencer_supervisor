#ifndef StateTransitionXByFunction_H
#define StateTransitionXByFunction_H

#include <vector>
#include "Const.h"
#include "Observations.h"
#include "Actions.h"
#include "States.h"
#include "MathLib.h"
#include "VariableRelation.h"
#include "StateTransitionX.h"

using namespace std;
using namespace momdp;
namespace momdp 
{
	class MOMDP;
	class StateTransitionXByFunction : public StateTransitionX
	{
	private:
		SharedPointer<SparseMatrix> getMatrixInner(int a, int x, bool transpose);

	public:
		StateTransitionXByFunction(void);
		virtual ~StateTransitionXByFunction(void);

		REAL_VALUE prob(States::iterator& x, States::iterator& y, Actions::iterator& a, States::iterator& xp);

		virtual SharedPointer<SparseMatrix> getMatrix(int a, int x);
		virtual SharedPointer<SparseMatrix> getMatrixTr(int a, int x);

		vector<SharedPointer<VariableRelation> > relations;
		MOMDP* problem;
	};
}

#endif

