#ifndef StateTransitionXYByFunction_H
#define StateTransitionXYByFunction_H

#include "Const.h"
#include "Observations.h"
#include "Actions.h"
#include "States.h"
#include "MathLib.h"
#include "VariableRelation.h"
#include "StateTransitionXY.h"
using namespace std;
using namespace momdp;
namespace momdp 
{
	class MOMDP;
	class StateTransitionXYByFunction : public StateTransitionXY
	{
	private:
		SharedPointer<SparseMatrix> getMatrixInner(int a, int x, bool transpose);

	public:
		StateTransitionXYByFunction(void);
		virtual ~StateTransitionXYByFunction(void);

		REAL_VALUE prob(States::iterator& x, States::iterator& y, States::iterator& newX, Actions::iterator& a, States::iterator& xp, States::iterator& yp);


		virtual SharedPointer<SparseMatrix> getMatrix(int a, int x);
		virtual SharedPointer<SparseMatrix> getMatrixTr(int a, int x);

		vector<SharedPointer<VariableRelation> > relations;
		MOMDP* problem;
	};
}

#endif

