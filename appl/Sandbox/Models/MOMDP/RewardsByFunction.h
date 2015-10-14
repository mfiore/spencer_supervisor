#ifndef RewardsByFunction_H
#define RewardsByFunction_H

#include "Const.h"
#include "Observations.h"
#include "Actions.h"
#include "States.h"
#include "MathLib.h"
#include "BeliefWithState.h"
#include "IVariable.h"
#include "VariableRelation.h"
#include "Rewards.h"

using namespace std;
using namespace momdp;
namespace momdp 
{
	class MOMDP;
	class RewardsByFunction : public Rewards
	{
	private:
		SharedPointer<SparseMatrix> getMatrixInner(int x, bool transpose);
		
	public:
		RewardsByFunction(void);
		virtual ~RewardsByFunction(void);

		vector<SharedPointer<IVariable> > vars;

		virtual REAL_VALUE getReward(BeliefWithState& belief, int a);
		REAL_VALUE reward(States::iterator& x, States::iterator& y, Actions::iterator& a);

		virtual SharedPointer<SparseMatrix> getMatrix(int x);
		//SharedPointer<SparseMatrix> getMatrixTr(int x);

		vector<SharedPointer<VariableRelation> > relations;
		MOMDP* problem;
	};
}
#endif


