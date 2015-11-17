#ifndef ObservationProbabilitiesByFunction_H
#define ObservationProbabilitiesByFunction_H

#include "Const.h"
#include "Observations.h"
#include "Actions.h"
#include "States.h"
#include "MathLib.h"
#include "VariableRelation.h"
#include "ObservationProbabilities.h"


using namespace std;
using namespace momdp;
namespace momdp 
{
	class MOMDP;
	class ObservationProbabilitiesByFunction : public ObservationProbabilities
	{
	private:
		SharedPointer<SparseMatrix> getMatrixInner(int a, int x, bool transpose);

	public:
		ObservationProbabilitiesByFunction(void);
		virtual ~ObservationProbabilitiesByFunction(void);

		REAL_VALUE prob(Observations::iterator& o, States::iterator& x, States::iterator& y, Actions::iterator& a);

		virtual SharedPointer<SparseMatrix> getMatrix(int a, int x);
		virtual SharedPointer<SparseMatrix> getMatrixTr(int a, int x);

		vector<SharedPointer<VariableRelation> > relations;

		MOMDP* problem;
	};
}

#endif




