#include "BeliefTransitionMOMDPLite.h"
#include "exception"
#include <stdexcept>
using namespace std;

BeliefTransitionMOMDPLite::BeliefTransitionMOMDPLite(void)
{
}

BeliefTransitionMOMDPLite::~BeliefTransitionMOMDPLite(void)
{
}


SharedPointer<BeliefWithState> BeliefTransitionMOMDPLite::nextBelief(SharedPointer<BeliefWithState> bp, int a, int o, int obsX )
{
    cout << "not implemented using this belief update .. tirtha 2" << endl;
    throw runtime_error("not implemented");
}

SharedPointer<BeliefWithState> BeliefTransitionMOMDPLite::nextBelief2(SharedPointer<BeliefWithState> bp, int a, int o, int obsX, SharedPointer<SparseVector>& jspv )
{
    cout << "not implemented using this belief update .. tirtha 2" << endl;
    throw runtime_error("not implemented");
}

SharedPointer<BeliefWithState> BeliefTransitionMOMDPLite::nextBelief(SharedPointer<belief_vector>& belY, DenseVector& belX, int a, int o, int obsX)
{
    cout << "not implemented using this belief update .. tirtha 2" << endl;
    throw runtime_error("not implemented");
}