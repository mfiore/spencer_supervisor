#include "CoLoc.h"
#include <string>
#include "ProblemParams.h"
#include "VariableRelation.h"
#include "Move2DVarRel.h"
#include "InRangeAllNto1VarRel.h"
#include "NoisyVariableRel.h"
#include "VariableCombined.h"
#include "BooleanVariable.h"

#include "Move2DPathFollowerActPosHasGuideNoisyVarRel.h"
#include "Obs2DBeaconNoisy.h"
#include "RewardMovementCostRel.h"
#include "RewardOnPath.h"
#include "RewardAtGoal.h"
#include "TriggerVarRel.h"
#include "TerminationVarPaddedRel.h"
#include "StateTransitionXByFunction.h"
#include "StateTransitionXYByFunction.h"
#include "RewardsByFunction.h"
#include "ObservationProbabilitiesByFunction.h"

using namespace std;


CoLoc::CoLoc(void) : MOMDP()
{
	// done by MOMDP constructor
	//beliefTransition = new BeliefTransitionMOMDP();
	//beliefTransition->problem = this;

	//
	//XStates = new States();
	//YStates = new States();
	//actions = new Actions();
	//observations = new Observations();

	
	StateTransitionXByFunction* XTransFunctions  =new StateTransitionXByFunction();
	XTransFunctions->problem = this;
	StateTransitionXYByFunction* XYTransFunctions  =new StateTransitionXYByFunction();
	XYTransFunctions->problem = this;
	RewardsByFunction* RewardsFunctions = new RewardsByFunction();
	RewardsFunctions->problem = this;
	ObservationProbabilitiesByFunction* ObsFunctions = new ObservationProbabilitiesByFunction();
	ObsFunctions->problem = this;

	this->XTrans = XTransFunctions;
	this->XYTrans = XYTransFunctions;
	this->obsProb = ObsFunctions;
	this->rewards = RewardsFunctions;

	//pomdpR = NULL; pomdpT = NULL; pomdpTtr = NULL; pomdpO = NULL;

	SharedPointer<ProblemParams> params(new ProblemParams());
	baseMap.setBounds(3,3);
	
	discount = params->DiscountFactor;

	SharedPointer<BooleanVariable> auvGetsGPSPos (new BooleanVariable("auvGetsGPSPos", 1.0, 0.0));
    SharedPointer<BooleanVariable> termVar (new BooleanVariable("termVar", 0.0, 1.0));

	SharedPointer<Map2DPosVar> suv1Pos = baseMap.makePosVar("suv1Pos");
    SharedPointer<Map2DPosVar> suv2Pos = baseMap.makePosVar("suv2Pos");
    SharedPointer<Map2DPosVar> auvPos = baseMap.makePosVar("auvPos");

	SharedPointer<Map2DPosValue> suv1InitPos = dynamic_pointer_cast<Map2DPosValue>(suv1Pos->getValueByName("x0y0"));
	SharedPointer<Map2DPosValue> suv2InitPos = dynamic_pointer_cast<Map2DPosValue>(suv2Pos->getValueByName("x0y2"));

    suv1Pos->setInitPos(suv1InitPos);
    suv2Pos->setInitPos(suv2InitPos);
	SharedPointer<Map2DPosValue> auvInitPos = dynamic_pointer_cast<Map2DPosValue>(auvPos->getValueByName("x0y1"));
	auvPos->setInitPos(auvInitPos);


	SharedPointer<Map2DPosValue> step1 = dynamic_pointer_cast<Map2DPosValue>(auvPos->getValueByName("x0y1"));
	SharedPointer<Map2DPosValue> step2 = dynamic_pointer_cast<Map2DPosValue>(auvPos->getValueByName("x1y1"));
	SharedPointer<Map2DPosValue> step3 = dynamic_pointer_cast<Map2DPosValue>(auvPos->getValueByName("x2y1"));

	baseMapPath = SharedPointer<Map2DPath> (new Map2DPath());
	baseMapPath->addStep(step1);
	baseMapPath->addStep(step2);
	baseMapPath->addStep(step3);

    SharedPointer<Variable> suv1Action (new Variable("suv1Action"));
    suv1Action->addValue("up");
    suv1Action->addValue("down");
    suv1Action->addValue("left");
    suv1Action->addValue("right");
    suv1Action->addValue("noop");

    SharedPointer<Variable>  suv2Action (new Variable("suv2Action"));
    suv2Action->addValue("up");
    suv2Action->addValue("down");
    suv2Action->addValue("left");
    suv2Action->addValue("right");
    suv2Action->addValue("noop");

    SharedPointer<Move2DVarRel> suv1Move (new Move2DVarRel(suv1Action, suv1Pos));
	SharedPointer<Move2DVarRel> suv2Move (new Move2DVarRel(suv2Action, suv2Pos));

	
    SharedPointer<InRangeAllNto1VarRel> auvGetsGPSPosRel (new InRangeAllNto1VarRel(auvGetsGPSPos, auvPos, params->ModemRange));
    auvGetsGPSPosRel->addSrcVar(suv1Pos);
    auvGetsGPSPosRel->addSrcVar(suv2Pos);
	
	SharedPointer<NoisyVariableRel> auvGetsGPSPosRelNoisy (new  NoisyVariableRel(auvGetsGPSPosRel, auvGetsGPSPos->getTrueValue() , params->SUVtoAUVProb, auvGetsGPSPos->getFalseValue()));

    SharedPointer<Move2DPathFollowerActPosHasGuideNoisyVarRel> auvMov ( new Move2DPathFollowerActPosHasGuideNoisyVarRel(auvPos, auvGetsGPSPos, baseMapPath, params));

	
    // Observation Variables
    SharedPointer<BooleanVariable> obsAuvGetsGPSPos (new BooleanVariable("obsAuvPosCal", 1.0, 0.0));
		 
    SharedPointer<Map2DPosVar> obsAuvPos = baseMap.makePosVar("obsAuvPos");
    SharedPointer<VariableCombined> obsAuvPosWithNAValue ( new VariableCombined("obsAuvPosWithNAValue") );
    obsAuvPosWithNAValue->addSubVar(dynamic_pointer_cast<IVariable>(obsAuvPos));
    obsAuvPosWithNAValue->addSubVar(obsAuvGetsGPSPos);
    obsAuvPosWithNAValue->seal();
    
    //SharedPointer<Variable> obsAuvPosWithNAValue (new Variable("obsAuvPosWithNAValue"));
	
    obsAuvPosWithNAValue->addValue("na");

    SharedPointer<Obs2DBeaconNoisy> obsAuvPosRel (new Obs2DBeaconNoisy(obsAuvPosWithNAValue, suv1Pos, suv2Pos, auvPos, auvGetsGPSPos, params));

		
    SharedPointer<Variable> movementCost1 (new Variable("movementCost1"));
    SharedPointer<RewardMovementCostRel> movementCostRel1 (new RewardMovementCostRel(movementCost1, suv1Action, params));
    SharedPointer<Variable> movementCost2 (new Variable("movementCost2"));
    SharedPointer<RewardMovementCostRel> movementCostRel2 (new RewardMovementCostRel(movementCost2, suv2Action, params));

		
    SharedPointer<Variable> auvOnPathReward (new Variable("auvOnPathReward"));
    SharedPointer<RewardOnPath> auvOnPathRewardRel ( new RewardOnPath(auvOnPathReward, auvPos, baseMapPath, params) );

    SharedPointer<Variable> auvAtGoalReward (new Variable("auvAtGoalReward"));
    SharedPointer<RewardAtGoal> auvAtGoalRewardRel ( new RewardAtGoal(auvAtGoalReward, auvPos, baseMapPath, params) );
        


    // States
	this->XStates->vars.push_back(suv1Pos);
    this->XStates->vars.push_back(suv2Pos);
    this->XStates->vars.push_back(termVar);
    this->YStates->vars.push_back(auvPos);
    this->YStates->vars.push_back(auvGetsGPSPos);
   
    // Actions
	this->actions->vars.push_back(suv1Action);
    this->actions->vars.push_back(suv2Action);

    // Observations
    this->observations->vars.push_back(obsAuvPosWithNAValue);

	
    // Rewards
	this->rewards->vars.push_back(movementCost1);
    this->rewards->vars.push_back(movementCost2);
    this->rewards->vars.push_back(auvOnPathReward);
    this->rewards->vars.push_back(auvAtGoalReward);


	this->XStates->sealAndPopulate();
	this->YStates->sealAndPopulate();
	this->actions->sealAndPopulate();
	this->observations->sealAndPopulate();
	


    // State Transitions
	
	int numSteps = baseMapPath->getPath().size();
    SharedPointer<IVariableValue> lastStepPosStr = baseMapPath->getPath()[numSteps - 1];

	SharedPointer<TriggerVarRel> termVarRel (new TriggerVarRel(termVar, auvPos, lastStepPosStr, termVar->getValueByName("true"), termVar->getValueByName("false") ) );

	SharedPointer<TerminationVarPaddedRel> suv1MovTerm (new TerminationVarPaddedRel(suv1Move, termVar));
	SharedPointer<TerminationVarPaddedRel> suv2MovTerm (new TerminationVarPaddedRel(suv2Move, termVar));
	SharedPointer<TerminationVarPaddedRel> auvMovTerm (new TerminationVarPaddedRel(auvMov, termVar));
	SharedPointer<TerminationVarPaddedRel> auvGetsGPSPosRelNoisyTerm (new TerminationVarPaddedRel(auvGetsGPSPosRelNoisy, termVar));
	SharedPointer<TerminationVarPaddedRel> termVarRelTerm (new TerminationVarPaddedRel(termVarRel, termVar));




	XTransFunctions->relations.push_back(suv1MovTerm);
	XTransFunctions->relations.push_back(suv2MovTerm);
	//XTransFunctions->relations.push_back(auvMovTerm);
	//XTransFunctions->relations.push_back(auvGetsGPSPosRelNoisyTerm);
	XTransFunctions->relations.push_back(termVarRelTerm);
	
	//XYTransFunctions->relations.push_back(suv1MovTerm);
	//XYTransFunctions->relations.push_back(suv2MovTerm);
	XYTransFunctions->relations.push_back(auvMovTerm);
	XYTransFunctions->relations.push_back(auvGetsGPSPosRelNoisyTerm);
	//XYTransFunctions->relations.push_back(termVarRelTerm);

	// Obs probs
	ObsFunctions->relations.push_back(obsAuvPosRel);

    // Rewards
	RewardsFunctions->relations.push_back(movementCostRel1);
    RewardsFunctions->relations.push_back(movementCostRel2);
    RewardsFunctions->relations.push_back(auvOnPathRewardRel);
    RewardsFunctions->relations.push_back(auvAtGoalRewardRel);


	copy(*this->initialBelief , *this->YStates->getInitialProb());
	SharedPointer<SparseVector> initX = this->XStates->getInitialProb();
	if(initX->data.size() > 1 )
	{
		this->initialBeliefStval->sval = -1;
	}
	else
	{
		this->initialBeliefStval->sval = initX->data[0].index;
	}
	copy(*this->initialBeliefStval->bvec, *initX);
	copy(*this->initialBeliefX ,*initX);
	
	// post-process: calculate isPOMDPTerminalState

	int numStatesObs = this->XStates->size();
	int numStatesUnobs = this->YStates->size();
	int numActions = this->actions->size();
	
	this->isPOMDPTerminalState.resize(numStatesObs);

	FOR	(state_idx, numStatesObs) 
	{
		this->isPOMDPTerminalState[state_idx].resize(numStatesUnobs, true);
		FOR (s, numStatesUnobs) 
		{
			FOR (a, numActions) 
			{
				if (   (fabs(1.0 - (XTrans->getMatrix(a,state_idx)->operator ()(s,state_idx)))> OBS_IS_ZERO_EPS) 
					|| (fabs(1.0 - (XYTrans->getMatrix(a,state_idx)->operator ()(s,s))) > OBS_IS_ZERO_EPS) 
					|| (rewards->getMatrix(state_idx)->operator ()(s,a) != 0.0) 
					)
				{
					this->isPOMDPTerminalState[state_idx][s] = false;
					break;
				}
			}
		}
	}
	
}

bool CoLoc::hasPOMDPMatrices()
{
	return false;
}


CoLoc::~CoLoc(void)
{
	//delete beliefTransition;
}

obsState_prob_vector& CoLoc::getObsStateProbVector(obsState_prob_vector& result, BeliefWithState& b, int a)
{
	int Xc = b.sval; // currrent value for observed state variable
	mult( result, *b.bvec, *this->XTrans->getMatrix(a, Xc));

	return result;
}


string CoLoc::ToString()
{
	stringstream sb ;
	sb << "discount : " << discount << endl;
	sb << "initialBelief : " << endl;
	initialBelief->write(sb) << endl;
	sb << "initialBeliefStval : " << endl;
	sb << "initialBeliefStval stval: " <<  initialBeliefStval->sval << endl;
	initialBeliefStval->bvec->write(sb) << endl;
	sb << "initialBeliefX : " << endl;
	initialBeliefX->write(sb) << endl;
	sb << "Num X States : " << XStates->size() << endl;
	sb << "Num Y States : " << YStates->size() << endl;
	sb << "Num Action : " << actions->size() << endl;
	sb << "Num Observations : " << observations->size() << endl;
	sb << "X Trans : " << XTrans->ToString() << endl;
	sb << "XY Trans : " << XYTrans->ToString() << endl;
	sb << "Obs Prob : " << obsProb->ToString() << endl;
	sb << "Rewards : " << rewards->ToString() << endl;

	return sb.str();
}

void CoLoc::getObsProbVectorFast(obs_prob_vector& result, int a, int Xn, SparseVector& tmp1)
{
	mult( result, tmp1, *obsProb->getMatrix(a, Xn) );
	// this should give the same result
	// mult( result, Otr[a][Xn], tmp1 );

	result *= (1.0/(result.norm_1()));

}

int  CoLoc::getNumActions()
{
	return actions->size();
}
int  CoLoc::getBeliefSize()
{
	return YStates->size();
}

SparseVector&  CoLoc::getJointUnobsStateProbVector(SparseVector& result, SharedPointer<BeliefWithState> b,	int a, int Xn) 
{
	int Xc = b->sval; // currrent value for observed state variable
	// belief_vector Bc = b.bvec; 	// current belief for unobserved state variable
	DenseVector tmp, tmp1;
	DenseVector Bc; // = b.bvec;

	copy(Bc, *(b->bvec));

	if (this->XStates->size() == 1)
	{
		tmp = Bc;
	}
	else
	{
		emult_column( tmp, *this->XTrans->getMatrix(a, Xc), Xn, Bc );
	}

	mult( tmp1, *this->XYTrans->getMatrixTr(a, Xc), tmp );
	
	copy(result, tmp1);
	return result;
}

void CoLoc::getObsProbVector(obs_prob_vector& result, const BeliefWithState& b, int a, int Xn) 
{
	int Xc = b.sval; // currrent value for observed state variable
	// belief_vector Bc = b.bvec; 	// current belief for unobserved state variable
	DenseVector tmp, tmp1, tmp2;
	DenseVector Bc; // = b.bvec;

	copy(Bc, *b.bvec);

	//cout << "a :" << a << " Xc :" << Xc << "Xn :" << Xn << endl;
	// --- overall: result = O_a_xn' * (TY_a_xc' * (TX_a_xc (:,xn) .* bc))
	// tmp = TX_a_xc (:,xn) .* bc
	emult_column( tmp, *XTrans->getMatrix(a, Xc), (int) Xn, Bc );
	// tmp1 = TY_a_xc' * tmp
	mult( tmp1, *XYTrans->getMatrixTr(a, Xc), tmp );
	// result = O_a_xn' * tmp1
	mult( tmp2, tmp1, *obsProb->getMatrix(a, Xn) );

	copy(result, tmp2);
	result *= (1.0/result.norm_1());
}


map<string, string> CoLoc::getActionsSymbols(int actionNum) 
{
	map<string, string> result;

	return result;
}


map<string, string> CoLoc::getFactoredObservedStatesSymbols(int stateNum) 
{
	map<string, string> result;

	return result;
}


map<string, string> CoLoc::getFactoredUnobservedStatesSymbols(int stateNum) 
{
	map<string, string> result;

	return result;
}

map<string, string> CoLoc::getObservationsSymbols(int observationNum) 
{
	map<string, string> result;

	return result;
}

