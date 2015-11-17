#include "Obs2DBeaconNoisy.h"
#include "InRangeAllNto1VarRel.h"


Obs2DBeaconNoisy::Obs2DBeaconNoisy(SharedPointer<Variable> obsVar, SharedPointer<Map2DPosVar> suv1PosVar, SharedPointer<Map2DPosVar> suv2PosVar, SharedPointer<Map2DPosVar> auvPosVar, SharedPointer<BooleanVariable> auvGetsGPSPos, SharedPointer<ProblemParams> params )
{
   this->suv1PosVar = suv1PosVar;
    this->suv2PosVar = suv2PosVar;
    this->auvPosVar = auvPosVar;
	this->params = params;
	this->auvGetsGPSPosVar = auvGetsGPSPos;
	this->obsVar = obsVar;

    isCal = SharedPointer<BooleanVariable> ( new BooleanVariable("isCal", 1.0, 0.0) );

    auvPosEst = SharedPointer<InRangeAllNto1VarRel> (new InRangeAllNto1VarRel(isCal, auvPosVar, params->ModemRange));
    auvPosEst->addSrcVar(suv1PosVar);
    auvPosEst->addSrcVar(suv2PosVar);

    this->addSourceVar(suv1PosVar);
    this->addSourceVar(suv2PosVar);
    this->addSourceVar(auvPosVar);
    this->addSourceVar(auvGetsGPSPos);
    this->setDestVariable(obsVar);
}

Obs2DBeaconNoisy::~Obs2DBeaconNoisy(void)
{
}

vector<SharedPointer<RelEntry> > Obs2DBeaconNoisy::getProb(map<string, SharedPointer<IVariableValue> > sourceVals)
{
	vector<SharedPointer<RelEntry> > input = auvPosEst->getProb(sourceVals);
    vector<SharedPointer<RelEntry> > result;

        FOREACH_NOCONST(SharedPointer<RelEntry>, psrcEntry , input)
        {
			SharedPointer<RelEntry>& srcEntry = *psrcEntry;
			/*SharedPointer<IVariableValue> auvPos = srcEntry->sourceValues[auvPosVar->getVariableName()];
			SharedPointer<IVariableValue> suv1Pos = srcEntry->sourceValues[suv1PosVar->getVariableName()];
            SharedPointer<IVariableValue> suv2Pos = srcEntry->sourceValues[suv2PosVar->getVariableName()];*/

			SharedPointer<IVariableValue> auvPos = sourceVals[auvPosVar->getVariableName()];
			SharedPointer<IVariableValue> suv1Pos = sourceVals[suv1PosVar->getVariableName()];
            SharedPointer<IVariableValue> suv2Pos = sourceVals[suv2PosVar->getVariableName()];

			SharedPointer<IVariableValue> inRange = srcEntry->destValues[isCal->getVariableName()];


			if(inRange->getIndex() == BooleanVariable::TrueValueIndex)
            {
                // if in range
				if(sourceVals[auvGetsGPSPosVar->getVariableName()]->getIndex() == BooleanVariable::TrueValueIndex)
				{
					{
						SharedPointer<RelEntry> newEntry ( new RelEntry() );
						SharedPointer<IVariableValue> destValue = obsVar->getValueByName(auvPos->ToString()+"true");
						newEntry->destValues[destValue->getVariableName()] = destValue;
						newEntry->prob = params->AUVtoSUVProb;
						result.push_back(newEntry);
					}

					{
						SharedPointer<RelEntry> newEntry ( new RelEntry() );
						SharedPointer<IVariableValue> destValue = obsVar->getValueByName("na");
						newEntry->destValues[destValue->getVariableName()] = destValue;
						newEntry->prob = 1 - params->AUVtoSUVProb;
						result.push_back(newEntry);
					}
				}
				else
				{
					{
						SharedPointer<RelEntry> newEntry ( new RelEntry() );
						SharedPointer<IVariableValue> destValue  = obsVar->getValueByName("na");
						newEntry->destValues[destValue->getVariableName()] =destValue;
						newEntry->prob =  1 - params->NoisyAUVObsProb;
						result.push_back(newEntry);
					}

					vector<SharedPointer<Map2DPosValue> >  adjCells =  auvPosVar->getAdjPos( dynamic_pointer_cast<Map2DPosValue>(auvPos), params->NoisyAUVObsRadius);

					FOREACH_NOCONST(SharedPointer<Map2DPosValue> , padjCell, adjCells)
					{
						SharedPointer<Map2DPosValue>& adjCell = *padjCell;
						SharedPointer<RelEntry> newEntry ( new RelEntry() );
						SharedPointer<IVariableValue> destValue = obsVar->getValueByName(adjCell->ToString() + "false");
						newEntry->destValues[destValue->getVariableName()] =destValue;
						newEntry->prob =  params->NoisyAUVObsProb / adjCells.size();
						result.push_back(newEntry);
					}
				}
            }
            else
            {
                // if not in range, then obs is useless
				SharedPointer<RelEntry> newEntry ( new RelEntry() );
				SharedPointer<IVariableValue> destValue = obsVar->getValueByName("na");
				newEntry->destValues[destValue->getVariableName()] = destValue;
                newEntry->prob = 1.0;
				result.push_back(newEntry);

            }

        }
        return result;
	
}
