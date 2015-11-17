#ifndef Map2DPosVar_H
#define Map2DPosVar_H


#include <vector>
#include <string>
#include "MOMDP.h"
#include "IVariable.h"
#include "Map2DPosValue.h"

using namespace std;
using namespace momdp;

namespace momdp
{
	class Map2D;
	class Map2DPosVar : public IVariable
	{
		friend class Map2D;
	private:
		string variableName;
		SharedPointer<Map2DPosValue> initPos;
		
	protected:
		
		Map2DPosVar(string name, int width, int height);
		virtual ~Map2DPosVar(void);
	public:
		int height;
		int width;

		virtual void setInitPos(SharedPointer<Map2DPosValue> pos);

		virtual string getVariableName();
		virtual SharedPointer<IVariableValue> getValueByName(string valName);

		virtual vector<SharedPointer<IVariableValue> > getInitialValues();

		//virtual void addValue(Map2DPosValue value);
		virtual int getNumValues();
		//virtual vector<SharedPointer<Map2DPosValue> > getValues();
		virtual vector<SharedPointer<IVariableValue> > getValues();

		virtual vector<SharedPointer<Map2DPosValue> > getDestSidePosPerpendicularToMovement(SharedPointer<Map2DPosValue>  src, SharedPointer<Map2DPosValue>  dest);

		virtual vector<SharedPointer<Map2DPosValue> > getAdjPos( SharedPointer<Map2DPosValue>  src, double range);
		
	};

}

#endif

