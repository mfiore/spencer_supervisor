#ifndef Map2DPosValue_H
#define Map2DPosValue_H


#include <vector>
#include <string>
#include "IVariableValue.h"
using namespace std;
using namespace momdp;

namespace momdp
{
	class Map2DPosValue : public IVariableValue
	{
	private:


		string varName;

	public:
		double prob;
		int x;
		int y;
		int height;

		Map2DPosValue(string varname, int x, int y, int height, double prob = 0.0);
		virtual ~Map2DPosValue(void);
		virtual string ToString();
		virtual double getProb();
		virtual string getVariableName();
		virtual string getValueName();
		virtual int getIndex();
		virtual bool equals(SharedPointer<IVariableValue> obj);

		virtual SharedPointer<Map2DPosValue> duplicate();
		virtual bool isSamePos(SharedPointer<Map2DPosValue> dest);
		virtual double distanceTo(SharedPointer<Map2DPosValue> dest);
	};
}

#endif

