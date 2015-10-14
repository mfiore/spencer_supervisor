#ifndef Map2DPath_H
#define Map2DPath_H


#include <vector>
#include <string>
#include "MObject.h"
#include "Map2DPosVar.h"

using namespace std;
using namespace momdp;

namespace momdp
{
	class Map2DPath : public MObject
	{
	private:
		vector<SharedPointer<Map2DPosValue> > path;
		
	public:
		Map2DPath(void);
		virtual ~Map2DPath(void);
		
		virtual void addStep(SharedPointer<Map2DPosValue> step);
		virtual vector<SharedPointer<Map2DPosValue> > getPath();

		
	};

}

#endif

