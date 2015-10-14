#ifndef Map2D_H
#define Map2D_H


#include <vector>
#include <string>
#include "MObject.h"
#include "Map2DPosVar.h"

using namespace std;
using namespace momdp;

namespace momdp
{
	class Map2D : public MObject
	{
	private:
		int height;
		int width;
		
	public:
		Map2D(void);
		Map2D(int width, int height);
		virtual ~Map2D(void);
		
		virtual void setBounds(int width, int height);
		virtual SharedPointer<Map2DPosVar> makePosVar(string name);

		
	};

}

#endif

