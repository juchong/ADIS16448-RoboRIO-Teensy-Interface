#include <climits>
#include <cstdlib>
#include <math.h>

#ifndef AHRSAccum_h
#define AHRSAccum_h

class AHRSAccum{
	private:
		int last = 0;
		int now = 0;
		int crossovers = 0;
		float angle = 0;
		float accumulated = 0;
		void calcAccum();
	public:
		bool setCurrentAngle(float angle);
		float getCurrentAngle();
		float getAccumulatedAngle();
		void reset();
		
};

#endif 