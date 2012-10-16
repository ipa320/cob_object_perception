#ifndef STOPWATCH_H_
#define STOPWATCH_H_


#include <ctime>
#include <sstream>
#ifdef __LINUX__
	#include <sys/time.h>
#else
	#include <Windows.h>
#endif


// low resolution stop watch
class SimpleStopWatch {
	clock_t startClock;

public:
	// start time measurement at this moment
	void start() { startClock = clock(); };

	// returns elapsed time since last start() in seconds
	double stop() { return ((double)(clock()-startClock))/(double)CLOCKS_PER_SEC; };
};


// high performance stop watch
#ifdef _WIN32
class PrecisionStopWatch {
	LARGE_INTEGER precisionClock;
	LARGE_INTEGER precisionFrequency;

public:
	PrecisionStopWatch() {
		precisionClock.QuadPart = 0;
		if(!QueryPerformanceFrequency(&precisionFrequency)) printf("PrecisionStopWatch: Error: Frequency query failed.\n");
	};

	// start time measurement at this moment
	void precisionStart() {
		if(!QueryPerformanceCounter(&precisionClock)) printf("PrecisionStopWatch: Error: precisionStart query failed.\n");
	};

	// returns elapsed time since last precisionStart() in seconds
	double precisionStop() {
		LARGE_INTEGER precisionClockEnd;
		if(!QueryPerformanceCounter(&precisionClockEnd)) printf("PrecisionStopWatch: Error: precisionStop query failed.\n");
		return (double)(precisionClockEnd.QuadPart - precisionClock.QuadPart) / (double)precisionFrequency.QuadPart;
	};
};
#else
// Linux version of precision host timer. See http://www.informit.com/articles/article.aspx?p=23618&seqNum=8
class PrecisionStopWatch {
	struct timeval precisionClock;

public:
	PrecisionStopWatch() {
		gettimeofday(&precisionClock, NULL);
	};

	void precisionStart() {
		gettimeofday(&precisionClock, NULL);
	};

	double precisionStop() {
		struct timeval precisionClockEnd;
		gettimeofday(&precisionClockEnd, NULL);
		return ((double)precisionClockEnd.tv_sec + 1.0e-6 * (double)precisionClockEnd.tv_usec) - ((double)precisionClock.tv_sec + 1.0e-6 * (double)precisionClock.tv_usec);
	};
};
#endif

#endif //STOPWATCH_H_
