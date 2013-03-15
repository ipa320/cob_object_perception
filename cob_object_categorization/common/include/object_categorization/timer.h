// from: http://snipplr.com/view/40650/timer-class-for-both-unixlinuxmac-and-windows-system/

//////////////////
// How to Use ////
//////////////////

//#include <iostream>
//#include "timer.h"
//using namespace std;
 
//int main()
//{
//    Timer timer;
// 
//    // start timer
//    timer.start();
// 
//    // do something
//    ...
// 
//    // stop timer
//    timer.stop();
// 
//    // print the elapsed time in millisec
//    cout << timer.getElapsedTimeInMilliSec() << " ms.\n";
// 
//    return 0;
//}
 
 
//////////////////////////////////////////////////////////////////////////////
// Timer.h
// =======
// High Resolution Timer.
// This timer is able to measure the elapsed time with 1 micro-second accuracy
// in both Windows, Linux and Unix system 
//
//  AUTHOR: Song Ho Ahn (song.ahn@gmail.com)
// CREATED: 2003-01-13
// UPDATED: 2006-01-13
//
// Copyright (c) 2003 Song Ho Ahn
//////////////////////////////////////////////////////////////////////////////
 
#ifndef TIMER_H_DEF
#define TIMER_H_DEF
 
#ifndef __LINUX__   // Windows system specific
#include <windows.h>
#else          // Unix based system specific
#include <sys/time.h>
#endif
 
 
class Timer
{
public:
    Timer();                                    // default constructor
    ~Timer();                                   // default destructor
 
    void   start();                             // start timer
    void   stop();                              // stop the timer
    double getElapsedTime();                    // get elapsed time in second
    double getElapsedTimeInSec();               // get elapsed time in second (same as getElapsedTime)
    double getElapsedTimeInMilliSec();          // get elapsed time in milli-second
    double getElapsedTimeInMicroSec();          // get elapsed time in micro-second
 
 
protected:
 
 
private:
    double startTimeInMicroSec;                 // starting time in micro-second
    double endTimeInMicroSec;                   // ending time in micro-second
    int    stopped;                             // stop flag 
#ifdef WIN32
    LARGE_INTEGER frequency;                    // ticks per second
    LARGE_INTEGER startCount;                   //
    LARGE_INTEGER endCount;                     //
#else
    timeval startCount;                         //
    timeval endCount;                           //
#endif
};
 
#endif // TIMER_H_DEF