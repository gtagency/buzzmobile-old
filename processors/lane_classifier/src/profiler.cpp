
#include <stdio.h>
#include <string.h>
#include <iostream>
#include "profiler.h"

using namespace std;

#define ENABLE_PROFILER

#ifdef __APPLE__
#include "mach/mach_time.h"
#define TIME mach_absolute_time
#else
//TODO: replace with clock_gettime
#include <ctime>
#define TIME clock
#endif

Profiler profiler;

Profiler::Profiler() {
    head = NULL;
    tail = NULL;
#if 0   
    int numSimulations = 1000000; 
    clock_t total = 0;
    for (int ii = 0; ii < numSimulations; ii++) {
        clock_t t = clock();
        clock();
        total += clock() - t;
    }
    offset = total/numSimulations;
#endif
    offset = 0;
    pStart = TIME();
}

Profiler::~Profiler() {

}

ProfilerEvent *Profiler::startEvent(const char *name) {
#ifdef ENABLE_PROFILER
    ProfilerEvent *pe = new ProfilerEvent();
    if (tail) {
        tail->next = pe;
    }
    if (!head) {
        head = pe;
    }
    tail = pe;
    eventNames.insert(name);
    pe->end = 0;
    pe->name = name;
    pe->start = TIME();
    return pe;
#else
    return NULL;
#endif
}


void Profiler::stopEvent(ProfilerEvent *pe) {
#ifdef ENABLE_PROFILER
    pe->end = TIME();
    if (pe->end < pe->start) {
        cout << "WHAT:" << pe->end << " " << pe->start << endl;
    }
#endif
}

void Profiler::printResults() {
    int nsPerSec = 1000 * 1000 * 1000;
    double totalRuntime = ((double)(TIME() - pStart))/nsPerSec;
    printf ("%f seconds elapsed.\n", totalRuntime);
#ifdef ENABLE_PROFILER
    double profiledSeconds = 0;
    for (std::set<const char *>::iterator it = eventNames.begin();
         it != eventNames.end();
         it++) {
        int count = 0;
        long totalClocks = 0;
        double seconds;
        ProfilerEvent *curr = head;
        while (curr) {
            if (strcmp(curr->name, *it) == 0) {
                count++;
                totalClocks += (curr->end - curr->start);
            }
            curr = curr->next;
        }
        seconds = ((double)totalClocks)/nsPerSec;
        profiledSeconds += seconds;
        cout << "Event: " << *it << ", Total Elapsed (secs): " << seconds << ", Number of calls: " << count << ", Avg per call: " << seconds / count << endl;
    }
    cout << "Unaccounted time (includes profiler time): " << totalRuntime - profiledSeconds << endl;
#endif
}
