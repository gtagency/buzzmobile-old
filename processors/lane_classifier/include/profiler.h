#ifndef __PROFILER_H
#define __PROFILER_H

#include <ctime>
#include <set>
#include <vector>

#define PROFILE_FUN(p) ProfileScope(p, __FUNCTION__) 
#define PROFILER_START(p,e)   ProfilerEvent *e##__pe = p->startEvent(#e)
#define PROFILER_START_FUN(p) ProfilerEvent *__FUNCTION__##__pe = p->startEvent(__FUNCTION__)

#define PROFILER_STOP(p,e)   p->stopEvent(e##__pe)
#define PROFILER_STOP_FUN(p) p->stopEvent(__FUNCTION__##__pe)

struct ProfilerEvent {
    const char *name;
    uint64_t start;
    uint64_t end;
    ProfilerEvent *next;
};

class Profiler {
private:
    ProfilerEvent *head;
    ProfilerEvent *tail;
//    std::vector<ProfilerEvent *> events;
    std::set<const char *> eventNames;
    clock_t pStart;
    clock_t offset;
public:
    Profiler();
    virtual ~Profiler();

    ProfilerEvent *startEvent(const char *event);
    void stopEvent(ProfilerEvent *pe);

    void printResults();
};

class ProfileScope {

    Profiler *p;
    ProfilerEvent *pe;
public:
    ProfileScope(Profiler *p, const char *name) {
        this->p = p;
        this->pe = p->startEvent(name);
    }

    ~ProfileScope() {
        p->stopEvent(pe);
    }
};
#endif //__PROFILER_H
