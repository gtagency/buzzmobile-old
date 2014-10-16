#ifndef __CLASSIFIER_H
#define __CLASSIFIER_H

#include <vector>
#include <queue>
#include <map>
#include "instance.h"
#include "profiler.h"

/*
 * A fast, online implementation of k-Nearest Neighbor that works if you can
 * produce an absolute distance for an instance (e.g. an evaluation function)
 */

struct Evaluation {
public:
    void *_data;
    double (*_score)(const Instance& instance, void *data);

    double score(const Instance& instance) const {
        return _score(instance, _data);
    }

    bool operator() (const Instance& i1, const Instance& i2) const {
        return _score(i1, _data) < _score(i2, _data);
    }
};

struct InstAndScore {
public:
    Instance instance;
    double score;
};

class Classifier {
private:
    int k;
    const Evaluation& evaluation;
    bool initialized;
    
    std::vector<InstAndScore> instances;
    std::map<int, int> f;
    int maxLabel;

    int findClosestIndex(const double score) const;
    std::vector<InstAndScore *> findKNearest(const double score);
    std::vector<InstAndScore> makeInstAndScore(const std::vector<Instance>& instances);
    int doClassify(const double score);

public:

    Classifier(int k, const Evaluation& evaluation); 

    void addInstances(const std::vector<Instance>& instances);
    std::vector<int> classifyAll(std::vector<Instance>& instance);
    void classify(Instance& instance);
    int pruneInstances(int thresh);

    bool isInitialized();
};
#endif //__CLASSIFIER_H
