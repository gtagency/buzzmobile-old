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
public:

    virtual void addInstances(const std::vector<Instance>& instances) = 0;
    virtual std::vector<int> classifyAll(std::vector<Instance>& instance) = 0;
    virtual void classify(Instance& instance) = 0;
    virtual int pruneInstances(int thresh) = 0;

    virtual bool isInitialized() = 0;
};

class KNNClassifier : public Classifier {
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
    KNNClassifier(int k, const Evaluation& evaluation); 

    virtual void addInstances(const std::vector<Instance>& instances);
    virtual std::vector<int> classifyAll(std::vector<Instance>& instance);
    virtual void classify(Instance& instance);
    virtual int pruneInstances(int thresh);

    virtual bool isInitialized();
};


class ClusterBasedClassifier : public Classifier {
private:
  long numUpdates;
  std::map<int, Instance> cluster_centers;

  std::vector<std::vector<Instance> > splitByLabel(const std::vector<Instance>& instances);

  Instance computeAverageFeatures(const std::vector<Instance>& instances);
  Instance average(const Instance& oldInst, const Instance& newInst, long count);

public:
    virtual void addInstances(const std::vector<Instance>& instances);
    virtual std::vector<int> classifyAll(std::vector<Instance>& instances);
    virtual void classify(Instance& instance);
    virtual int pruneInstances(int thresh);
    virtual bool isInitialized();
};
#endif //__CLASSIFIER_H
