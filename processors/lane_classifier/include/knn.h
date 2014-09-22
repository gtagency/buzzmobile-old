#ifndef __CLASSIFIER_H
#define __CLASSIFIER_H

#import <vector>
#import <queue>

class Instance {
private:
    int timeToLive;
//    int numFeatures;
    std::vector<double> features;
    int label;
public:
    Instance(int numFeatures, double *features, int label);
  //  Instance(Instance& copy);
    //Instance(const Instance& copy);
    virtual ~Instance();
    double getFeature(int feature) const;
    int getLabel() const;
    
};

struct DistanceComparator {
    const Instance& target;
    double (*distanceFn)(const Instance& i1, const Instance& i2);

    DistanceComparator(const Instance& t, double (*dFn)(const Instance& i1, const Instance& i2)) : target(t), distanceFn(dFn) {}

    bool operator() (const Instance& i1, const Instance& i2) {
        return i1.getFeature(0) > i2.getFeature(0); //distanceFn(i1, target) > distanceFn(i2, target);
    }
};

/*
 * An instance specific k-nearest neighbor algorithm.
 */

class KNearestNeighbor {
private:
    typedef std::priority_queue<Instance, std::vector<Instance>, DistanceComparator> ClassifyQueue;
    ClassifyQueue q;

public:
    KNearestNeighbor(std::vector<Instance> instances, const Instance& instance, double (*dFn)(const Instance& i1, const Instance& i2))
    :  q(instances.begin(), instances.end(), DistanceComparator(instance, dFn)) {}

    int getMostLikely(int k);
};

class Classifier {
private:
    std::vector<Instance> instances;
    double (*distanceFn)(const Instance& i1, const Instance& i2);
    int k;

public:

    Classifier(int k, double (*distanceFn)(const Instance& i1, const Instance& i2));

    void addInstances(const std::vector<Instance>& instances);
    int classify(const Instance& instance);

};
#endif //__CLASSIFIER_H
