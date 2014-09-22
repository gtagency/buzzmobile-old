
#include <vector>
#include <queue>
#include <map>
#include "classifier.h"
#include <cassert>

Instance::Instance(int n, double *f, int l) : label(l) {
//    this->features = new double[n];
//    printf("%lx: %lx\n", (long) this, (long)this->features);
  //  memcpy(this->features, f, n * sizeof(double));
    for (int ii = 0; ii < n; ii++) {
        this->features.push_back(f[ii]);
    }
}

Instance::~Instance() {
}

double Instance::getFeature(int feature) const {
    assert(feature >= 0 && feature < this->features.size());
    return this->features[feature];
}

int Instance::getLabel() const {
    return this->label;
}

int KNearestNeighbor::getMostLikely(int k) {
    //work with a copy so we can call this method over and over again
    ClassifyQueue qCopy = ClassifyQueue(q);
    std::map<int, int> histogram;
    int label = -1;
    int maxCount = 0;
    for (int ii = 0; ii < k; ii++) {
        const Instance& inst = qCopy.top();
        qCopy.pop();
        int count = histogram[inst.getLabel()];
        count++;
        histogram[inst.getLabel()] = count;
        if (count > maxCount) {
            label = inst.getLabel();
            maxCount = count;
        }
    }
    return label;
}

Classifier::Classifier(int k, double (*dFn)(const Instance& i1, const Instance& i2)) : k(k), distanceFn(dFn) {
    
}

void Classifier::addInstances(const std::vector<Instance>& instances) {
   this->instances.insert(this->instances.end(), instances.begin(), instances.end()); 
}


int Classifier::classify(const Instance& instance) {
    KNearestNeighbor knn(instances, instance, distanceFn);
    return knn.getMostLikely(this->k);
}
