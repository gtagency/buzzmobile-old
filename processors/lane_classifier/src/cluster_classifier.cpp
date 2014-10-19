
#include "classifier.h"
#include "score.h"
#include <iostream>

std::vector<std::vector<Instance> > ClusterBasedClassifier::splitByLabel(const std::vector<Instance>& instances) {
  std::map<int,std::vector<Instance> > grouped;

  for (std::vector<Instance>::const_iterator it = instances.begin();
       it != instances.end();
       it ++) {
    if (grouped.find(it->label) == grouped.end()) {
      grouped[it->label] = std::vector<Instance>();
    }

    grouped[it->label].push_back(*it);
  }
  std::vector<std::vector<Instance> > groups;
  for (std::map<int, std::vector<Instance> >::const_iterator it = grouped.begin();
       it != grouped.end();
       it ++) {
    groups.push_back(it->second);
  }
  return groups;
}

Instance ClusterBasedClassifier::computeAverageFeatures(const std::vector<Instance>& instances) {
  double features[2] = {0, 0}; //FIXME: magic number, should come from instances
  int label = 0;
  for (std::vector<Instance>::const_iterator it = instances.begin();
       it != instances.end();
       it ++) {
    features[0] += it->getFeature(0); //features[0];
    features[1] += it->getFeature(1); //features[1];
    label = it->label;
  }
  features[0] = features[0] / instances.size();
  features[1] = features[1] / instances.size();
  return Instance(2, features, label); 
}

Instance ClusterBasedClassifier::average(const Instance& oldInst, const Instance& newInst, long count) {
  double alpha = 1.0/count;
  double features[] = {
    oldInst.getFeature(0) + alpha * (newInst.getFeature(0) - oldInst.getFeature(0)),

    oldInst.getFeature(1) + alpha * (newInst.getFeature(1) - oldInst.getFeature(1)),
  };
  return Instance(2, features, oldInst.label);
}
void ClusterBasedClassifier::addInstances(const std::vector<Instance>& instances) {

  numUpdates++;
  std::vector<std::vector<Instance> > groups = splitByLabel(instances);
  for (std::vector<std::vector<Instance> >::iterator it = groups.begin();
       it != groups.end();
       it++) {
    Instance center = computeAverageFeatures(*it);
    if (cluster_centers.find(center.label) == cluster_centers.end()) {
      cluster_centers.insert(std::pair<int,Instance>(center.label, center));
    } else {
      cluster_centers.at(center.label) = average(cluster_centers.at(center.label), center, numUpdates);
    }
  }
}

std::vector<int> ClusterBasedClassifier::classifyAll(std::vector<Instance>& instances) {
  std::vector<int> labels;
  for (std::vector<Instance>::iterator it = instances.begin();
       it != instances.end();
       it++) {
    classify(*it);
    labels.push_back(it->label);
  }
  return labels;
}

void ClusterBasedClassifier::classify(Instance& instance) {
  Evaluation eval;
  double features[] = {instance.getFeature(0), instance.getFeature(1)};
  eval._data = features;
  eval._score = score::scoreHueAndSat;
  double minScore = 9999999999.0;
  Instance *minI;
//  std::cout << instance.getFeature(0) << ", " << instance.getFeature(1) << std::endl;
  int ii = 0;
  for (std::map<int, Instance>::iterator it = cluster_centers.begin();
       it != cluster_centers.end();
       it ++) {
    double score = score::dist(instance, it->second);
  //std::cout << "(" << ii << ")" << it->getFeature(0) << ", " << it->getFeature(1) << " : " << score << std::endl;
    if (score < minScore) {
      minI = &it->second;
      minScore = score;
    }
    ii++;
  }

  instance.label = minI->label;
}

int ClusterBasedClassifier::pruneInstances(int thresh) {
  return 0;
}

bool ClusterBasedClassifier::isInitialized() {
  return cluster_centers.size() != 0;
}
