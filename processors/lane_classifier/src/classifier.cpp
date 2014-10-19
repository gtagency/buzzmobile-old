#include <stdint.h>
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <cstring>
#include <cassert>
#include "math.h"
#include "classifier.h"
#include "instance.h"
#include "profiler.h"
using namespace std;

KNNClassifier::KNNClassifier(int k, const Evaluation& evaluation) : k(k), evaluation(evaluation), initialized(false), maxLabel(0) {}

//binary search to find a match, or find the point where a match would be
// (where min crosses over max)
/* private */
int KNNClassifier::findClosestIndex(const double score) const {

    //first, find the closest
    PROFILER_START_FUN(profiler);
    int mid = -1;
    int min = 0;
    int max = instances.size() - 1;
    while(max >= min) {
        mid = min + (max - min) / 2;
        //double dist = distance(inst, instances[mid]);
        double dist = score - instances[mid].score; //evaluation.score(instances[mid]);
        if (dist == 0) {
            break;
        } else if (dist > 0) {
            min = mid + 1;
        } else {
            max = mid - 1;
        }
    }
    PROFILER_STOP_FUN(profiler);
    return mid;
}

/* private */
std::vector<InstAndScore *> KNNClassifier::findKNearest(const double score) {
//    cout << "DONE: " << min << ", " << mid << ", " << max << ", " << instances.size() << endl;
    int mid = findClosestIndex(score);
    PROFILER_START_FUN(profiler);
    std::vector<InstAndScore *> kNearest;
//    InstAndScore **kn = new InstAndScore*[this->k];
    //InstAndScore *kn[32];
    //int kninx = 0;
//    std::cout << "Score: " << score << ", Mid: " << mid << std::endl;
    if (mid >= 0 && mid - k >= 0 && mid + k < instances.size()) {

        //NOTE: this range will be 1 or 2 bigger than k
        int start = mid - k;
        int end = mid + k;

        double endScore = instances[end].score - score;
        double startScore = score - instances[start].score;
        while (end - start > k) {
            
            if (startScore < endScore) {
                end--;
                endScore = instances[end].score - score;
            } else {
                start++;
                startScore = score - instances[start].score;
            }
        }
        for (int ii = start; ii <= end; ii++) {
          kNearest.push_back(&instances[ii]);
        }
    }
    PROFILER_STOP_FUN(profiler);
    return kNearest;
}

int sortInstances(const InstAndScore& a, const InstAndScore& b) {
    return a.score - b.score < 0;
}

void KNNClassifier::addInstances(const std::vector<Instance>& instances) {
    std::set<int> scores;
    //TODO: as per Nick's suggestion, perhaps we can learn the function
    // <I,L> (e.g. train a neural network or SVM).  Classifying is then
    // as simple as running that function over the target image.
    for (std::vector<Instance>::const_iterator it = instances.begin();
         it !=instances.end();
         it++) {
        // Assume positive labels, and -1 means unlabeled.
        // We only want labeled instances here.
        assert(it->getLabel() >= 0);
        if (it->getLabel() > maxLabel) {
            maxLabel = it->getLabel();
        }
        //TODO: maybe we only need to store the scores...
        InstAndScore ias = {*it, evaluation.score(*it)};
    //    if (scores.find(ias.score) == scores.end()) {
    //
//        std::cout << ias.score << "," << it->getLabel() << std::endl;    
        this->instances.push_back(ias);
        //    scores.insert(ias.score);
      //  }
//    this->maxLabel = -1;
    }
    std::sort(this->instances.begin(), this->instances.end(), sortInstances);

    for (std::vector<InstAndScore>::const_iterator it = this->instances.begin();
         it != this->instances.end();
         it++) {
        //FIXME: why not just it->instance.label ?
        f[it->score] = doClassify(it->score);
    }
    initialized = true;
//#ifdef DEBUG
    cout << this->instances.size() << " instances" << endl;
//#endif
}

std::vector<InstAndScore> KNNClassifier::makeInstAndScore(const std::vector<Instance>& instances) {
    std::vector<InstAndScore> out;
    for (std::vector<Instance>::const_iterator it = instances.begin();
         it !=instances.end();
         it++) {
        
        InstAndScore ias = {*it, evaluation.score(*it)};
        out.push_back(ias);
    }
    return out;
}

std::vector<int> KNNClassifier::classifyAll(std::vector<Instance>& instances) {

//FIXME: this looks like it was never implemented -- JR, 09/27/14
    PROFILER_START_FUN(profiler);
    std::vector<InstAndScore> incoming = makeInstAndScore(instances);
    std::map<int, int> labels;
    std::set<int> scores;

    for (std::vector<InstAndScore>::const_iterator it = incoming.begin();
         it != incoming.end();
         it++) {
        int score = it->score; //ii++; //evaluation.score(*it);
        if (scores.find(score) == scores.end()) {
            scores.insert(score);
            labels[score] = doClassify(score); 
//            std::cout << score << "," << labels[score] << std::endl;
        }
    } 

    std::vector<int> labelVec;
    for (std::vector<InstAndScore>::iterator it = incoming.begin();
         it != incoming.end();
         it++) {
//         std::cout << it->score << "," <<labels[it->score] << std::endl;
         labelVec.push_back(labels[it->score]);
    } 
    PROFILER_STOP_FUN(profiler);
    return labelVec;
}

void KNNClassifier::classify(Instance& instance) {

    PROFILE_FUN(profiler);
    double score = evaluation.score(instance);
    if (f.find(score) == f.end()) {
        f[score] = doClassify(score);
    }

    instance.label = f[score];
}

/* private */
int KNNClassifier::doClassify(const double score) {
    
    std::vector<InstAndScore *> kNearest = findKNearest(score);
    PROFILER_START(profiler, mostFrequentLabel);
//    std::map<int, int> histogram;
    int label = -1;
    int maxCount = 0;
    int size = kNearest.size();
//    std::cout << "Max label: " << this->maxLabel << std::endl;
//    std::cout << "Score: " << score << ", Size: " << size << std::endl;
    int *histogram = new int[this->maxLabel + 1];
    memset(histogram, 0, (this->maxLabel + 1) * sizeof(int));
    for (int ii = 0; ii < size; ii++) {
        Instance *inst = &kNearest[ii]->instance;
        int count = ++histogram[inst->getLabel()];
        inst->markUsed();
        if (count > maxCount) {
            label = inst->getLabel();
            maxCount = count;
        }
    }
    delete [] histogram;
    PROFILER_STOP(profiler, mostFrequentLabel);
    return label;
}


int KNNClassifier::pruneInstances(int thresh) {
  std::vector<InstAndScore> newVec;
  for (std::vector<InstAndScore>::iterator it = this->instances.begin();
       it != this->instances.end();
       it++) {
     if (it->instance.getUses() >= thresh) {
       newVec.push_back(*it);
     }
     it->instance.clearUses();
  }
  this->instances = newVec;
  return newVec.size();
}

bool KNNClassifier::isInitialized() {
  return initialized;
}
