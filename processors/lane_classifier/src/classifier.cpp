
#include <iostream>
#include <vector>
#include <map>
#include "math.h"
#include "classifier.h"
#include "instance.h"

using namespace std;
/*
#undef PROFILER_START_FUN
#undef PROFILER_STOP_FUN
#undef PROFILER_START
#undef PROFILER_STOP

#define PROFILER_START_FUN(p)
#define PROFILER_STOP_FUN(p)
#define PROFILER_START(p,e)
#define PROFILER_STOP(p,e)
*/
#include "profiler.h"
//binary search to find a match, or find the point where a match would be
// (where min crosses over max)
int Classifier::findClosestIndex(const InstAndScore& inst) const {

    //first, find the closest
    PROFILER_START_FUN(profiler);
    int mid = -1;
    int min = 0;
    int max = instances.size() - 1;
    while(max >= min) {
        mid = min + (max - min) / 2;
        //double dist = distance(inst, instances[mid]);
        double dist = inst.score - instances[mid].score; //evaluation.score(instances[mid]);
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

std::vector<InstAndScore *> Classifier::findKNearest(const InstAndScore& inst) {
//    cout << "DONE: " << min << ", " << mid << ", " << max << ", " << instances.size() << endl;
    int mid = findClosestIndex(inst);
    PROFILER_START_FUN(profiler);
    std::vector<InstAndScore *> kNearest;
//    InstAndScore **kn = new InstAndScore*[this->k];
    InstAndScore *kn[32];
    int kninx = 0;
    if (mid >= 0) {

        //NOTE: this range will be 1 or 2 bigger than k
        int start = mid - k;
        int end = mid + k;

        double endScore = instances[end].score - inst.score;
        double startScore = inst.score - instances[start].score;
        while (end - start > k) {
            
            if (startScore < endScore) {
                end--;
                endScore = instances[end].score - inst.score;
            } else {
                start++;
                startScore = inst.score - instances[start].score;
            }
        }
#if 0
        int maxCount = 0;
//        int *histogram = new int[this->maxLabel + 1];
        int histogram[5] = {0};
//        memset(histogram, 0, (this->maxLabel + 1) * sizeof(int));
        for (int ii = start; ii <= end; ii++) {

            Instance *inst = &instances[ii].instance;
            int count = ++histogram[inst->getLabel()];
            inst->markUsed();
            if (count > maxCount) {
//                label = inst->getLabel();
                maxCount = count;
            }
        }
#endif
//    delete [] histogram;
#if 0
        int size = instances.size();
//        kNearest.reserve(this->k);
        double instEval = 0; //evaluation.score(inst);
//        double two_e = instEval * 2;
//        kNearest.push_back(&instances[mid]);
//
        kn[kninx++] = &instances[mid];
        for (int ii = mid - 1, jj = mid + 1;
             (ii >= 0 || jj < size) && /*kNearest.size()*/ kninx < k;
             ) {
            if (ii < 0 && jj < size) {
                kn[kninx++] = &instances[jj];
                //kNearest.push_back(&instances[jj]);
                jj++;
            } else if (ii >= 0 && jj >= size) {
                kn[kninx++] = &instances[ii];
//                kNearest.push_back(&instances[ii]);
                ii--;
            } else {
    //            cout << ii << "," << jj << "," << size << endl;
                double iiEval   = instances[ii].score; //0;// evaluation.score(instances[ii]);
                double jjEval   = instances[jj].score;// evaluation.score(instances[jj]);
                
                double distii = instEval - iiEval;
                double distjj = jjEval - instEval;
                if (distii < distjj) {
                    kn[kninx++] = &instances[ii];
//                    kNearest.push_back(&instances[ii]);
                    ii--;
                } else {
                    kn[kninx++] = &instances[jj];
                    //kNearest.push_back(&instances[jj]);
                    jj++;
                }
            }
        }
#endif
    }
#if 0
    const Instance **kNearest = new const Instance*[k];
    int inx = 0;
    if (mid >= 0) {
        kNearest[inx++] = &instances[mid];
        int size = instances.size();
        for (int ii = mid - 1, jj = mid + 1; ii >= 0 && jj < size && inx < k; ) {
            double distii = 0; //fabs(distance2(inst, instances[ii], maxA, maxB));
            double distjj = 0; //fabs(distance2(inst, instances[jj], maxA, maxB));
            if (distii < distjj) {
                kNearest[inx++] = &instances[ii];
                ii--;
            } else {
                kNearest[inx++] = &instances[jj];
                jj++;
            }
        }
    }
//    vec.assign(kNearest, kNearest + k);
#endif
    PROFILER_STOP_FUN(profiler);
    return kNearest;
}


Classifier::Classifier(Profiler *profiler, int k, const Evaluation& evaluation) : profiler(profiler), k(k), evaluation(evaluation) {
    
}


int sortInstances(const InstAndScore& a, const InstAndScore& b) {
    return a.score - b.score < 0;
}
void Classifier::addInstances(const std::vector<Instance>& instances) {
#if 0
    this->instances.insert(this->instances.end(), instances.begin(), instances.end());
    std::sort(this->instances.begin(), this->instances.end(), evaluation);
    this->maxLabel = -1;
    for (std::vector<Instance>::const_iterator it = this->instances.begin();
         it != this->instances.end();
         it++) {
        if (it->getLabel() > maxLabel) {
            maxLabel = it->getLabel();
        }
    }
#endif
    std::set<int> scores;

    for (std::vector<Instance>::const_iterator it = instances.begin();
         it !=instances.end();
         it++) {
        if (it->getLabel() > maxLabel) {
            maxLabel = it->getLabel();
        }
        InstAndScore ias = {*it, evaluation.score(*it)};
    //    if (scores.find(ias.score) == scores.end()) {
            this->instances.push_back(ias);
        //    scores.insert(ias.score);
      //  }
//    this->maxLabel = -1;
    }
    std::sort(this->instances.begin(), this->instances.end(), sortInstances);

    for (std::vector<InstAndScore>::const_iterator it = this->instances.begin();
         it != this->instances.end();
         it++) {
        f[it->score] = doClassify(*it);
    }
//#ifdef DEBUG
    cout << this->instances.size() << " instances" << endl;
//#endif
}

std::vector<InstAndScore> makeInstAndScore(const std::vector<Instance>& instances) {
    std::vector<InstAndScore> out;
    for (std::vector<Instance>::const_iterator it = instances.begin();
         it !=instances.end();
         it++) {
        
        InstAndScore ias = {*it, 0}; //evaluation.score(*it)};
        out.push_back(ias);
    }
    return out;
}

int findClosestIndexG(const InstAndScore& inst, const std::vector<InstAndScore>& vec) {

    //first, find the closest
    //PROFILER_START_FUN(profiler);
    int mid = -1;
    int min = 0;
    int max = vec.size() - 1;
    double instEval = inst.score; //evaluation.score(inst);
    while(max >= min) {
        mid = min + (max - min) / 2;
        //double dist = distance(inst, instances[mid]);
        double dist = instEval - vec[mid].score; //evaluation.score(instances[mid]);
        if (dist == 0) {
            break;
        } else if (dist > 0) {
            min = mid + 1;
        } else {
            max = mid - 1;
        }
    }
    //PROFILER_STOP_FUN(profiler);
    return mid;
}
std::vector<int> Classifier::classifyAll(const std::vector<Instance>& instances) {

    PROFILER_START_FUN(profiler);
//    std::vector<InstAndScore> incoming = makeInstAndScore(instances);
    std::vector<int> labels;
    std::set<int> scores;

    int ii = 0;
    for (std::vector<Instance>::const_iterator it = instances.begin();
         it != instances.end();
         it++) {
        int score = ii++; //evaluation.score(*it);
        if (scores.find(score) == scores.end()) {
            int nearest = findClosestIndex(*it);
            scores.insert(score);
        }
         
#if 0        
        std::vector<InstAndScore *> kNearest = findKNearest(*it, incominginstance);
        PROFILER_START(profiler, mostFrequentLabel);
        int label = -1;
        int maxCount = 0;
        int size = kNearest.size();
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
        labels.push_back(label);
        delete [] histogram;
        PROFILER_STOP(profiler, mostFrequentLabel);
#endif
    }

    PROFILER_STOP_FUN(profiler);
    return labels;
}
int Classifier::classify(const Instance& instance) {

    PROFILE_FUN(profiler);
    InstAndScore ias = {instance, evaluation.score(instance)};
    if (f.find(ias.score) == f.end()) {
        f[ias.score] = doClassify(ias);
    }

    return f[ias.score];
}

int Classifier::doClassify(const InstAndScore& ias) {
    
    std::vector<InstAndScore *> kNearest = findKNearest(ias.instance);
    PROFILER_START(profiler, mostFrequentLabel);
//    std::map<int, int> histogram;
    int label = -1;
    int maxCount = 0;
    int size = kNearest.size();
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


int Classifier::pruneInstances(int thresh) {
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
