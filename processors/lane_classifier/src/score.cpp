
#include "score.h"
#include "math.h"

namespace score {
	double dist(double d1, double d2) {
	    return sqrt(d1*d1 + d2*d2);
	}

	double scoreHueAndThreshold(const Instance& instance, void *data) {
	    return instance.getFeature(0) + (instance.getFeature(2) < 50 ? 0 : *(int *)data);
	}

	double scoreHueAndSat(const Instance& instance, void *data) {
	    int *data2 = (int *)data;
	    return data2[0] * instance.getFeature(0) + instance.getFeature(1);
	}

	double scoreSum(const Instance& instance, void *data) {

	    //cout << instance.getFeature(0) << " " << instance.getFeature(1) << " " << instance.getFeature(2) << endl;
	    return instance.getFeature(0) + instance.getFeature(1) + instance.getFeature(2);
	}

	double score3(const Instance& instance, void *data) {
	    int *data2 = (int *)data;
	    //cout << "in score3: " << data2[0] << ", " << data2[1] << endl;
    
	    double score = data2[2] * instance.getFeature(2) +
	                   data2[0] * instance.getFeature(0) +
	                              instance.getFeature(1);
	    //cout << score << endl;
	    return score;
	}

	double scoreW(const Instance& instance, void *data) {
	    int *data2 = (int *)data;
	    double sum = data2[0] + data2[1] + data2[2];
	    double w0 = data2[0] / sum;
	    double w1 = data2[1] / sum;
	    double w2 = data2[2] / sum;
	    //cout << "in score3: " << data2[0] << ", " << data2[1] << endl;
    
	    double score = //w0 * instance.getFeature(0) +
	                   w1 * instance.getFeature(1) +
	                   w2 * instance.getFeature(2);
	    //cout << score << endl;
	    return score;
	}
}