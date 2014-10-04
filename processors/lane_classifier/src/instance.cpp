
#include "instance.h"
#include <cassert>

Instance::Instance(int n, double *f, int l) : label(l), uses(0) {
//    this->features = new double[n];
//    printf("%lx: %lx\n", (long) this, (long)this->features);
  //  memcpy(this->features, f, n * sizeof(double));
    this->features.reserve(n);
    for (int ii = 0; ii < n; ii++) {
        this->features.push_back(f[ii]);
    }
}

Instance::~Instance() {
}

double Instance::getFeature(unsigned int feature) const {
    assert(feature < this->features.size());
    return this->features[feature];
}

int Instance::clearUses() {
	int last = this->uses;
	this->uses = 0;
	return last;
}

int Instance::markUsed() {
	return ++this->uses;
}
