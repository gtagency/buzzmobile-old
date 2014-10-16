
#include <iostream>
#include "instance.h"
#include <cassert>

Instance::Instance(int n, double *f, int l) : label(l), uses(0) {
    this->features = new double[n];
  //std::cout << "Creating"  << (long)this->features << std::endl;
    for (int ii = 0; ii < n; ii++) {
      this->features[ii] = f[ii];
    }
}
/*
Instance::Instance(Instance& copy) : label(copy.label), uses(copy.uses) {
    this->features = new double[2];
  std::cout << "Copying"  << "," << (long)this->features << std::endl;
    for (int ii = 0; ii < 2; ii++) {
      this->features[ii] = copy.features[ii];
    }
}*/
Instance::Instance(const Instance& copy) : label(copy.label), uses(copy.uses) {
    this->features = new double[2];
  //std::cout << "Copying Const"  << "," << (long)this->features << std::endl;
    for (int ii = 0; ii < 2; ii++) {
      this->features[ii] = copy.features[ii];
    }
}

Instance& Instance::operator=( Instance rhs ) {
//  std::cout << "Assigning"  << "," << (long)this->features << "," << (long)rhs.features << std::endl;
//  std::swap (*this, rhs); // Non-throwing swap
  
  this->label = rhs.label;
  this->uses = rhs.uses;
  this->timeToLive = rhs.timeToLive;
  double *ftemp = this->features;
  this->features = rhs.features;
  rhs.features = ftemp;
  return *this;
}

Instance::~Instance() {
 // std::cout << "Destroying"  << (long)this->features << std::endl;
  delete [] this->features;
  this->features = NULL;
}
double Instance::getFeature(unsigned int feature) const {
//    assert(feature < this->features.size());
//  std::cout << feature << ", " << (long)this->features << std::endl;
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
