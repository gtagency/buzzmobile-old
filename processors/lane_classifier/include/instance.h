#ifndef __INSTANCE_H
#define __INSTANCE_H

#include <vector>

class Instance {
private:
  int timeToLive;
  double *features;
public:
  int label;
	int uses;
  Instance(int numFeatures, double *features, int label);
//  Instance(Instance& copy);
  Instance(const Instance& copy);
  Instance& operator=( Instance rhs );


  virtual ~Instance();
  double getFeature(unsigned int feature) const;

	int clearUses();
	int markUsed();
	
  int getLabel() const { return this->label; }
	int getUses() const { return this->uses; }
};

#endif //__INSTANCE_H
