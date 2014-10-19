#ifndef __SCORE_H
#define __SCORE_H

#include "instance.h"

namespace score {
  double dist(const Instance& i1, const Instance& i2);
	double scoreHueAndSat(const Instance& instance, void *data);
}

#endif //__SCORE_H
