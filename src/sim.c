#include "sim.h"
#include <math.h>

void step(struct pendulum_chain *chain) {
	for (size_t i = 0; i < chain->count; ++i) {
		// testing
		chain->chain[i].angle += 0.05;
	}
	return;
}
