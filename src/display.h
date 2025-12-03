#ifndef DISPLAY_H
#define DISPLAY_H
#include "sim.h"
#include <stdbool.h>
bool display_enable();
bool display_disable();
bool display_render(struct pendulum_system *system, const char *info);
#endif
