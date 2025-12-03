#ifndef SIM_H
#define SIM_H
#include <symengine/cwrapper.h>
#include <stdbool.h>

struct pendulum {
	float mass, length, angle, angular_velocity;
	basic sym_mass, sym_length, sym_angle, sym_angular_velocity;
	basic sym_part_deriv_angular_velocity;
	basic sym_part_deriv_angle;
};

struct pendulum_system {
	float gravity;
	basic sym_gravity, ke, gpe, lagrangian;
	unsigned count;
	struct pendulum *chain;
};

bool pend_init(struct pendulum_system *system);
bool pend_step(struct pendulum_system *system);
bool pend_free(struct pendulum_system *system);
#endif
