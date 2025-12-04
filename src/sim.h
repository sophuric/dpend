#ifndef SIM_H
#define SIM_H
#include <symengine/cwrapper.h>
#include <stdbool.h>

struct pendulum {
	float mass, length, angle, angular_velocity;
	basic_struct *sym_mass, *sym_length, *sym_angle, *sym_angular_velocity,
	        *sym_part_deriv_angular_velocity, *sym_part_deriv_angle;
};

struct pendulum_system {
	float gravity;
	basic_struct *sym_gravity, *time, *ke, *gpe, *lagrangian;
	unsigned count;
	struct pendulum *chain;
};

bool sim_init(struct pendulum_system *system);
bool sim_step(struct pendulum_system *system);
bool sim_free(struct pendulum_system *system);
#endif
