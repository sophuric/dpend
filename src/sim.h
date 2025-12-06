#ifndef SIM_H
#define SIM_H
#include <symengine/cwrapper.h>
#include <stdbool.h>

struct pendulum {
	double mass, length, angle, angvel;
	basic_struct *sym_mass, *sym_length, *sym_angle, *sym_angvel, *sym_angacc, *solution_angacc,
	        *equation_of_motion,
	        *func_angle, *func_angvel, *func_angacc;
};

struct pendulum_system {
	double gravity;
	basic_struct *sym_gravity,
	        *time, *ke, *gpe, *lagrangian;
	unsigned count;
	struct pendulum *chain;
};

bool sim_substitute(double *out, basic in, struct pendulum_system *system);
bool sim_init(struct pendulum_system *system);
bool sim_step(struct pendulum_system *system, int steps, double time_span);
bool sim_free(struct pendulum_system *system);
#endif
