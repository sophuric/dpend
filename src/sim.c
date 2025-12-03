#include "sim.h"

static unsigned log10i(size_t x) {
	unsigned i;
	for (i = 1; x >= 10; x /= 10) ++i;
	return i;
}

#define ASSERT(x) \
	if ((res = (x))) goto fail;

bool pend_init(struct pendulum_system *system) {
	bool ret = false;
	CWRAPPER_OUTPUT_TYPE res = 0;

	// initialise temp variables
	basic temp, vx, vy, vlx, vly, half, one;
	basic_new_stack(temp);
	basic_new_stack(vx);
	basic_new_stack(vy);
	basic_new_stack(vlx);
	basic_new_stack(vly);
	basic_new_stack(half);
	basic_new_stack(one);

	// initialise system symbols
	basic_new_stack(system->sym_gravity);
	basic_new_stack(system->ke);
	basic_new_stack(system->gpe);
	basic_new_stack(system->lagrangian);

	ASSERT(symbol_set(system->sym_gravity, "g"));
	basic_const_zero(system->ke);
	basic_const_zero(system->gpe);
	basic_const_zero(vx);
	basic_const_zero(vy);
	ASSERT(rational_set_ui(half, 1, 2));
	basic_const_one(one);

	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];

		// initialise symbols for pendulum
		basic_new_stack(p->sym_mass);
		basic_new_stack(p->sym_length);
		basic_new_stack(p->sym_angle);
		basic_new_stack(p->sym_angular_velocity);

		// create unique name for the variable
		unsigned str_size = 16 + log10i(i);
		char str[str_size];
		int printf_res = snprintf(str, str_size, ".%u", i);
		if (printf_res < 0 || printf_res >= str_size) goto fail;

		str[0] = 'm';
		ASSERT(symbol_set(p->sym_mass, str));
		str[0] = 'l';
		ASSERT(symbol_set(p->sym_length, str));
		str[0] = 'x';
		ASSERT(symbol_set(p->sym_angle, str));
		str[0] = 'v';
		ASSERT(symbol_set(p->sym_angular_velocity, str));

		// define the kinetic energy

		ASSERT(basic_sin(vlx, p->sym_angle));
		ASSERT(basic_cos(vly, p->sym_angle));

		ASSERT(basic_mul(temp, p->sym_length, p->sym_angular_velocity)); // v = rω
		ASSERT(basic_mul(vlx, vlx, temp));
		ASSERT(basic_mul(vly, vly, temp)); // add velocity vector
		ASSERT(basic_sub(vx, vx, vlx));
		ASSERT(basic_add(vy, vy, vly));

		// compute magnitude^2
		ASSERT(basic_mul(vlx, vx, vx));
		ASSERT(basic_mul(vly, vy, vy));
		ASSERT(basic_add(temp, vlx, vly));

		// multiply by mass
		ASSERT(basic_mul(temp, temp, p->sym_mass));

		// halve
		ASSERT(basic_mul(temp, temp, half));

		// add value, KE=0.5mv^2
		ASSERT(basic_add(system->ke, system->ke, temp));

		// define the gravitational potential energy

		ASSERT(basic_cos(vly, p->sym_angle)); // cos is in the vertical axis, unlike the unit circle
		ASSERT(basic_sub(vly, one, vly));     // flip vertically
		ASSERT(basic_mul(vly, vly, p->sym_length));
		ASSERT(basic_mul(temp, vly, system->sym_gravity)); // multiply by gravity
		ASSERT(basic_mul(temp, temp, p->sym_mass));        // multiply by mass

		// add value, GPE=mgh
		ASSERT(basic_add(system->gpe, system->gpe, temp));
	}

	// define the Lagrangian function
	ASSERT(basic_sub(system->lagrangian, system->ke, system->gpe)); // L = T (kinetic) - V (potential)

	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];

		// initialise symbols for pendulum
		basic_new_stack(p->sym_part_deriv_angle);
		basic_new_stack(p->sym_part_deriv_angular_velocity);

		// partially differentiate Lagrangian function
		ASSERT(basic_diff(p->sym_part_deriv_angle, system->lagrangian, p->sym_angle));
		ASSERT(basic_diff(p->sym_part_deriv_angular_velocity, system->lagrangian, p->sym_angular_velocity));
	}

	ret = true;

fail:
	basic_free_stack(temp);
	basic_free_stack(vx);
	basic_free_stack(vy);
	basic_free_stack(vlx);
	basic_free_stack(vly);
	basic_free_stack(half);
	basic_free_stack(one);

	return ret;
}

bool pend_free(struct pendulum_system *system) {
	basic_free_stack(system->sym_gravity);
	basic_free_stack(system->ke);
	basic_free_stack(system->gpe);
	basic_free_stack(system->lagrangian);
	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];
		basic_free_stack(p->sym_mass);
		basic_free_stack(p->sym_length);
		basic_free_stack(p->sym_angle);
		basic_free_stack(p->sym_angular_velocity);
	}
	return true;
}

bool pend_step(struct pendulum_system *system) {
	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];
		// testing
		p->angle += 0.05 * (i + 1);
	}
	return true;
}
