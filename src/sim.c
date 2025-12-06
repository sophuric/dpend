#include "sim.h"
#include "rk4.h"

static unsigned log10i(size_t x) {
	unsigned i;
	for (i = 1; x >= 10; x /= 10) ++i;
	return i;
}

#define ASSERT(x) \
	if ((res = (x))) goto fail
#define HEAP_ALLOC(x) \
	if (!(x = basic_new_heap())) goto fail
#define HEAP_FREE(x) \
	if (x) x = (basic_free_heap(x), NULL)

bool sim_init(struct pendulum_system *system) {
	bool ret = false;
	CWRAPPER_OUTPUT_TYPE res = 0;

	// initialise temp variables
	basic temp, vx, vy, vlx, vly, half, one, t_angvel, t_angle;
	CVecBasic *time_args = NULL,
	          *acc_system = NULL, *acc_solution = NULL, *acc_symbol = NULL;
	CMapBasicBasic *to_func_subs = NULL, *to_sym_subs = NULL;
	basic_new_stack(temp);
	basic_new_stack(vx);
	basic_new_stack(vy);
	basic_new_stack(vlx);
	basic_new_stack(vly);
	basic_new_stack(half);
	basic_new_stack(one);
	basic_new_stack(t_angvel);
	basic_new_stack(t_angle);

	// initialise system symbols
	HEAP_ALLOC(system->sym_gravity);
	HEAP_ALLOC(system->ke);
	HEAP_ALLOC(system->gpe);
	HEAP_ALLOC(system->lagrangian);
	HEAP_ALLOC(system->time);

	ASSERT(symbol_set(system->sym_gravity, "g"));
	ASSERT(symbol_set(system->time, "t"));
	basic_const_zero(system->ke);
	basic_const_zero(system->gpe);
	basic_const_zero(vx);
	basic_const_zero(vy);
	ASSERT(rational_set_ui(half, 1, 2));
	basic_const_one(one);

	time_args = vecbasic_new();
	if (!time_args) goto fail;
	vecbasic_push_back(time_args, system->time);

	acc_system = vecbasic_new();
	if (!acc_system) goto fail;
	acc_solution = vecbasic_new();
	if (!acc_solution) goto fail;
	acc_symbol = vecbasic_new();
	if (!acc_symbol) goto fail;

	to_func_subs = mapbasicbasic_new();
	if (!to_func_subs) goto fail;

	to_sym_subs = mapbasicbasic_new();
	if (!to_sym_subs) goto fail;

	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];

		// initialise symbols for pendulum
		HEAP_ALLOC(p->sym_mass);
		HEAP_ALLOC(p->sym_length);
		HEAP_ALLOC(p->sym_angle);
		HEAP_ALLOC(p->sym_angvel);
		HEAP_ALLOC(p->sym_angacc);
		HEAP_ALLOC(p->func_angle);
		HEAP_ALLOC(p->func_angvel);
		HEAP_ALLOC(p->func_angacc);
		HEAP_ALLOC(p->equation_of_motion);
		HEAP_ALLOC(p->solution_angacc);

		// create unique name for the variable
		unsigned str_size = 16 + log10i(i);
		char str[str_size];
		int printf_res = snprintf(str, str_size, ".%u", i);
		if (printf_res < 0 || printf_res >= str_size) goto fail;

		// define symbols
		str[0] = 'm';
		ASSERT(symbol_set(p->sym_mass, str));
		str[0] = 'l';
		ASSERT(symbol_set(p->sym_length, str));
		str[0] = 'x';
		ASSERT(symbol_set(p->sym_angle, str));
		str[0] = 'v';
		ASSERT(symbol_set(p->sym_angvel, str));
		str[0] = 'a';
		ASSERT(symbol_set(p->sym_angacc, str));

		// define angle and angular velocity functions
		str[0] = 'f';
		ASSERT(function_symbol_set(p->func_angle, str, time_args));
		ASSERT(basic_diff(p->func_angvel, p->func_angle, system->time));
		ASSERT(basic_diff(p->func_angacc, p->func_angvel, system->time));

		// define the kinetic energy

		ASSERT(basic_sin(vlx, p->sym_angle));
		ASSERT(basic_cos(vly, p->sym_angle));

		ASSERT(basic_mul(temp, p->sym_length, p->sym_angvel)); // v = rω
		ASSERT(basic_mul(vlx, vlx, temp));
		ASSERT(basic_mul(vly, vly, temp));

		// add velocity vector
		ASSERT(basic_sub(vx, vx, vlx));
		ASSERT(basic_add(vy, vy, vly));

		// compute magnitude^2
		ASSERT(basic_mul(vlx, vx, vx));
		ASSERT(basic_mul(vly, vy, vy));
		ASSERT(basic_add(temp, vlx, vly));

		// multiply by mass and halve
		ASSERT(basic_mul(temp, temp, p->sym_mass));
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

		mapbasicbasic_insert(to_func_subs, p->sym_angacc, p->func_angacc);
		mapbasicbasic_insert(to_func_subs, p->sym_angvel, p->func_angvel);
		mapbasicbasic_insert(to_func_subs, p->sym_angle, p->func_angle);

		mapbasicbasic_insert(to_sym_subs, p->func_angacc, p->sym_angacc);
		mapbasicbasic_insert(to_sym_subs, p->func_angvel, p->sym_angvel);
		mapbasicbasic_insert(to_sym_subs, p->func_angle, p->sym_angle);
	}

	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];

		// https://en.wikipedia.org/wiki/Lagrangian_mechanics#Equations_of_motion

		// partially differentiate Lagrangian function
		// these are taken separately for each axis
		ASSERT(basic_diff(t_angle, system->lagrangian, p->sym_angle));
		// note that angular velocity is treated as a separate variable to angle when finding this partial derivative,
		// instead of as the derivative of the angle w.r.t. time
		// see https://math.stackexchange.com/a/2085001
		ASSERT(basic_diff(t_angvel, system->lagrangian, p->sym_angvel));

		// implement Lagrange's equations

		// convert into a function of time, so SymEngine doesn't think angle and angular velocity are constants and differentiates them to zero
		basic_subs(t_angvel, t_angvel, to_func_subs);

		// differentiate t_angvel w.r.t. time
		ASSERT(basic_diff(t_angvel, t_angvel, system->time));

		// substitute symbols back in
		basic_subs(t_angvel, t_angvel, to_sym_subs);

		// final equation
		ASSERT(basic_sub(p->equation_of_motion, t_angvel, t_angle));

		// add equation in system of equations to solve for angular acceleration
		vecbasic_push_back(acc_system, p->equation_of_motion);
		vecbasic_push_back(acc_symbol, p->sym_angacc);
	}

	// solve system of equations for angular acceleration
	ASSERT(vecbasic_linsolve(acc_solution, acc_system, acc_symbol));

	// assign solutions for each angular acceleration
	for (unsigned i = 0; i < system->count; ++i) {
		ASSERT(vecbasic_get(acc_solution, i, system->chain[i].solution_angacc));
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
	basic_free_stack(t_angvel);
	basic_free_stack(t_angle);

	vecbasic_free(time_args);
	vecbasic_free(acc_system);
	vecbasic_free(acc_solution);

	mapbasicbasic_free(to_func_subs);
	mapbasicbasic_free(to_sym_subs);

	if (!ret) {
		if (res) fprintf(stderr, "SymEngine exception %d\n", res);
		sim_free(system);
	}

	return ret;
}

bool sim_free(struct pendulum_system *system) {
	HEAP_FREE(system->sym_gravity);
	HEAP_FREE(system->ke);
	HEAP_FREE(system->gpe);
	HEAP_FREE(system->lagrangian);
	HEAP_FREE(system->time);
	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];
		HEAP_FREE(p->sym_mass);
		HEAP_FREE(p->sym_length);
		HEAP_FREE(p->sym_angle);
		HEAP_FREE(p->sym_angvel);
		HEAP_FREE(p->sym_angacc);
		HEAP_FREE(p->func_angle);
		HEAP_FREE(p->func_angvel);
		HEAP_FREE(p->func_angacc);
		HEAP_FREE(p->equation_of_motion);
		HEAP_FREE(p->solution_angacc);
	}
	return true;
}

static CWRAPPER_OUTPUT_TYPE basic_substitute(basic out, basic in, struct pendulum_system *system) {
	CWRAPPER_OUTPUT_TYPE res = 0;

	basic sym_number;
	basic_new_stack(sym_number);

	ASSERT(basic_assign(out, in));

#define SUBS(symbol, number)                       \
	ASSERT(real_double_set_d(sym_number, number)); \
	ASSERT(basic_subs2(out, out, symbol, sym_number));

	// substitute in real numbers
	SUBS(system->sym_gravity, system->gravity);
	for (int j = 0; j < system->count; ++j) {
		struct pendulum *pend = &system->chain[j];
		SUBS(pend->sym_mass, pend->mass);
		SUBS(pend->sym_length, pend->length);
		SUBS(pend->sym_angle, pend->angle);
		SUBS(pend->sym_angvel, pend->angvel);
	}

#undef SUBS

fail:
	basic_free_stack(sym_number);
	return res;
}

bool sim_substitute(double *out, basic in, struct pendulum_system *system) {
	basic sym_out;
	basic_new_stack(sym_out);
	if (basic_substitute(sym_out, in, system)) {
		basic_free_stack(sym_out);
		return false;
	}

	*out = real_double_get_d(sym_out);
	basic_free_stack(sym_out);
	return true;
}

static const int var_per_pendulum = 2;

static struct pendulum_system *dydt_system;
static bool dydt_success;

static void dydt(double t, double y[], double out[]) {
	struct pendulum_system *system = dydt_system;

	// set all zeros beforehand as failsafe
	for (int i = 0; i < system->count; ++i) out[i * var_per_pendulum] = 0, out[i * var_per_pendulum + 1] = 0;

	for (int i = 0; i < system->count; ++i) {
		struct pendulum *pend = &system->chain[i];

		for (int j = 0; j < system->count; ++j) {
			struct pendulum *pend2 = &system->chain[j];
			// using values from the solver
			pend2->angle = y[j * var_per_pendulum];
			pend2->angvel = y[j * var_per_pendulum + 1];
		}

		double angacc;
		if (!sim_substitute(&angacc, pend->solution_angacc, system)) goto fail;

		out[i * var_per_pendulum] = y[i * var_per_pendulum + 1]; // angle changes by angular velocity
		out[i * var_per_pendulum + 1] = angacc;                  // angular velocity changes by angular acceleration
	}

	dydt_success = true;
fail:
}

bool sim_step(struct pendulum_system *system, int steps, double time_span) {
	if (steps < 1) return false;
	if (time_span <= 0) return false;

	dydt_system = system;
	int variables = system->count * var_per_pendulum;
	double tspan[2] = {0, time_span};
	double y[variables * (steps + 1)];
	double t[steps + 1];

	// copy pendulum data into input
	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];
		y[i * var_per_pendulum] = p->angle;
		y[i * var_per_pendulum + 1] = p->angvel;
	}

	// perform Runge-Kutta order 4
	dydt_success = false;
	rk4(dydt, tspan, y, steps, variables, t, y);
	if (!dydt_success) return false;

	// copy output back into pendulum data
	double *final_y = &y[variables * steps];
	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];
		p->angle = final_y[i * var_per_pendulum];
		p->angvel = final_y[i * var_per_pendulum + 1];
	}

	return true;
}
