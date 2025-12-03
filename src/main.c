#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <limits.h>
#include <stdbool.h>
#include <time.h>
#include <stdint.h>
#include <inttypes.h>

#define eprintf(...) fprintf(stderr, __VA_ARGS__)

#include "display.h"
#include "sim.h"

static bool running = false;

static bool stop(void) {
	if (!running) return true;
	running = false;
	if (!display_disable()) {
		eprintf("Failed to deinitialise terminal\n");
		return false;
	}
	return true;
}

static bool start(void) {
	if (running) return true;
	running = true;
	if (!display_enable()) {
		eprintf("Failed to initialise terminal\n");
		stop();
		return false;
	}
	return true;
}

static void signal_func(int signal) {
	if (signal == SIGWINCH) {
		return;
	}

	if (signal == SIGCONT) {
		if (!start()) exit(3);
		return;
	}

	bool did_stop = stop();
	eprintf("Caught signal %02i: %s\n", signal, strsignal(signal));
	if (!did_stop) exit(3);
	switch (signal) {
		case SIGTSTP:
		case SIGTTIN:
		case SIGTTOU:
			raise(SIGSTOP);
			if (!start()) exit(3);
			return;
	}
	exit(signal == SIGINT ? 0 : 1);
}

#define SEC 1000000000

typedef uintmax_t nsec_t;

nsec_t get_time(void) {
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC, &tp);
	return (nsec_t) tp.tv_sec * SEC + tp.tv_nsec;
}

bool nsleep(nsec_t time) {
	struct timespec tp = {.tv_sec = time / SEC, .tv_nsec = time % SEC};
	return !nanosleep(&tp, NULL);
};

int main() {
	struct sigaction sa;
	if (sigemptyset(&sa.sa_mask)) return 2;
	sa.sa_handler = signal_func;
	sa.sa_flags = 0;
	for (int signal = 1; signal < NSIG; ++signal) {
		if (signal == SIGTRAP) continue;
		if (signal == SIGCHLD || signal == SIGURG) continue;  // signals that are ignored by default (excl. SIGCONT, SIGWINCH)
		if (signal == SIGKILL || signal == SIGSTOP) continue; // can't handle these
		sigaction(signal, &sa, NULL);
	}

	struct pendulum_system system = {
	        .count = 2,
	        .gravity = 1,
	        .chain =
	                (struct pendulum[]) {
	                                     {.mass = 1, .length = 1, .angular_velocity = 0, .angle = 0.2},
	                                     {.mass = 1, .length = 1, .angular_velocity = 0, .angle = 1}}
    };

	if (!pend_init(&system)) goto fail;

	if (!start()) return 3;

#define WAIT ((nsec_t) SEC * 0.01)
	nsec_t dest = 0;

	char str[1024] = "";

	for (;;) {
		nsec_t time = get_time();

		if (time > dest) dest = time + WAIT; // if more than one second has elapsed, reset the offset and wait until 1 second has passed since now
		nsec_t delay = dest - time;          // wait until destination time

		if (dest == time + WAIT || nsleep(delay)) {
			time = get_time();
			if (!pend_step(&system)) goto fail;
			nsec_t diff_time = get_time() - time;
			int printf_res = snprintf(str, sizeof(str), "Simulation time: %6" PRIuMAX "ns", diff_time);
			if (printf_res < 0 || printf_res >= sizeof(str)) goto fail;
			dest += WAIT; // add delay amount to destination time so we can precisely run the code on that interval
		}
		if (!display_render(&system, str)) goto fail;
	}

	if (!pend_free(&system)) goto fail;
	return 0;
fail:
	if (!stop()) return 3;
	return 1;
}
