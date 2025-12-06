#define MAX_FPS 240
#define SIMULATION_SPEED 1
#define STEPS_PER_FRAME 1
#define FRAME_SKIP true // frame skipping is non-deterministic
#define CONFIGURE(system)                                                   \
	system.gravity = 9.81;                                                  \
	system.count = 2;                                                       \
	system.chain = (struct pendulum[]) {                                    \
	        {.mass = 1.5, .length = 1, .angvel = 0, .angle = M_PI * 2 / 3}, \
	        {.mass = 1,   .length = 1, .angvel = 0, .angle = M_PI / 2    } \
    };
