#include "display.h"
#include "util.h"

#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <limits.h>
#include <stdlib.h>
#include <inttypes.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#define DISPLAY_FD (STDOUT_FILENO)
#define eprintf(...) \
	if (dprintf(DISPLAY_FD, __VA_ARGS__) < 0) goto fail

struct display_screen {
	union {
		struct poss size;
		struct {
			size_t w, h;
		};
	};
	size_t buf_size;
	char *buf;
};

#define INDEX(pos, screen) ((size_t) (pos.y) * (size_t) (screen.w) + (size_t) (pos.x))
#define INDEX_CHAR(pos, screen) (INDEX(pos, screen) / CHAR_BIT)
#define INDEX_BIT(pos, screen) (1 << (INDEX(pos, screen) % CHAR_BIT)) // returns bit mask
#define SCREEN_BUF_SIZE(screen) (((screen.w * screen.h) + CHAR_BIT - 1) / CHAR_BIT)

#define SET_BIT(char, mask, set) ((set) ? (char) | (mask) : (char) &~(mask))
#define GET_CELL(pos, screen) ((screen.buf[INDEX_CHAR(pos, screen)] & INDEX_BIT(pos, screen)) != 0)
#define SET_CELL(pos, set, screen)                                                    \
	if ((pos).x >= 0 && (pos).y >= 0 && (pos).x < (screen).w && (pos).y < (screen).h) \
	(screen.buf[INDEX_CHAR(pos, screen)] = SET_BIT(screen.buf[INDEX_CHAR(pos, screen)], INDEX_BIT(pos, screen), set))

static struct display_data {
	struct termios old_termios;
	struct poss term_size;
	struct display_screen screen[2]; // double-buffered rendering
	unsigned screen_index;
} display;

#define SCREEN (display.screen[display.screen_index])
#define SCREEN_OTHER (display.screen[display.screen_index ^ 1])

bool display_enable() {
	if (tcgetattr(DISPLAY_FD, &display.old_termios)) return false;

	struct termios new_termios = display.old_termios;
	cfmakeraw(&new_termios);     // stty raw
	new_termios.c_lflag |= ISIG; // allow ^C = SIGINT, etc.

	if (tcsetattr(DISPLAY_FD, TCSANOW, &new_termios)) return false;

	eprintf("\x1b[?1049h"); // move to separate buffer
	eprintf("\x1b[?7l");    // disable newline at end of line
	eprintf("\x1b[?25l");   // hide cursor

	display.screen[0].buf = NULL;
	display.screen[1].buf = NULL;
	display.screen_index = 0;
	return true;
fail:
	return false;
}

bool display_disable() {
	FREE(display.screen[0].buf);
	FREE(display.screen[1].buf);
	eprintf("\x1b[H");                                                      // move to start
	eprintf("\x1b[2J");                                                     // clear
	eprintf("\x1b[?25h");                                                   // show cursor
	eprintf("\x1b[?7h");                                                    // re-enable newline at end of line
	eprintf("\x1b[?1049l");                                                 // restore buffer
	if (tcsetattr(DISPLAY_FD, TCSANOW, &display.old_termios)) return false; // restore terminal settings
	return true;
fail:
	return false;
}

bool display_render(struct pendulum_system *system, const char *info) {
	bool res = false;

	eprintf("\x1b[H"); // move to start

	struct winsize ioctl_term_size;
	if (ioctl(DISPLAY_FD, TIOCGWINSZ, &ioctl_term_size)) return false; // get terminal size

	struct posf stretch = POSF(2, 1);

	const static struct poss block_size = POSS(2, 2); // adjust code for producing block character if changing this

	display.term_size = POSS(ioctl_term_size.ws_col, ioctl_term_size.ws_row);

	// initialise new screen
	display.screen_index ^= 1;
	SCREEN.size = poss_mul(display.term_size, block_size);
	size_t buf_size = SCREEN_BUF_SIZE(SCREEN);
	bool needs_clear = true;
	// see https://stackoverflow.com/a/39562813
	if (SCREEN.buf == NULL || buf_size > SCREEN.buf_size) {
		// create new buffer if it doesn't exist yet or we are scaling up
		FREE(SCREEN.buf);
		SCREEN.buf = calloc(1, buf_size);
		needs_clear = false;
	} else if (buf_size < SCREEN.buf_size) {
		// realloc if we are scaling down
		SCREEN.buf = realloc(SCREEN.buf, buf_size);
	}
	if (!SCREEN.buf) goto fail;

	if (buf_size != SCREEN.buf_size) eprintf("\x1b[2J"); // clear on resize

	SCREEN.buf_size = buf_size;


	if (needs_clear) memset(SCREEN.buf, 0x00, SCREEN.buf_size);

	float total_length = 0;
	for (unsigned i = 0; i < system->count; ++i) total_length += system->chain[i].length;

	struct rectf
	        rect_from = RECTF(-total_length, -total_length, total_length * 2, total_length * 2), // max distance the pendulum can reach
	        rect_stretched = RECTF2(POSF2(0), stretch),
	        rect_to = get_fit_rectf(rect_stretched.size, RECTF2(POSF2(0), poss2f(SCREEN.size))); // letter-box rect to the display screen size

	struct posf pend_t = POSF(0, 0);
	for (unsigned i = 0; i < system->count; ++i) {
		struct pendulum *p = &system->chain[i];
		struct posf pend_f = pend_t;
		pend_t.x += sin(p->angle) * p->length;
		pend_t.y += cos(p->angle) * p->length;

		// draw line
		struct posf cell_f = map_rectf(pend_f, rect_from, rect_to),
		            cell_t = map_rectf(pend_t, rect_from, rect_to),
		            delta = posf_sub(cell_t, cell_f);
		if (delta.x == 0 && delta.y == 0) break;
		bool swap = fabsf(delta.y) > fabsf(delta.x);
		if (swap) { // swap x and y if gradient > 1 (45° from horizontal), otherwise there will be gaps since it loops over x-values
			SWAP_POSF(cell_f);
			SWAP_POSF(cell_t);
			SWAP_POSF(delta);
		}
		float gradient = delta.y / delta.x;
		if (delta.x < 0) SWAP(struct posf, cell_f, cell_t); // swap from/to values to make it easier to loop
		for (size_t x = floorf(cell_f.x); x <= ceilf(cell_t.x); ++x) {
			struct poss cell = POSS(x, roundf(gradient * (cell.x - cell_f.x) + cell_f.y)); // y=m*(x-x1)+y1
			if (swap) SWAP_POSS(cell);
			SET_CELL(cell, 1, SCREEN);
		}
	}

	if (info) {
		eprintf("\x1b[H");
		const char *newline;
		do {
			newline = strchr(info, '\n');
			size_t nbyte = newline ? newline - info : strlen(info);
			eprintf("\x1b[2K");                                     // clear line
			if (nbyte != write(DISPLAY_FD, info, nbyte)) goto fail; // write up until first newline
			eprintf("\x1b[E");                                      // next line
			info = newline + 1;
		} while (newline);
	}

	struct poss cursor = POSS2(0);

	struct poss term;
	for (term.y = 0; term.y < display.term_size.y; ++term.y)
		for (term.x = 0; term.x < display.term_size.x; ++term.x) {
			struct poss cell = poss_mul(term, block_size);

			static const char *chars[] = {" ", "▘", "▝", "▀", "▖", "▌", "▞", "▛", "▗", "▚", "▐", "▜", "▄", "▙", "▟", "█"};
			unsigned char index = 0;
			struct poss block;
			for (block.y = 0; block.y < block_size.y; ++block.y)
				for (block.x = 0; block.x < block_size.x; ++block.x)
					if (GET_CELL(poss_add(cell, block), SCREEN))
						index |= 1 << (block.y * block_size.x + block.x); // set corresponding bit for the block character

			if (index == 0) {
				if (SCREEN_OTHER.buf)
					for (block.y = 0; block.y < block_size.y; ++block.y)
						for (block.x = 0; block.x < block_size.x; ++block.x)
							if (GET_CELL(poss_add(cell, block), SCREEN_OTHER))
								goto draw_char; // override previous character
				// skip drawing empty space
				continue;
			}

		draw_char:
			const char *c = chars[index];
			(void) c;
			if (!poss_eq(term, cursor)) {
				eprintf("\x1b[%zu;%zuH", term.y + 1, term.x + 1);
				cursor.x = term.x, cursor.y = term.y;
			}
			eprintf("%s", c);
			++cursor.x;
		}

	res = true;
fail:
	fsync(DISPLAY_FD);
	return res;
}
