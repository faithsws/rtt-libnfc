#ifndef _ERR_H_
#define _ERR_H_

#include <stdlib.h>

#define warnx(...) do { \
    rt_kprintf ( __VA_ARGS__); \
  } while (0)

#define errx(code, ...) do { \
	rt_kprintf ( __VA_ARGS__); \
  } while (0)

#define err errx

#endif /* !_ERR_H_ */
