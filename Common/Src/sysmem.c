/* Heap support for the CubeMX GCC build. The heap lives after AXI .bss. */
#include <errno.h>
#include <stddef.h>
#include <stdint.h>

void *_sbrk(ptrdiff_t incr)
{
    extern uint8_t _end;
    extern uint8_t __heap_end__;
    static uint8_t *heap_end;
    uint8_t *previous;
    uint8_t *limit = &__heap_end__;

    if (heap_end == NULL)
    {
        heap_end = &_end;
    }

    previous = heap_end;
    if (incr < 0 || heap_end > limit - incr)
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_end += incr;
    return previous;
}
