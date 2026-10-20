/* Newlib support. Match the Keil printf destination: USART1. */
#include "usart.h"
#include <errno.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/stat.h>

extern unsigned char __HeapBase;
extern unsigned char __HeapLimit;

void *_sbrk(ptrdiff_t increment)
{
    static uintptr_t current;
    const uintptr_t base = (uintptr_t)&__HeapBase;
    const uintptr_t limit = (uintptr_t)&__HeapLimit;
    if (current == 0) current = base;
    if ((increment >= 0 && (uintptr_t)increment > limit - current) ||
        (increment < 0 && (uintptr_t)(-(increment + 1)) + 1 > current - base))
    {
        errno = ENOMEM;
        return (void *)-1;
    }
    void *previous = (void *)current;
    current += increment;
    return previous;
}

int _write(int fd, char *buffer, int length)
{
    if (fd != 1 && fd != 2) { errno = EBADF; return -1; }
    for (int i = 0; i < length; ++i)
    {
        if (HAL_UART_Transmit(&huart1, (uint8_t *)&buffer[i], 1, 10) != HAL_OK)
        {
            errno = EIO;
            return i ? i : -1;
        }
    }
    return length;
}

int _read(int fd, char *buffer, int length)
{
    (void)fd; (void)buffer; (void)length;
    errno = ENOSYS;
    return -1;
}
int _close(int fd) { (void)fd; errno = EBADF; return -1; }
int _fstat(int fd, struct stat *st)
{
    if (fd < 0 || fd > 2) { errno = EBADF; return -1; }
    st->st_mode = S_IFCHR;
    return 0;
}
int _isatty(int fd) { return fd >= 0 && fd <= 2; }
int _lseek(int fd, int offset, int whence)
{
    (void)fd; (void)offset; (void)whence;
    errno = ESPIPE;
    return -1;
}
int _getpid(void) { return 1; }
int _kill(int pid, int signal)
{
    (void)pid; (void)signal; errno = EINVAL; return -1;
}
void _exit(int status)
{
    (void)status;
    __disable_irq();
    for (;;) { __NOP(); }
}
