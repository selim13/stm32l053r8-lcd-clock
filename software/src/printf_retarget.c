#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include <stm32l0xx_ll_lpuart.h>

int _write(int file, char *data, int len)
{
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    }

    int i = 0;
    for (; i < len; i++) {
        LL_LPUART_TransmitData8(LPUART1, data[i]);
        while (!LL_LPUART_IsActiveFlag_TC(LPUART1))
            ;
    }

    return i;
}
