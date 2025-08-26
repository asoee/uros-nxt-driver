#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <rmw_microros/rmw_microros.h>
#include "pico_uart_transports.h"

void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

bool pico_serial_transport_open(struct uxrCustomTransport * transport)
{
    // Ensure that stdio_init_all is only called once on the runtime
    static bool require_init = true;
    if(require_init)
    {
        stdio_init_all();
        
        // For USB CDC, baud rate doesn't matter but some systems expect it
        // This is mainly for compatibility
        require_init = false;
    }

    return true;
}

bool pico_serial_transport_close(struct uxrCustomTransport * transport)
{
    return true;
}

size_t pico_serial_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *err)
{
    for (size_t i = 0; i < len; i++)
    {
        if (buf[i] != putchar(buf[i]))
        {
            *err = 1;
            return i;
        }
    }
    return len;
}

size_t pico_serial_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    uint64_t start_time_us = time_us_64();
    for (size_t i = 0; i < len; i++)
    {
        int64_t elapsed_time_us = timeout * 1000 - (time_us_64() - start_time_us);
        if (elapsed_time_us < 0)
        {
            *err = 1;
            return i;
        }

        int character = getchar_timeout_us(elapsed_time_us);
        if (character == PICO_ERROR_TIMEOUT)
        {
            *err = 1;
            return i;
        }
        buf[i] = character;
    }
    return len;
}

bool set_microros_transports()
{
    rmw_uros_set_custom_transport(
        true, // framing enabled - needed for micro-ROS protocol
        (void *) NULL, // optional data parameter  
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );
    return true;
}
