/* Platform clock include. */
#include "clock.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

uint32_t Clock_GetTimeMs( void )
{
    /* esp_timer_get_time is in microseconds, converting to milliseconds */
    int64_t timeMs = esp_timer_get_time() / 1000;

    /* Libraries need only the lower 32 bits of the time in milliseconds, since
     * this function is used only for calculating the time difference.
     * Also, the possible overflows of this time value are handled by the
     * libraries. */
    return ( uint32_t ) timeMs;
}

/*-----------------------------------------------------------*/

void Clock_SleepMs( uint32_t sleepTimeMs )
{
    vTaskDelay( sleepTimeMs/portTICK_PERIOD_MS );
}