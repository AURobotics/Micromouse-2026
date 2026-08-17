#ifndef __FREERTOS_H__
#define __FREERTOS_H__

#ifdef __cplusplus
extern "C" {
#endif

    /* Includes ------------------------------------------------------------------*/
#include "main.h"

    /* USER CODE BEGIN Includes */
#include "cmsis_os.h"
    /* USER CODE END Includes */

    extern I2C_HandleTypeDef hi2c1;

    extern I2C_HandleTypeDef hi2c2;

    /* USER CODE BEGIN Private defines */

    /* Debug logging output selector, shared by any file that wants to call
     * Logger_SetOutput() (defined in freertos.c). */
    typedef enum
    {
        LOG_OUTPUT_USB = 0,
        LOG_OUTPUT_STLINK,
        LOG_OUTPUT_BLUETOOTH,
        LOG_OUTPUT_COUNT
      } LogOutput_t;

    /* USER CODE END Private defines */

    void MX_I2C1_Init(void);
    void MX_I2C2_Init(void);

    /* USER CODE BEGIN Prototypes */

    /* Public logger API (implemented in freertos.c) */
    void       Logger_SetOutput(LogOutput_t output);
    osStatus_t Logger_Print(uint32_t period_ms, const char *format, ...);
    uint32_t   Logger_GetDroppedCount(void);

    /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FREERTOS_H__ */