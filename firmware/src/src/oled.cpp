
#include <stdio.h>
#include <stdlib.h>
#include <zephyr.h>
#include <device.h>
#include <display/cfb.h>
#include <drivers/display.h>
#include <kernel.h>

#include "oled.h"
#include "log.h"

static const struct device *display = DEVICE_DT_GET(DT_NODELABEL(ssd1306));

void oled_Thread()
{
/* waiting until log functions initialize */
  rt_sleep_ms(3000);

  LOGI("Oled thread started");

  if (display == NULL) {
    LOGI("device pointer is NULL");
  } else {
    LOGI("device pointer is OK");
  }

  while (1) {
    rt_sleep_ms(500);
  }
}