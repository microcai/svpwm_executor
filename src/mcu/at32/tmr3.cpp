


#if defined (AT32F415)

#include "at32f415.h"
#include "at32f415_tmr.h"
#include "at32f415_crm.h"
#include "at32f415_gpio.h"

#elif defined(AT32F403)

#include "at32f403a_407.h"
#include "at32f403a_407_tmr.h"
#include "at32f403a_407_crm.h"
#include "at32f403a_407_gpio.h"

#elif defined(AT32F421)

#include "at32f421.h"
#include "at32f421_tmr.h"
#include "at32f421_crm.h"
#include "at32f421_gpio.h"

#elif defined(AT32M4xx)

#include "at32m412_416.h"
#include "at32m412_416_tmr.h"
#include "at32m412_416_crm.h"
#include "at32m412_416_gpio.h"

#elif defined(AT32F402) || defined (AT32F405)

#include "at32f402_405.h"
#include "at32f402_405_tmr.h"
#include "at32f402_405_crm.h"
#include "at32f402_405_gpio.h"

#endif

/**
  * @brief  init tmr3 function.
  * @param  none
  * @retval none
  */
void encoder_tmr3_init(void)
{
  /* add user code begin tmr3_init 0 */
  crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);

  /* add user code end tmr3_init 0 */

  gpio_init_type gpio_init_struct;
  tmr_input_config_type  tmr_input_struct;

  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin tmr3_init 1 */

  /* add user code end tmr3_init 1 */

  /* configure the tmr3 CH1 pin */
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE6, GPIO_MUX_2);
  gpio_init_struct.gpio_pins = GPIO_PINS_6;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(GPIOC, &gpio_init_struct);

  /* configure the tmr3 CH2 pin */
  gpio_pin_mux_config(GPIOC, GPIO_PINS_SOURCE7, GPIO_MUX_2);
  gpio_init_struct.gpio_pins = GPIO_PINS_7;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init(GPIOC, &gpio_init_struct);

  /* configure counter settings */
  tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR3, TMR_CLOCK_DIV1);
  tmr_period_buffer_enable(TMR3, FALSE);
  tmr_base_init(TMR3, 10000, 0);

  /* configure primary mode settings */
  tmr_sub_sync_mode_set(TMR3, FALSE);
  tmr_primary_mode_select(TMR3, TMR_PRIMARY_SEL_RESET);

  /* configure encoder mode */
  tmr_input_struct.input_channel_select = TMR_SELECT_CHANNEL_1;
  tmr_input_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
  tmr_input_struct.input_polarity_select = TMR_INPUT_RISING_EDGE;
  tmr_input_struct.input_filter_value = 6;
  tmr_input_channel_init(TMR3, &tmr_input_struct, TMR_CHANNEL_INPUT_DIV_1);

  tmr_input_struct.input_channel_select = TMR_SELECT_CHANNEL_2;
  tmr_input_struct.input_mapped_select = TMR_CC_CHANNEL_MAPPED_DIRECT;
  tmr_input_struct.input_polarity_select = TMR_INPUT_RISING_EDGE;
  tmr_input_struct.input_filter_value = 6;
  tmr_input_channel_init(TMR3, &tmr_input_struct, TMR_CHANNEL_INPUT_DIV_1);

  tmr_encoder_mode_config(TMR3, TMR_ENCODER_MODE_C, TMR_INPUT_RISING_EDGE, TMR_INPUT_RISING_EDGE);

  tmr_counter_enable(TMR3, TRUE);

  /* add user code begin tmr3_init 2 */

  /* add user code end tmr3_init 2 */
}
