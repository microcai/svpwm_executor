
#include "pwmdriver.hpp"
#include "at32pwm.hpp"

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

#include "tmr2_eclipsed_timer.h"

void tmr2_eclipse_timer_init()
{
  /* add user code begin tmr2_init 0 */
  crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);

  /* add user code end tmr2_init 0 */

  /* add user code begin tmr2_init 1 */

  /* add user code end tmr2_init 1 */

  /* configure plus mode */
  tmr_32_bit_function_enable(TMR2, TRUE);

  /* configure counter settings */
  tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
  tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
  tmr_period_buffer_enable(TMR2, FALSE);
  tmr_base_init(TMR2, 4294967294, 0);

  tmr_counter_enable(TMR2, TRUE);

  /* add user code begin tmr2_init 2 */

  /* add user code end tmr2_init 2 */
}

int get_eclipsed()
{
    return tmr_counter_value_get(TMR2);
}
int get_eclipsed_and_reset()
{
    auto v = tmr_counter_value_get(TMR2);
    tmr_counter_value_set(TMR2, 0);
    return v;
}
