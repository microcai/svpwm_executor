
#if defined(AT32F421) || defined (AT32M4xx) || defined (AT32F415) || defined (AT32F405) || defined(AT32F402)

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

#include <cstdlib>

static motorlib::at32pwmdriver_impl* _g_instance = nullptr;

bool at32_board_specific_tmr_gpio_setup();

namespace motorlib
{
struct at32pwmdriver_impl
{
	at32pwmdriver* parent;
	tmr_type* tmr;
	// hardware variables
	int pwm_frequency = 8000;

	crm_clocks_freq_type crm_clocks_freq_struct;
	uint16_t timer_period = 0;

	enum output_gpio_state_t {
		PWMA_FLOAT = 1,
		PWMB_FLOAT = 2,
		PWMC_FLOAT = 4,
	};

	unsigned output_gpio_state = 0;

	at32pwmdriver_impl(at32pwmdriver* parent)
		: parent(parent)
		, tmr(TMR1)
	{
		_g_instance = this;
		// see https://www.arterychip.com/download/APNOTE/AN0069_AT32F421_GPIO_Application_Note_ZH_V2.0.1.pdf

		// AT32F415 的 pwm 输出并不能随意选择输出引脚。
		// 所以传入的是 tmr index, 选定了一个 定时器，就只能使用这个定时器配好的固定输出引脚


		crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

		if (!at32_board_specific_tmr_gpio_setup())
		{
			/* enable tmr1/gpioa/gpiob clock */
			crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
			crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
			// debug_periph_mode_set(DEBUG_TMR1_PAUSE, TRUE);

			gpio_init_type  gpio_init_struct = {0};
			gpio_default_para_init(&gpio_init_struct);

			/* timer1 output pin Configuration */
			gpio_init_struct.gpio_pins = GPIO_PINS_8|GPIO_PINS_9|GPIO_PINS_10;
			gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
			gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
			gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
			gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
			gpio_init(GPIOA, &gpio_init_struct);

			#ifdef AT32F42
			gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_2);
			gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_2);
			gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_2);
			#endif

			#if defined (AT32F421) || defined (AT32F405)
			gpio_init_struct.gpio_pins = GPIO_PINS_7;
			gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
			gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
			gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
			gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
			gpio_init(GPIOA, &gpio_init_struct);
			gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_2);

			gpio_init_struct.gpio_pins = GPIO_PINS_0|GPIO_PINS_1;
			gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
			gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
			gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
			gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
			gpio_init(GPIOB, &gpio_init_struct);

			gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE0, GPIO_MUX_2);
			gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE1, GPIO_MUX_2);
			#else
			gpio_init_struct.gpio_pins = GPIO_PINS_13|GPIO_PINS_14|GPIO_PINS_15;
			gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
			gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
			gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
			#if defined(AT32M4xx) || defined (AT32F403) || defined(AT32F402) || defined(AT32F405)
			gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
			#else
			gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MAXIMUM;
			#endif
			gpio_init(GPIOB, &gpio_init_struct);
			#endif
		}

		crm_clocks_freq_get(&crm_clocks_freq_struct);

		/* compute the value to be set in arr regiter to generate signal frequency*/
  		timer_period = (crm_clocks_freq_struct.sclk_freq / pwm_frequency/2 ) - 1;

		/* channel 1, 2, 3 configuration in output mode */
		tmr_output_config_type tmr_output_struct_normal;
		tmr_output_default_para_init(&tmr_output_struct_normal);
		tmr_output_struct_normal.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
		tmr_output_struct_normal.oc_output_state = TRUE;

		tmr_output_struct_normal.oc_polarity =  TMR_OUTPUT_ACTIVE_HIGH;
		#ifdef INVERT_LOW_SIDE
		tmr_output_struct_normal.occ_polarity = TMR_OUTPUT_ACTIVE_LOW;
		#else
		tmr_output_struct_normal.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
		#endif

		tmr_output_struct_normal.oc_idle_state = TRUE;
		tmr_output_struct_normal.occ_output_state = TRUE;
		tmr_output_struct_normal.occ_idle_state = TRUE;

		tmr_reset(tmr);
  		tmr_base_init(tmr, timer_period, 0);
		tmr_cnt_dir_set(tmr, TMR_COUNT_TWO_WAY_1);
		tmr_period_value_set(tmr, timer_period - 1);
		tmr_output_channel_config(tmr, TMR_SELECT_CHANNEL_1, &tmr_output_struct_normal);
#ifndef PWM_B_BROKEN
		tmr_output_channel_config(tmr, TMR_SELECT_CHANNEL_2, &tmr_output_struct_normal);
#endif
		tmr_output_channel_config(tmr, TMR_SELECT_CHANNEL_3, &tmr_output_struct_normal);
		tmr_output_channel_config(tmr, TMR_SELECT_CHANNEL_4, &tmr_output_struct_normal);

		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_1, TRUE);
#ifndef PWM_B_BROKEN
		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_2, TRUE);
#endif
		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_3, TRUE);
		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_4, TRUE);

		tmr_brkdt_config_type tmr_brkdt_config_struct;
		tmr_brkdt_default_para_init(&tmr_brkdt_config_struct);

#ifdef USE_BREAK
		{
			gpio_init_type  gpio_init_struct = {
				.gpio_pins = GPIO_PINS_12,
				.gpio_out_type = GPIO_OUTPUT_OPEN_DRAIN,
				.gpio_pull = GPIO_PULL_UP,
				.gpio_mode = GPIO_MODE_MUX,
				.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER,
			};
			gpio_init(GPIOB, &gpio_init_struct);
			gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE12, GPIO_MUX_2);
		}
		tmr_brkdt_config_struct.brk_enable = TRUE;
#else
		tmr_brkdt_config_struct.brk_enable = FALSE;
#endif
		tmr_brkdt_config_struct.auto_output_enable = FALSE;
		tmr_brkdt_config_struct.deadtime = 160;
		tmr_brkdt_config_struct.fcsoen_state = TRUE;
		tmr_brkdt_config_struct.fcsodis_state = FALSE;
		tmr_brkdt_config_struct.brk_polarity = TMR_BRK_INPUT_ACTIVE_LOW;
		tmr_brkdt_config_struct.wp_level = TMR_WP_OFF;
 		tmr_brkdt_config(TMR1, &tmr_brkdt_config_struct);


		tmr_channel_buffer_enable(tmr, TRUE);
		tmr_output_channel_buffer_enable(tmr, TMR_SELECT_CHANNEL_1, TRUE);
		tmr_output_channel_buffer_enable(tmr, TMR_SELECT_CHANNEL_2, TRUE);
		tmr_output_channel_buffer_enable(tmr, TMR_SELECT_CHANNEL_3, TRUE);
		tmr_output_channel_buffer_enable(tmr, TMR_SELECT_CHANNEL_4, TRUE);

		tmr_period_buffer_enable(tmr, TRUE);

		tmr_interrupt_enable(tmr, TMR_OVF_INT, TRUE);

		set_frequency(pwm_frequency);

		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_4, TRUE);
		tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_4, 2);//timer_period - 30);

		tmr_output_enable(tmr, TRUE);
		tmr_counter_enable(tmr, TRUE);
		start();

		#ifdef AT32F421
  		nvic_irq_enable(TMR1_CH_IRQn, 0, 0);
		#else
  		nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 0, 0);
		#endif

		set_duty(-1, -1, -1);
	}

	#if defined (AT32F405) || defined (AT32F402)
	void set_pwma_float()
	{
		gpio_init_type pwm_float_set = {
			.gpio_pins  = GPIO_PINS_8|GPIO_PINS_7,
			.gpio_out_type = GPIO_OUTPUT_PUSH_PULL,
			.gpio_pull = GPIO_PULL_NONE,
			.gpio_mode = GPIO_MODE_OUTPUT,
			.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER,
		};
		gpio_init(GPIOA, &pwm_float_set);
		gpio_bits_write(GPIOA, GPIO_PINS_8, FALSE);
		gpio_bits_write(GPIOA, GPIO_PINS_7, FALSE);
	}
	void set_pwma_pwm()
	{
		gpio_init_type pwm_float_set = {
			.gpio_pins  = GPIO_PINS_8|GPIO_PINS_7,
			.gpio_out_type = GPIO_OUTPUT_PUSH_PULL,
			.gpio_pull = GPIO_PULL_NONE,
			.gpio_mode = GPIO_MODE_MUX,
			.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER,
		};
		gpio_init(GPIOA, &pwm_float_set);
		gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_1);
		gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_1);
	}

	void set_pwmb_float()
	{
		gpio_init_type pwm_float_set = {
			.gpio_pins  = GPIO_PINS_9,
			.gpio_out_type = GPIO_OUTPUT_PUSH_PULL,
			.gpio_pull = GPIO_PULL_NONE,
			.gpio_mode = GPIO_MODE_OUTPUT,
			.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER,
		};
		gpio_init(GPIOA, &pwm_float_set);
		pwm_float_set.gpio_pins = GPIO_PINS_0;
		gpio_init(GPIOB, &pwm_float_set);

		gpio_bits_write(GPIOA, GPIO_PINS_9, FALSE);
		gpio_bits_write(GPIOB, GPIO_PINS_0, FALSE);
	}

	void set_pwmb_pwm()
	{
		gpio_init_type pwm_float_set = {
			.gpio_pins  = GPIO_PINS_9,
			.gpio_out_type = GPIO_OUTPUT_PUSH_PULL,
			.gpio_pull = GPIO_PULL_NONE,
			.gpio_mode = GPIO_MODE_MUX,
			.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER,
		};
		gpio_init(GPIOA, &pwm_float_set);
		pwm_float_set.gpio_pins = GPIO_PINS_0;
		gpio_init(GPIOB, &pwm_float_set);
		gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_1);
		gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE0, GPIO_MUX_1);

	}

	void set_pwmc_float()
	{
		gpio_init_type pwm_float_set = {
			.gpio_pins  = GPIO_PINS_10,
			.gpio_out_type = GPIO_OUTPUT_PUSH_PULL,
			.gpio_pull = GPIO_PULL_NONE,
			.gpio_mode = GPIO_MODE_OUTPUT,
			.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER,
		};
		gpio_init(GPIOA, &pwm_float_set);
		pwm_float_set.gpio_pins = GPIO_PINS_1;
		gpio_init(GPIOB, &pwm_float_set);

		gpio_bits_write(GPIOA, GPIO_PINS_10, FALSE);
		gpio_bits_write(GPIOB, GPIO_PINS_1, FALSE);
	}

	void set_pwmc_pwm()
	{
		gpio_init_type pwm_float_set = {
			.gpio_pins  = GPIO_PINS_10,
			.gpio_out_type = GPIO_OUTPUT_PUSH_PULL,
			.gpio_pull = GPIO_PULL_NONE,
			.gpio_mode = GPIO_MODE_MUX,
			.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER,
		};
		gpio_init(GPIOA, &pwm_float_set);
		pwm_float_set.gpio_pins = GPIO_PINS_1;
		gpio_init(GPIOB, &pwm_float_set);

		gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_1);
		gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE1, GPIO_MUX_1);

	}

	#else
	void set_pwma_float()
	{
		gpio_init_type pwm_float_set = {
			.gpio_pins  = GPIO_PINS_8,
			.gpio_out_type = GPIO_OUTPUT_PUSH_PULL,
			.gpio_pull = GPIO_PULL_NONE,
			.gpio_mode = GPIO_MODE_OUTPUT,
			.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER,
		};
		gpio_init(GPIOA, &pwm_float_set);
		pwm_float_set.gpio_pins = GPIO_PINS_13;
		gpio_init(GPIOB, &pwm_float_set);

		gpio_bits_write(GPIOA, GPIO_PINS_8, FALSE);
		gpio_bits_write(GPIOB, GPIO_PINS_13, FALSE);
	}

	void set_pwma_pwm()
	{
		gpio_init_type pwm_float_set = {
			.gpio_pins  = GPIO_PINS_8,
			.gpio_out_type = GPIO_OUTPUT_PUSH_PULL,
			.gpio_pull = GPIO_PULL_NONE,
			.gpio_mode = GPIO_MODE_MUX,
			.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER,
		};
		gpio_init(GPIOA, &pwm_float_set);
		pwm_float_set.gpio_pins = GPIO_PINS_13;
		gpio_init(GPIOB, &pwm_float_set);
	}

	void set_pwmb_float()
	{
		gpio_init_type pwm_float_set = {
			.gpio_pins  = GPIO_PINS_9,
			.gpio_out_type = GPIO_OUTPUT_PUSH_PULL,
			.gpio_pull = GPIO_PULL_NONE,
			.gpio_mode = GPIO_MODE_OUTPUT,
			.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER,
		};
		gpio_init(GPIOA, &pwm_float_set);
		pwm_float_set.gpio_pins = GPIO_PINS_14;
		gpio_init(GPIOB, &pwm_float_set);

		gpio_bits_write(GPIOA, GPIO_PINS_8, FALSE);
		gpio_bits_write(GPIOB, GPIO_PINS_13, FALSE);
	}

	void set_pwmb_pwm()
	{
		gpio_init_type pwm_float_set = {
			.gpio_pins  = GPIO_PINS_9,
			.gpio_out_type = GPIO_OUTPUT_PUSH_PULL,
			.gpio_pull = GPIO_PULL_NONE,
			.gpio_mode = GPIO_MODE_MUX,
			.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER,
		};
		gpio_init(GPIOA, &pwm_float_set);
		pwm_float_set.gpio_pins = GPIO_PINS_14;
		gpio_init(GPIOB, &pwm_float_set);
	}

	void set_pwmc_float()
	{
		gpio_init_type pwm_float_set = {
			.gpio_pins  = GPIO_PINS_10,
			.gpio_out_type = GPIO_OUTPUT_PUSH_PULL,
			.gpio_pull = GPIO_PULL_NONE,
			.gpio_mode = GPIO_MODE_OUTPUT,
			.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER,
		};
		gpio_init(GPIOA, &pwm_float_set);
		pwm_float_set.gpio_pins = GPIO_PINS_15;
		gpio_init(GPIOB, &pwm_float_set);

		gpio_bits_write(GPIOA, GPIO_PINS_8, FALSE);
		gpio_bits_write(GPIOB, GPIO_PINS_13, FALSE);
	}

	void set_pwmc_pwm()
	{
		gpio_init_type pwm_float_set = {
			.gpio_pins  = GPIO_PINS_10,
			.gpio_out_type = GPIO_OUTPUT_PUSH_PULL,
			.gpio_pull = GPIO_PULL_NONE,
			.gpio_mode = GPIO_MODE_MUX,
			.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER,
		};
		gpio_init(GPIOA, &pwm_float_set);
		pwm_float_set.gpio_pins = GPIO_PINS_15;
		gpio_init(GPIOB, &pwm_float_set);
	}
	#endif

	void set_duty(float U_a, float U_b, float U_c)
	{
		uint32_t channel1_pulse = static_cast<int>(U_a * timer_period);
		uint32_t channel2_pulse = static_cast<int>(U_b * timer_period);
		uint32_t channel3_pulse = static_cast<int>(U_c * timer_period);
		if (U_a < 0)
		{
			if (!(output_gpio_state & PWMA_FLOAT))
			{
				output_gpio_state |= PWMA_FLOAT;
				set_pwma_float();
			}
		}
		else
		{
			if (output_gpio_state & PWMA_FLOAT)
			{
				set_pwma_pwm();
				output_gpio_state &= ~PWMA_FLOAT;
			}

			tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_1, channel1_pulse);
		}

		#ifndef PWM_B_BROKEN

		if (U_b < 0)
		{
			if (!(output_gpio_state & PWMB_FLOAT))
			{
				output_gpio_state |= PWMB_FLOAT;
				set_pwmb_float();
			}
		}
		else
		{
			if (output_gpio_state & PWMB_FLOAT)
			{
				set_pwmb_pwm();
				output_gpio_state &= ~PWMB_FLOAT;
			}
			tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_2, channel2_pulse);
		}

		#endif

		if (U_c < 0)
		{
			if (!(output_gpio_state & PWMC_FLOAT))
			{
				output_gpio_state |= PWMC_FLOAT;
				set_pwmc_float();
			}
		}
		else
		{
			if (output_gpio_state & PWMC_FLOAT)
			{
				set_pwmc_pwm();
				output_gpio_state &= ~PWMC_FLOAT;
			}
			tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_3, channel3_pulse);
		}

		// if (timer_period > 100)
		// 	tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_4, timer_period - 30);
		// else
		// 	tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_4, 3);
	}

    void set_frequency(int freq)
	{
		pwm_frequency = freq;

		freq*=2;

		auto div = crm_clocks_freq_struct.sclk_freq / freq / 65536;
		timer_period = crm_clocks_freq_struct.sclk_freq / (div +1) / freq -1;
		tmr_div_value_set(tmr, div);
		tmr_period_value_set(tmr, timer_period+1);
		// tmr->brk_bit.dtc = 100 / (div +1);
	}

	void start()
	{
		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_1, TRUE);
		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_2, TRUE);
		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_3, TRUE);
		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_1C, TRUE);
		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_2C, TRUE);
		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_3C, TRUE);

		set_duty(0, 0, 0);

	}

    void stop()
	{
		set_duty(-1, -1, -1);
	}

	int tmr1_interrupt(int perids)
	{
		parent->tmr1_interrupt(perids);
		return perids;
	}
};

//////////////////////////////////////////////////////////////////////////////
at32pwmdriver::at32pwmdriver()
{
	static_assert( sizeof(impl_static_storage) >= sizeof (at32pwmdriver_impl) );
	impl = new (impl_static_storage) at32pwmdriver_impl(this);
}

at32pwmdriver::~at32pwmdriver()
{
	impl->~at32pwmdriver_impl();
}

// Function setting the duty cycle to the pwm pin (ex. analogWrite())
// - BLDC driver - 6PWM setting
// - hardware specific
void at32pwmdriver::set_duty(float U_a, float U_b, float U_c)
{
	impl->set_duty(U_a, U_b, U_c);
}

void at32pwmdriver::start()
{
	impl->start();
}

void at32pwmdriver::stop()
{
	impl->stop();
}

int at32pwmdriver::get_frequency()
{
	return impl->pwm_frequency;
}

void at32pwmdriver::set_frequency(int f)
{
	impl->set_frequency(f);
}

void at32pwmdriver::tmr1_interrupt(int perids)
{
	invoke_callbacks(impl->pwm_frequency, perids);
}

}

/**
  * @brief  tmr1 interrupt handler
  * @param  none
  * @retval none
  */
#ifdef AT32F405
extern "C" void TMR1_OVF_TMR10_IRQHandler()
#elif defined(AT32F415)
extern "C" void TMR1_OVF_TMR10_IRQHandler(void)
#else
extern "C" void TMR1_CH_IRQHandler(void)
#endif
{
	static std::atomic_flag in_isr {0};
	static __IO int missed_pwm_interrupt;
	static std::atomic_flag skip_odd_intrrupt{0};
	if(tmr_flag_get(TMR1, TMR_OVF_FLAG) != RESET)
	{
		if (skip_odd_intrrupt.test_and_set())
		{
			if (!in_isr.test_and_set())
			{
				int missed_pwm_interrupt_ = missed_pwm_interrupt;
				missed_pwm_interrupt = 0;
				tmr_flag_clear(TMR1, TMR_OVF_FLAG);
				_g_instance->tmr1_interrupt(missed_pwm_interrupt_ + 1);
				in_isr.clear();
			}
			else
			{
				++missed_pwm_interrupt;
				tmr_flag_clear(TMR1, TMR_OVF_FLAG);
			}

			skip_odd_intrrupt.clear();
		}
		else
		{
			tmr_flag_clear(TMR1, TMR_OVF_FLAG);
		}
	}
}

namespace os{
	void reset_mcu()
	{
		nvic_system_reset();
	}
}

__WEAK bool at32_board_specific_tmr_gpio_setup(){ return false ; }

#endif // defined(STM32F4xx)
