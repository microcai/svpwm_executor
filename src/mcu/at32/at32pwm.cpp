
#include "gpio.h"
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
	struct pin_config
	{
		gpio_type* port;
		gpio_init_type port_cfg;
		gpio_pins_source_type pin_source;
		gpio_mux_sel_type mux_type;
	};

	static void port_setup(const pin_config& pin)
	{
		gpio_pin_mux_config(pin.port, pin.pin_source, pin.mux_type);
		gpio_init(pin.port, (gpio_init_type*) &pin.port_cfg);
	}

struct at32pwmdriver_impl
{
	at32pwmdriver* parent;
	tmr_type* tmr;
	// hardware variables
	int pwm_frequency = 12000;

	crm_clocks_freq_type crm_clocks_freq_struct;
	uint16_t timer_period = 0;

	enum output_gpio_state_t {
		PWMA_FLOAT = 1,
		PWMB_FLOAT = 2,
		PWMC_FLOAT = 4,
	};

	unsigned output_gpio_state = 0;

	pin_config hpwm_pins[3] = {
		{
			GPIOA,
			{
				GPIO_PINS_8,
				GPIO_OUTPUT_PUSH_PULL,
				GPIO_PULL_NONE,
				GPIO_MODE_MUX,
				GPIO_DRIVE_STRENGTH_STRONGER				
			},
			GPIO_PINS_SOURCE8,
			GPIO_MUX_1
		},
		{
			GPIOA,
			{
				GPIO_PINS_9,
				GPIO_OUTPUT_PUSH_PULL,
				GPIO_PULL_NONE,
				GPIO_MODE_MUX,
				GPIO_DRIVE_STRENGTH_STRONGER				
			},
			GPIO_PINS_SOURCE9,
			GPIO_MUX_1
		},
		{
			GPIOA,
			{
				GPIO_PINS_10,
				GPIO_OUTPUT_PUSH_PULL,
				GPIO_PULL_NONE,
				GPIO_MODE_MUX,
				GPIO_DRIVE_STRENGTH_STRONGER				
			},
			GPIO_PINS_SOURCE10,
			GPIO_MUX_1			
		}
	};
	pin_config lpwm_pins[3] = {
		{
			GPIOA,
			{
				GPIO_PINS_7,
				GPIO_OUTPUT_PUSH_PULL,
				GPIO_PULL_NONE,
				GPIO_MODE_MUX,
				GPIO_DRIVE_STRENGTH_STRONGER				
			},
			GPIO_PINS_SOURCE7,
			GPIO_MUX_1			
		},
		{
			GPIOB,
			{
				GPIO_PINS_0,
				GPIO_OUTPUT_PUSH_PULL,
				GPIO_PULL_NONE,
				GPIO_MODE_MUX,
				GPIO_DRIVE_STRENGTH_STRONGER				
			},
			GPIO_PINS_SOURCE0,
			GPIO_MUX_1			
		},
		{
			GPIOB,
			{
				GPIO_PINS_1,
				GPIO_OUTPUT_PUSH_PULL,
				GPIO_PULL_NONE,
				GPIO_MODE_MUX,
				GPIO_DRIVE_STRENGTH_STRONGER				
			},
			GPIO_PINS_SOURCE1,
			GPIO_MUX_1			
		}
	};

	pin_config brk_pin = {
		GPIOB,
		{
			GPIO_PINS_12,
			GPIO_OUTPUT_OPEN_DRAIN,
			GPIO_PULL_UP,
			GPIO_MODE_MUX,
			GPIO_DRIVE_STRENGTH_STRONGER				
		},
		GPIO_PINS_SOURCE12,
		GPIO_MUX_2			
	};

	at32pwmdriver_impl(at32pwmdriver* parent)
		: parent(parent)
		, tmr(TMR1)
	{
		// see https://www.arterychip.com/download/APNOTE/AN0069_AT32F421_GPIO_Application_Note_ZH_V2.0.1.pdf

		crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
		crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

		port_setup(hpwm_pins[0]);
#ifndef PWM_B_BROKEN
		port_setup(hpwm_pins[1]);
#endif
		port_setup(hpwm_pins[2]);
		port_setup(lpwm_pins[0]);
#ifndef PWM_B_BROKEN
		port_setup(lpwm_pins[1]);
#endif
		port_setup(lpwm_pins[2]);

		port_setup(brk_pin);

		crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

		crm_clocks_freq_get(&crm_clocks_freq_struct);

		/* compute the value to be set in arr regiter to generate signal frequency*/
  		timer_period = (crm_clocks_freq_struct.sclk_freq / 2/ pwm_frequency ) - 1;

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
  		tmr_base_init(tmr, timer_period, TMR_CLOCK_DIV1);
		tmr_cnt_dir_set(tmr, TMR_COUNT_TWO_WAY_1);
		tmr_period_value_set(tmr, timer_period - 1);
		tmr_output_channel_config(tmr, TMR_SELECT_CHANNEL_1, &tmr_output_struct_normal);
		tmr_output_channel_config(tmr, TMR_SELECT_CHANNEL_2, &tmr_output_struct_normal);
		tmr_output_channel_config(tmr, TMR_SELECT_CHANNEL_3, &tmr_output_struct_normal);
		tmr_output_channel_config(tmr, TMR_SELECT_CHANNEL_4, &tmr_output_struct_normal);

		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_1, TRUE);
		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_2, TRUE);
		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_3, TRUE);
		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_4, TRUE);

		tmr_brkdt_config_type tmr_brkdt_config_struct;
		tmr_brkdt_default_para_init(&tmr_brkdt_config_struct);

#ifdef USE_BREAK
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

		set_frequency(pwm_frequency);

		tmr_channel_enable(tmr, TMR_SELECT_CHANNEL_4, TRUE);
		tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_4, 2);//timer_period - 30);

		tmr_output_enable(tmr, TRUE);
		tmr_counter_enable(tmr, TRUE);

		_g_instance = this;

		start();

		#ifdef AT32F421
  		nvic_irq_enable(TMR1_CH_IRQn, 0, 0);
		#else
  		nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 1, 1);
		nvic_irq_enable(TMR1_BRK_TMR9_IRQn, 0, 0);
		#endif

		tmr_interrupt_enable(tmr, TMR_OVF_INT, TRUE);
		tmr_interrupt_enable(TMR1, TMR_BRK_INT, TRUE);

		set_duty(-1, -1, -1);
	}

	void set_pwm_float(int channel)
	{
		gpio_init_type pwm_float_set = hpwm_pins[channel].port_cfg;
		pwm_float_set.gpio_mode = GPIO_MODE_OUTPUT;
		gpio_init(hpwm_pins[channel].port, &pwm_float_set);

		pwm_float_set = lpwm_pins[channel].port_cfg;
		pwm_float_set.gpio_mode = GPIO_MODE_OUTPUT;
		gpio_init(lpwm_pins[channel].port, &pwm_float_set);

		gpio_bits_reset(hpwm_pins[channel].port, hpwm_pins[channel].port_cfg.gpio_pins);
		gpio_bits_reset(lpwm_pins[channel].port, lpwm_pins[channel].port_cfg.gpio_pins);
	}

	void set_pwm_pwm(int channel)
	{
		port_setup(hpwm_pins[channel]);

		gpio_init_type pwm_float_set = lpwm_pins[channel].port_cfg;
		pwm_float_set.gpio_mode = GPIO_MODE_OUTPUT;
		gpio_init(lpwm_pins[channel].port, &pwm_float_set);

		gpio_bits_reset(lpwm_pins[channel].port, lpwm_pins[channel].port_cfg.gpio_pins);
	}

	void set_pwm_cpwm(int channel)
	{
		port_setup(hpwm_pins[channel]);
		port_setup(lpwm_pins[channel]);
	}

	void set_pwm_low(int channel)
	{
		gpio_init_type pwm_float_set = hpwm_pins[channel].port_cfg;
		pwm_float_set.gpio_mode = GPIO_MODE_OUTPUT;
		gpio_init(hpwm_pins[channel].port, &pwm_float_set);

		pwm_float_set = lpwm_pins[channel].port_cfg;
		pwm_float_set.gpio_mode = GPIO_MODE_OUTPUT;
		gpio_init(lpwm_pins[channel].port, &pwm_float_set);

		gpio_bits_reset(hpwm_pins[channel].port, hpwm_pins[channel].port_cfg.gpio_pins);
		gpio_bits_set(lpwm_pins[channel].port, lpwm_pins[channel].port_cfg.gpio_pins);
	}

	void set_duty(float U_c, float U_b, float U_a)
	{
		m_step = -1;
#ifdef PWM_B_BROKEN
		U_b = -1;
#endif
		uint32_t channel1_pulse = static_cast<int>(U_a * timer_period);
		uint32_t channel2_pulse = static_cast<int>(U_b * timer_period);
		uint32_t channel3_pulse = static_cast<int>(U_c * timer_period);
		if (U_a < 0)
		{
			if (!(output_gpio_state & PWMA_FLOAT))
			{
				output_gpio_state |= PWMA_FLOAT;
				set_pwm_float(0);
			}

			tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_1, 0);

		}
		else
		{
			if (output_gpio_state & PWMA_FLOAT)
			{
				set_pwm_cpwm(0);
				output_gpio_state &= ~PWMA_FLOAT;
			}

			tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_1, channel1_pulse);
		}

		if (U_b < 0)
		{
			if (!(output_gpio_state & PWMB_FLOAT))
			{
				output_gpio_state |= PWMB_FLOAT;
				set_pwm_float(1);
			}
			tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_2, 0);
		}
		else
		{
			if (output_gpio_state & PWMB_FLOAT)
			{
				set_pwm_cpwm(1);
				output_gpio_state &= ~PWMB_FLOAT;
			}
			tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_2, channel2_pulse);
		}

		if (U_c < 0)
		{
			if (!(output_gpio_state & PWMC_FLOAT))
			{
				output_gpio_state |= PWMC_FLOAT;
				set_pwm_float(2);
			}
			tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_3, 0);
		}
		else
		{
			if (output_gpio_state & PWMC_FLOAT)
			{
				set_pwm_cpwm(2);
				output_gpio_state &= ~PWMC_FLOAT;
			}
			tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_3, channel3_pulse);
		}
	}

	int m_step = -1;

	void set_6step(int step, float PWM)
	{
		bool sector_changed = m_step != step;
		m_step = step;
		uint32_t channel_pulse = static_cast<int>(PWM * timer_period);

		switch (m_step)
		{
			case 0: // CA
				if (sector_changed)
				{
					set_pwm_float(1);
					set_pwm_pwm(2);
					set_pwm_low(0);
				}
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_1, 0);
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_2, 0);
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_3, channel_pulse);
				break;
			case 1: // BA
				if (sector_changed)
				{
					set_pwm_float(2);
					set_pwm_pwm(1);
					set_pwm_low(0);
				}
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_1, 0);
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_2, channel_pulse);
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_3, 0);
				break;
			case 2: // BC
				if (sector_changed)
				{
					set_pwm_float(0);
					set_pwm_pwm(1);
					set_pwm_low(2);
				}
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_1, 0);
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_2, channel_pulse);
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_3, 0);
				break;
			case 3: //AC
				if (sector_changed)
				{
					set_pwm_float(1);
					set_pwm_pwm(0);
					set_pwm_low(2);
				}
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_1, channel_pulse);
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_2, 0);
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_3, 0);
				break;
			case 4: // AB
				if (sector_changed)
				{
					set_pwm_float(2);
					set_pwm_pwm(0);
					set_pwm_low(1);
				}
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_1, channel_pulse);
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_2, 0);
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_3, 0);
				break;
			case 5: // CB
				if (sector_changed)
				{
					set_pwm_float(0);
					set_pwm_pwm(2);
					set_pwm_low(1);
				}
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_1, 0);
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_2, 0);
				tmr_channel_value_set(tmr, TMR_SELECT_CHANNEL_3, channel_pulse);
				break;
			default:
				set_pwm_float(0);
				set_pwm_float(1);
				set_pwm_float(2);
				break;
		}

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

		tmr_interrupt_enable(TMR1, TMR_BRK_INT, TRUE);
		tmr_output_enable(tmr, TRUE);
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

void at32pwmdriver::set_6step(int step, float PWM)
{
	impl->set_6step(step, PWM);
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
	static __IO int missed_pwm_interrupt {0};
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
				if (_g_instance)
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

extern "C" void TMR1_BRK_TMR9_IRQHandler(void)
{
  	/* TMR9_CH1 toggling with frequency = 366.2 Hz */
	if (tmr_interrupt_flag_get(TMR1, TMR_BRK_FLAG) != RESET)
	{
		tmr_flag_clear(TMR1, TMR_BRK_FLAG );
		_g_instance->parent->break_status = 1;
		tmr_interrupt_enable(TMR1, TMR_BRK_INT, FALSE);
	}
}

namespace os{
	void reset_mcu()
	{
		nvic_system_reset();
	}
}

#endif // defined(STM32F4xx)
