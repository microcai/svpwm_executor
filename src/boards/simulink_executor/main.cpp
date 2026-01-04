
#if defined (BOARD_SIMULINK_BOARD)

#include <Arduino.h>
#include <USBSerial.h>

#include "coroutine.hpp"
#include "mcu_coro.hpp"
#include "coro_condvar.hpp"
#include "pins.hpp"

#include "led_status.hpp"

#include "at32pwm.hpp"

#include "mtl.hpp"

mtl::static_store<motorlib::at32pwmdriver> driver_store;

static __IO uint16_t ADC_Converted_Data[8] = { 0 };

#define ADC_IDX_ISENSEA 0
#define ADC_IDX_ISENSEA_REF 1
#define ADC_IDX_ISENSEB 2
#define ADC_IDX_ISENSEBREF  3

#define ADC_IDX_C_OUT 4
#define ADC_IDX_B_OUT 5
#define ADC_IDX_A_OUT 6
#define ADC_IDX_VBUS  7

static inline int max(int a, int b, int c)
{
	int max = a;
	if (b > max)
		max = b;
	if (c > max)
		max = c;
	return max;
}

void ADC_init()
{
	gpio_init_type gpio_initstructure;
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

	gpio_default_para_init(&gpio_initstructure);
	gpio_initstructure.gpio_mode = GPIO_MODE_ANALOG;
	gpio_initstructure.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_2 | GPIO_PINS_3;
	gpio_init(GPIOA, &gpio_initstructure);
	gpio_init(GPIOC, &gpio_initstructure);

	dma_init_type dma_init_struct;
	dma_reset(DMA1_CHANNEL1);
	dma_default_para_init(&dma_init_struct);
	dma_init_struct.buffer_size = sizeof(ADC_Converted_Data)/sizeof(ADC_Converted_Data[0]);
	dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
	dma_init_struct.memory_base_addr = (uint32_t)ADC_Converted_Data;
	dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
	dma_init_struct.memory_inc_enable = TRUE;
	dma_init_struct.peripheral_base_addr = (uint32_t)&(ADC1->odt);
	dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
	dma_init_struct.peripheral_inc_enable = FALSE;
	dma_init_struct.priority = DMA_PRIORITY_VERY_HIGH;
	dma_init_struct.loop_mode_enable = TRUE;
	dma_init(DMA1_CHANNEL1, &dma_init_struct);
	dmamux_enable(DMA1, TRUE);
	dmamux_init(DMA1MUX_CHANNEL1, DMAMUX_DMAREQ_ID_ADC1);

	// dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
	dma_channel_enable(DMA1_CHANNEL1, TRUE);

	// nvic_irq_enable(DMA1_Channel1_IRQn, 0, 0);
	adc_enable(ADC1, FALSE);
	adc_base_config_type adc_base_struct;
	crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
	adc_reset(ADC1);
	adc_clock_div_set(ADC_DIV_8);

	adc_base_struct.sequence_mode = TRUE;
	adc_base_struct.repeat_mode = TRUE;
	adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
	adc_base_struct.ordinary_channel_length = dma_init_struct.buffer_size;
	adc_base_config(ADC1, &adc_base_struct);
	// ADC_isense bus
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_0, 1, ADC_SAMPLETIME_1_5);
	// ADC_vsense bus
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_1, 2, ADC_SAMPLETIME_1_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_2, 3, ADC_SAMPLETIME_1_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_3, 4, ADC_SAMPLETIME_1_5);

	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_10, 5, ADC_SAMPLETIME_1_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_11, 6, ADC_SAMPLETIME_1_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_12, 7, ADC_SAMPLETIME_1_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_13, 8, ADC_SAMPLETIME_1_5);

	adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_TMR1CH4, TRUE);

	adc_enable(ADC1, TRUE);
	delay_ms(50);
	adc_calibration_init(ADC1);
	while(adc_calibration_init_status_get(ADC1));
	adc_calibration_start(ADC1);
	while(adc_calibration_status_get(ADC1));

	adc_dma_mode_enable(ADC1, TRUE);

	nvic_irq_enable(ADC1_IRQn, 0 , 0);
}

mcucoro::condition_variable one_ms_interval;

void interval_setup(int freq)
{
	crm_periph_clock_enable(CRM_TMR6_PERIPH_CLOCK, TRUE);
	/* configure counter settings */
	tmr_cnt_dir_set(TMR6, TMR_COUNT_UP);
	tmr_period_buffer_enable(TMR6, TRUE);

	int tmr_clock_div = system_core_clock /freq / 65536;
	int peroid_counter = system_core_clock / (tmr_clock_div+1) / freq - 1;
	tmr_base_init(TMR6, peroid_counter, tmr_clock_div);

	tmr_counter_enable(TMR6, TRUE);

	nvic_irq_enable(TMR6_GLOBAL_IRQn, 1, 1);
	/* enable ovfien interrupt */
	tmr_interrupt_enable(TMR6, TMR_OVF_INT, TRUE);
}

mcucoro::awaitable<void> dc_mode_usb_commander(motorlib::pwmdriver* pwm_driver)
{
	struct dc_mode_command_packet{
		uint16_t header;
		int16_t duty;
	};

	for(;;)
	{
		co_await coro_delay_ms(0);

		char input_buffer[64];

		// read data from USB cdc
		auto len = SerialUSB.readBytes(input_buffer, 64);
		if (len == 4)
		{
			auto cmd_pkt = reinterpret_cast<dc_mode_command_packet*>(input_buffer);
			if (cmd_pkt->header = 0xDCDC)
			{
				auto duty = cmd_pkt->duty;

				// set duty to hardware!
				if (duty == 0)
				{
					pwm_driver->set_duty(float_number{-1}, float_number{-1}, float_number{-1});
				}
				else if (duty > 0)
				{
					pwm_driver->set_duty(float_number(duty)/32767, float_number{0}, float_number{-1});
				}
				else
				{
					pwm_driver->set_duty(float_number{0}, float_number(duty)/32767, float_number{-1});
				}
			}

		}

	}
}

mcucoro::awaitable<void> dc_mode_usb_reporter()
{
	interval_setup(1000);

	struct dc_mode_report_packet
	{
		uint16_t header; // header must be 0xDCDC
		int16_t DC_current;
		uint16_t BUS_Voltage;
		uint32_t time_stamp;
	};

	dc_mode_report_packet report_packet = {0xDCDC, 0, 0, 0};

	for(;;)
	{
		co_await one_ms_interval.wait();
		co_await leave_isr();
		report_packet.time_stamp = millis();
		report_packet.BUS_Voltage = ADC_Converted_Data[ADC_IDX_VBUS];
		report_packet.DC_current = ADC_Converted_Data[ADC_IDX_ISENSEA];

		SerialUSB.write((const uint8_t*) &report_packet, sizeof(report_packet));
	}
}

mcucoro::awaitable<void> svpwm_mode_usb_commander(motorlib::pwmdriver* pwm_driver)
{
	struct svpwm_mode_command_packet{
		uint16_t header; // must be 0x9D33
		int16_t dutyA;
		int16_t dutyB;
		int16_t dutyC;
	};

	for(;;)
	{
		co_await coro_delay_ms(0);

		char input_buffer[64];

		// read data from USB cdc
		auto len = SerialUSB.readBytes(input_buffer, 64);
		if (len == 4)
		{
			const svpwm_mode_command_packet* cmd_pkt = reinterpret_cast<const svpwm_mode_command_packet*>(input_buffer);
			if (cmd_pkt->header == 0x9D33)
			{
				pwm_driver->set_duty(float_number{cmd_pkt->dutyA}/32767, float_number{cmd_pkt->dutyB}/32767, float_number{cmd_pkt->dutyC}/32767);
			}
		}
	}
}

mcucoro::awaitable<void> svpwm_mode_usb_reporter()
{

	struct svpwm_mode_report_packet
	{
		uint16_t header; // header must be 0x9D33
		uint16_t hall_state;
		float time_stamp;
		float BUS_Voltage;
		float A_current;
		float B_current;
		float C_current;
		uint32_t tail;
	};

	svpwm_mode_report_packet report_packet = {0x9D33};
	report_packet.tail = 0x7f800000;

	interval_setup(2000);

	for(;;)
	{
		co_await one_ms_interval.wait();
		co_await leave_isr();
		report_packet.time_stamp = millis()/1000.0f;
		report_packet.BUS_Voltage = ADC_Converted_Data[ADC_IDX_VBUS];
		report_packet.A_current = ADC_Converted_Data[ADC_IDX_ISENSEA];
		report_packet.B_current = ADC_Converted_Data[ADC_IDX_ISENSEB];
		report_packet.C_current = 0 - report_packet.A_current - report_packet.B_current;
		report_packet.hall_state = digitalRead(PC6) + (digitalRead(PC7) << 1) + (digitalRead(PC8) << 2);

		SerialUSB.write((const uint8_t*) &report_packet, sizeof(report_packet));
	}
}

void setup()
{
	// Serial.begin(UART_BAUD_RATE);
	crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, FALSE);
	crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
	pinMode(PD0, OUTPUT);
	digitalWrite_LOW(PD0);

	pinMode(LED1_PIN, OUTPUT_OPEN_DRAIN);
	pinMode(LED2_PIN, OUTPUT_OPEN_DRAIN);
	pinMode(LED3_PIN, OUTPUT_OPEN_DRAIN);

	digitalWrite_HIGH(LED1_PIN);
	digitalWrite_HIGH(LED2_PIN);
	digitalWrite_HIGH(LED3_PIN);

	pinMode(MODE_PIN1, INPUT);
	pinMode(MODE_PIN2, INPUT);

	int board_mode = digitalRead(MODE_PIN1) + (digitalRead(MODE_PIN2) << 1);

	// can_config();
	ADC_init();

	auto pwm_driver = new (driver_store.address())  motorlib::at32pwmdriver(1);


	// 电流采样找 0.
	adc_interrupt_enable(ADC1, ADC_CCE_INT, TRUE);
	delay(200);
	adc_interrupt_enable(ADC1, ADC_CCE_INT, FALSE);

	//

	// init USB CDC
	SerialUSB.begin();


	led_status_1(LED1_PIN);

	switch (board_mode)
	{
		case 0:  // DC mode
			// 直流电机模式
			// 打开 直流模式回报和执行代码
			dc_mode_usb_reporter();
			dc_mode_usb_commander(pwm_driver);
			break;

		case 1:// BLDC mode
			// BLDC 模式下，命令等同于 DC 模式，板子会根据霍尔自动运行无刷电机

		case 2: // VFD mode

		case 3: // SVPWM 模式
			svpwm_mode_usb_reporter();
			svpwm_mode_usb_commander(pwm_driver);
			break;
	}

	int oled_delay_ms = 100;

	wdt_enable();

	// app->switch_mode(op_mode::mode_BLDC);
	adc_ordinary_software_trigger_enable(ADC1, TRUE);
}

void loop()
{
	wdt_counter_reload();
	mcucoro::executor::system_executor().poll();
}

extern "C" void ADC1_IRQHandler()
{
    adc_flag_clear(ADC1, ADC_CCE_FLAG);

	// TODO 计算 电流采样 0 点.
}

extern "C" void TMR6_GLOBAL_IRQHandler(void)
{
  /* overflow interrupt management */
  if(tmr_interrupt_flag_get(TMR6, TMR_OVF_FLAG) != RESET)
  {
    /* add user code begin TMR6_TMR_OVF_FLAG */
    /* clear flag */
    tmr_flag_clear(TMR6, TMR_OVF_FLAG);
    /* add user code end TMR6_TMR_OVF_FLAG */

	one_ms_interval.notify();
  }
}

/**
  * @brief  this function handles nmi exception.
  * @param  none
  * @retval none
  */
extern "C" void NMI_Handler(void)
{
}

void system_clock_config(void)
{
  /* reset crm */
  crm_reset();

  /* config flash psr register */
  flash_psr_set(FLASH_WAIT_CYCLE_6);

  /* enable pwc periph clock */
  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);

  /* config ldo voltage */
  pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);

  /* enable lick */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_LICK, TRUE);

  /* wait till lick is ready */
  while(crm_flag_get(CRM_LICK_STABLE_FLAG) != SET)
  {
  }

  /* enable hext */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);

  /* wait till hext is ready */
  while(crm_hext_stable_wait() == ERROR)
  {
  }

  /* enable hick */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);

  /* wait till hick is ready */
  while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET)
  {
  }

  /* config pll clock resource */
  crm_pll_config(CRM_PLL_SOURCE_HEXT, 72, 1, CRM_PLL_FP_4);
  /* config pllu div */
  crm_pllu_div_set(CRM_PLL_FU_18);
  /* enable pll */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  /* wait till pll is ready */
  while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET)
  {
  }

  /* config ahbclk */
  crm_ahb_div_set(CRM_AHB_DIV_1);

  /* config apb2clk, the maximum frequency of APB2 clock is 216 MHz  */
  crm_apb2_div_set(CRM_APB2_DIV_1);

  /* config apb1clk, the maximum frequency of APB1 clock is 120 MHz  */
  crm_apb1_div_set(CRM_APB1_DIV_2);

  /* enable auto step mode */
  crm_auto_step_mode_enable(TRUE);

  /* select pll as system clock source */
  crm_sysclk_switch(CRM_SCLK_PLL);

  /* wait till pll is used as system clock source */
  while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
  {
  }

  /* disable auto step mode */
  crm_auto_step_mode_enable(FALSE);

  /* update system_core_clock global variable */
  system_core_clock_update();

#ifdef AT32F405xx
  /*
    AT32405 OTGHS PHY not initialized, resulting in high power consumption
    Solutions:
    1. If OTGHS is not used, call the "reduce_power_consumption" function to reduce power consumption.
       PLL or HEXT should be enabled when calling this function.
       Example: reduce_power_consumption();

    2. If OTGHS is required, initialize OTGHS to reduce power consumption, without the need to call this function.

       for more detailed information. please refer to the faq document FAQ0148.
  */
#endif

#ifdef AT32F402xx
  /* reduce power comsumption */
  reduce_power_consumption();
#endif
}

#endif
