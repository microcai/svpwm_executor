
#if defined (BOARD_SIMULINK_BOARD)

#include <Arduino.h>
#include <USBSerial.h>

#include "at32f402_405_tmr.h"

#include "dros/mcu_coro.hpp"
#include "dros/coro_condvar.hpp"
#include "dros/delay.hpp"
#include "dros/corothread.hpp"
#include "dros/thread_condvar.hpp"

#include "pins.hpp"

#include "led_status.hpp"

#include "at32pwm.hpp"

#include "mtl.hpp"
#include "vvvf.hpp"
#include "BLDC.hpp"

#include "tmr2_eclipsed_timer.h"

#include "SEGGER_RTT.h"

template<typename T=float>
T clamp(T v, T min, T max)
{
	if (v < min)
		return min;
	else if (v > max)
		return max;
	else
		return v;
}

mtl::static_store<motorlib::at32pwmdriver> driver_store;

static uint16_t ADC_Converted_Data_average[10] = { 0 };
static __IO uint16_t ADC_Converted_Data[10] = { 0 };

#define ADC_IDX_ISENSEA 0
#define ADC_IDX_ISENSEA_REF 1
#define ADC_IDX_ISENSEB 2
#define ADC_IDX_ISENSEB_REF  3

#define ADC_IDX_ISENSEC 4
#define ADC_IDX_ISENSEC_REF  5

#define ADC_IDX_C_OUT 6
#define ADC_IDX_B_OUT 7
#define ADC_IDX_A_OUT 8

#define ADC_IDX_VBUS  9

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
	gpio_initstructure.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_2 | GPIO_PINS_3|GPIO_PINS_4|GPIO_PINS_5;
	gpio_init(GPIOA, &gpio_initstructure);
	gpio_initstructure.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_2 | GPIO_PINS_3;
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
	// ADC_isense abc
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_0, 1, ADC_SAMPLETIME_1_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_1, 2, ADC_SAMPLETIME_1_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_2, 3, ADC_SAMPLETIME_1_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_3, 4, ADC_SAMPLETIME_1_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_4, 5, ADC_SAMPLETIME_1_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_5, 6, ADC_SAMPLETIME_1_5);

	// ADC_vsense abc bus
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_10, 7, ADC_SAMPLETIME_1_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_11, 8, ADC_SAMPLETIME_1_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_12, 9, ADC_SAMPLETIME_1_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_13, 10, ADC_SAMPLETIME_1_5);

	adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_TMR1CH4, TRUE);

	adc_enable(ADC1, TRUE);
	delay_ms(50);
	adc_calibration_init(ADC1);
	while(adc_calibration_init_status_get(ADC1));
	adc_calibration_start(ADC1);
	while(adc_calibration_status_get(ADC1));

	adc_dma_mode_enable(ADC1, TRUE);

	nvic_irq_enable(DMA1_Channel1_IRQn, 1, 1);
	nvic_irq_enable(ADC1_IRQn, 0 , 0);
}

int ttl_counter = 0;
corothread::condition_variable one_ms_interval;
hall_sensor  tmr3_hall;

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

int __IO command_last_received;

void dc_mode_usb_commander(motorlib::pwmdriver* pwm_driver)
{
	struct dc_mode_command_packet{
		uint32_t header; // header must be 'DCDC'
		float duty;
	};

	for(;;)
	{
		corothread::context_yield();

		char input_buffer[64];

		// read data from USB cdc
		auto len = SerialUSB.readBytes(input_buffer, 64);
		if (len == 8)
		{
			auto cmd_pkt = reinterpret_cast<dc_mode_command_packet*>(input_buffer);
			if (cmd_pkt->header == 0x44434443)
			{
				command_last_received = 0;
				auto duty = cmd_pkt->duty;

				// set duty to hardware!
				if (duty == 0)
				{
					pwm_driver->set_duty(-1, -1, -1);
				}
				else if (duty > 0)
				{
					pwm_driver->set_duty(0, -1, std::min(duty, 1.0f));
				}
				else
				{
					pwm_driver->set_duty(std::min(-duty, 1.0f), -1, 0);
				}
			}

		}

		if (command_last_received > 1000)
		{
			pwm_driver->set_duty(-1, -1, -1);
		}
	}
}

float __IO encoder_speed;


void dc_mode_usb_reporter()
{
	corothread::context_yield();

	interval_setup(500);

	struct dc_mode_report_packet
	{
		uint32_t header; // header must be 0xDCDC
		float DC_current;
		float BUS_Voltage;
		float EncoderPos;
		float encoder_speed;
		uint32_t tail;
	};

	dc_mode_report_packet report_packet = {'DCDC', 0, 0, 0};
	report_packet.tail = 0x7F800000;

	for(;;)
	{
		one_ms_interval.wait();
		command_last_received ++;
		report_packet.BUS_Voltage = ADC_Converted_Data[ADC_IDX_VBUS];
		report_packet.DC_current = ADC_Converted_Data[ADC_IDX_ISENSEA] - ADC_Converted_Data_average[ADC_IDX_ISENSEA];
		report_packet.EncoderPos = tmr_counter_value_get(TMR3)/10000.0 * 360.0;
		report_packet.encoder_speed = encoder_speed;

		SerialUSB.write((const uint8_t*) &report_packet, sizeof(report_packet));
	}
}


void vfd_mode_usb_reporter(VVVF* vvvf)
{

	struct vfd_mode_report_packet
	{
		uint32_t header; // header must be 'VVVF'
		float BUS_Voltage;
		float A_current;
		float B_current;
		float C_current;
		float EncoderPos;
		float current_angle_of_output;
		float encoder_speed;
		uint32_t tail;
	};

	vfd_mode_report_packet report_packet = {'FVVV'};
	report_packet.tail = 0x45454545;
	report_packet.tail = 0x7F800000;

	interval_setup(1000);

	for(;;)
	{
		one_ms_interval.wait();

		report_packet.BUS_Voltage = ADC_Converted_Data[ADC_IDX_VBUS];
		float centor = ADC_Converted_Data[ADC_IDX_ISENSEA] + ADC_Converted_Data[ADC_IDX_ISENSEB] + ADC_Converted_Data[ADC_IDX_ISENSEC];
		centor /=3;

		report_packet.A_current = ADC_Converted_Data[ADC_IDX_ISENSEA] - centor;//ADC_Converted_Data_average[ADC_IDX_ISENSEA_REF];
		report_packet.B_current = ADC_Converted_Data[ADC_IDX_ISENSEB] - centor;//ADC_Converted_Data_average[ADC_IDX_ISENSEB_REF];
		report_packet.C_current = ADC_Converted_Data[ADC_IDX_ISENSEC] - centor;//ADC_Converted_Data_average[ADC_IDX_ISENSEC_REF];
		report_packet.current_angle_of_output = vvvf->cur_angle;
		report_packet.EncoderPos = tmr_counter_value_get(TMR3)/10000.0 * 360.0;
		report_packet.encoder_speed = encoder_speed;

		SerialUSB.write((const uint8_t*) &report_packet, sizeof(report_packet));
	}
}

void vfd_mode_usb_commander(VVVF* vvvf)
{
	struct vfd_mode_command_packet{
		uint32_t header; // header must be 'VVVF'
		float freq;
		float voltate;
	};
	for(;;)
	{
		corothread::context_yield();

		char input_buffer[64];

		// read data from USB cdc
		auto len = SerialUSB.readBytes(input_buffer, 64);
		if (len == 12)
		{
			const vfd_mode_command_packet* cmd_pkt = reinterpret_cast<const vfd_mode_command_packet*>(input_buffer);
			if (cmd_pkt->header == 'FVVV')
			{
				togglePin(LED2_PIN);

				vvvf->set_v_and_f(cmd_pkt->voltate, cmd_pkt->freq);
			}
		}
		else if (len == 8)
		{
			const vfd_mode_command_packet* cmd_pkt = reinterpret_cast<const vfd_mode_command_packet*>(input_buffer);
			if (cmd_pkt->header == 0x56560000)
			{
				togglePin(LED2_PIN);

				vvvf->set_v_and_f(vvvf->output_duty, cmd_pkt->freq);
			}
			else if (cmd_pkt->header == 0x46560000)
			{
				togglePin(LED2_PIN);

				vvvf->set_v_and_f(cmd_pkt->freq, vvvf->output_freq);
			}
		}
	}
}

void hall_study(BLDC* bldc)
{
	bldc->direct_control_mode = true;

	int pre_hall_state = -1;

	for (int i = 10; i < 360 ; i+=15)
	{
		corothread::thread_delay(20);
		bldc->set_foc(i, 0.1);
		corothread::thread_delay(10);
		char buf[64];
		int len = snprintf(buf, 64, "angle %d, hall = %d", i, tmr3_hall.hall_state );
		if (pre_hall_state != tmr3_hall.hall_state)
		{
	  		SEGGER_RTT_Write(0, buf, len);
			pre_hall_state = tmr3_hall.hall_state;
			tmr3_hall.update_sector_hall_map(tmr3_hall.hall_state,  i / 60);
		}
	}

	for (int i = 10; i < 360 ; i+=15)
	{
		corothread::thread_delay(20);
		bldc->set_foc(i, 0.1);
		corothread::thread_delay(10);
		char buf[64];
		int len = snprintf(buf, 64, "angle %d, hall = %d", i, tmr3_hall.hall_state );
		if (pre_hall_state != tmr3_hall.hall_state)
		{
	  		SEGGER_RTT_Write(0, buf, len);
			pre_hall_state = tmr3_hall.hall_state;
			tmr3_hall.update_sector_hall_map(tmr3_hall.hall_state,  i / 60);
		}
	}

	bldc->set_duty(0);
	corothread::thread_delay(200);
	bldc->set_duty(0.1);
	return;
}

void svpwm_mode_usb_commander(BLDC* bldc)
{
	corothread::thread_delay(200);

	struct svpwm_mode_command_packet{
		uint32_t header; // header must be 'BLDC'
		float dutyA;
		float dutyB;
		float dutyC;
	};

	struct bldc_mode_command_packet{
		uint32_t header; // header must be 'BLDC'
		float duty;
	};

	for(;;)
	{
		corothread::context_yield();

		char input_buffer[64];

		// read data from USB cdc
		auto len = SerialUSB.readBytes(input_buffer, 64);
		if (len == 16)
		{
			const svpwm_mode_command_packet* cmd_pkt = reinterpret_cast<const svpwm_mode_command_packet*>(input_buffer);
			if (cmd_pkt->header == 'SPWM')
			{
				togglePin(LED2_PIN);
				bldc->set_duty(cmd_pkt->dutyA, cmd_pkt->dutyB, cmd_pkt->dutyC);
				command_last_received = 0;
			}
		}
		else if (len == 8)
		{
			const bldc_mode_command_packet* cmd_pkt = reinterpret_cast<const bldc_mode_command_packet*>(input_buffer);
			if (cmd_pkt->header == 0x424c4443)
			{
				command_last_received = 0;
				togglePin(LED2_PIN);
				bldc->set_duty(clamp(cmd_pkt->duty, -1.0f, 1.0f));
			}
		}
		
		if (command_last_received > 1000)
		{
			bldc->set_duty(-1.0f,-1.0f, -1.0f);
		}			
	}
}

void svpwm_mode_usb_reporter(BLDC* bldc)
{
	struct svpwm_mode_report_packet
	{
		uint32_t header; // header must be 'BLDC'
		float BUS_Voltage;
		float A_current;
		float B_current;
		float C_current;
		float pos_by_hall;
		float erpm;
		uint32_t tail;
	};

	svpwm_mode_report_packet report_packet = {'BLDC'};
	report_packet.tail = 0x7F800000;

	interval_setup(500);

	for(;;)
	{
		one_ms_interval.wait();
		ttl_counter ++;
		command_last_received ++;
		report_packet.BUS_Voltage = ADC_Converted_Data[ADC_IDX_VBUS];
		float centor = ADC_Converted_Data[ADC_IDX_ISENSEA] + ADC_Converted_Data[ADC_IDX_ISENSEB] + ADC_Converted_Data[ADC_IDX_ISENSEC];
		centor /=3;

		report_packet.A_current = ADC_Converted_Data[ADC_IDX_ISENSEA] - centor;//ADC_Converted_Data_average[ADC_IDX_ISENSEA_REF];
		report_packet.B_current = ADC_Converted_Data[ADC_IDX_ISENSEB] - centor;//ADC_Converted_Data_average[ADC_IDX_ISENSEB_REF];
		report_packet.C_current = ADC_Converted_Data[ADC_IDX_ISENSEC] - centor;//ADC_Converted_Data_average[ADC_IDX_ISENSEC_REF];

		report_packet.pos_by_hall = tmr3_hall.get_sector() * 60 + 30;
		report_packet.erpm = tmr3_hall.erpm;

		if (ttl_counter >  1200)
		{
			// get_eclipsed_and_reset();
			tmr3_hall.erpm = 0;
		}
		else if (ttl_counter >  300)
		{
			// get_eclipsed_and_reset();
			tmr3_hall.erpm = tmr3_hall.erpm * 0.99;
		}

		SerialUSB.write((const uint8_t*) &report_packet, sizeof(report_packet));		
	}
}

static __IO uint32_t ADC_Converted_Data_sum[8] = { 0 };
static __IO uint32_t ADC_Convert_count = 0;

static corothread::thread_context reporter_thread_ctx;
static corothread::thread_context commander_thread_ctx;


void hall_tmr3_init();
void encoder_tmr3_init(void);

extern "C" void EXINT9_5_IRQHandler()
{
	if(exint_interrupt_flag_get(EXINT_LINE_8) != RESET)
	{
		tmr_counter_value_set(TMR3, 0);
	}

	exint_flag_clear(EXINT_LINE_5|EXINT_LINE_6|EXINT_LINE_7|EXINT_LINE_8|EXINT_LINE_9);
}

void setup()
{
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);
  	SEGGER_RTT_WriteString(0, "SEGGER Real-Time-Terminal Started\r\n\r\n");
	// Serial.begin(UART_BAUD_RATE);
	crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);

	scfg_exint_line_config(SCFG_PORT_SOURCE_GPIOC, SCFG_PINS_SOURCE8);

	exint_init_type exint_init_struct;

	exint_init_struct.line_enable = TRUE;
	exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
	exint_init_struct.line_select = EXINT_LINE_8;
	exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;

	pinMode(LED1_PIN, OUTPUT_OPEN_DRAIN);
	pinMode(LED2_PIN, OUTPUT_OPEN_DRAIN);
	pinMode(LED3_PIN, OUTPUT_OPEN_DRAIN);
	pinMode(GATE_EN_PIN, OUTPUT);
	digitalWrite_LOW(GATE_EN_PIN);

	digitalWrite_HIGH(LED1_PIN);
	digitalWrite_HIGH(LED2_PIN);
	digitalWrite_HIGH(LED3_PIN);

	// can_config();
	ADC_init();

	dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);

	auto pwm_driver = new (driver_store.address())  motorlib::at32pwmdriver();

	pwm_driver->start();

	// 电流采样找 0.
	delay(150);
	dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, FALSE);
	delay(1);

	for (int i=0; i < sizeof(ADC_Converted_Data)/sizeof(ADC_Converted_Data[0]) ; i++)
	{
		ADC_Converted_Data_average[i] = ADC_Converted_Data_sum[i]/(ADC_Convert_count - 5000);
	}

	// init USB CDC
	SerialUSB.begin();

	digitalWrite_HIGH(GATE_EN_PIN);

	led_status_1(LED1_PIN);

	// BLDC

	#if BOARD_MODE == 1
		encoder_tmr3_init();
		pinMode(PC8, INPUT);
		exint_init(&exint_init_struct);
		nvic_irq_enable(EXINT9_5_IRQn, 0, 1);
		VVVF * vvvf = new VVVF(pwm_driver);
		corothread::create_static_context(&reporter_thread_ctx, callable(&vfd_mode_usb_reporter, vvvf));
		corothread::create_static_context(&commander_thread_ctx, callable(&vfd_mode_usb_commander, vvvf));
	#elif BOARD_MODE == 0
		encoder_tmr3_init();
		pinMode(PC8, INPUT);
		exint_init(&exint_init_struct);
		//VVVF 直流电机模式
		// 打开 直流模式回报和执行代码
		nvic_irq_enable(EXINT9_5_IRQn, 0, 1);
		corothread::create_static_context(&reporter_thread_ctx, callable(&dc_mode_usb_reporter));
		corothread::create_static_context(&commander_thread_ctx, callable(&dc_mode_usb_commander, pwm_driver));
	#elif BOARD_MODE == 2
		tmr2_eclipse_timer_init();
		hall_tmr3_init();
		tmr3_hall.hal_irq_handle(get_eclipsed_and_reset());
		BLDC * bldc = new BLDC(pwm_driver, &tmr3_hall);
		corothread::create_static_context(&reporter_thread_ctx, callable(&svpwm_mode_usb_reporter, bldc));
		corothread::create_static_context(&commander_thread_ctx, callable(&svpwm_mode_usb_commander, bldc));
	#endif
	// app->switch_mode(op_mode::mode_BLDC);
	adc_ordinary_software_trigger_enable(ADC1, TRUE);
}

void loop()
{
	wdt_counter_reload();
	mcucoro::executor::system_executor().poll();
}

extern "C" void DMA1_Channel1_IRQHandler(void)
{
	if(dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET)
	{
		dma_flag_clear(DMA1_FDT1_FLAG);

		if (ADC_Convert_count >=5000)
		{
			// 累加 ADC 结果.
			for (int i=0; i < sizeof(ADC_Converted_Data)/sizeof(ADC_Converted_Data[0]) ; i++)
			{
				ADC_Converted_Data_sum[i] += ADC_Converted_Data[i];
			}
		}

		++ADC_Convert_count;
	}
}

template<int full_angle>
int angle_diff(int cur_angle, int pre_angle)
{
	auto guess1 = cur_angle - pre_angle;
	auto guess2 = cur_angle + full_angle - pre_angle;

	int diff1, diff2;
	bool cur_bigger = cur_angle >= pre_angle;
	diff1 = cur_angle - pre_angle;
	diff2 = pre_angle - cur_angle;

	if (cur_bigger)
	{
		if (diff1 > (full_angle/2))
		{
			return diff1 - full_angle;
		}
		return diff1;
	}
	else
	{
		if (diff2 > (full_angle/2))
		{
			return full_angle - diff2;
		}
		return diff1;
	}
}

extern "C" void TMR3_GLOBAL_IRQHandler()
{
  if(tmr_interrupt_flag_get(TMR3, TMR_TRIGGER_FLAG) == SET)
  {
	ttl_counter = 0;
	tmr3_hall.hal_irq_handle(get_eclipsed_and_reset());
    tmr_flag_clear(TMR3, TMR_TRIGGER_FLAG);
  }
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

	// caculate RPM by using TMR3
	static int pre_value = 0;
	int value = tmr_counter_value_get(TMR3);

	// 首先判定旋转方向.
	int delta_angle = angle_diff<10000>(value, pre_value);
	encoder_speed = delta_angle / 10000.0 * 60 * 500;

	pre_value = value;

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


bool at32_board_specific_tmr_gpio_setup()
{
	gpio_init_type gpio_init_struct;
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);

	gpio_default_para_init(&gpio_init_struct);

	/* add user code begin tmr1_init 1 */

	/* add user code end tmr1_init 1 */

	/* configure the tmr1 CH1 pin */
	gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_1);
	gpio_init_struct.gpio_pins = GPIO_PINS_8;
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOA, &gpio_init_struct);

	/* configure the tmr1 CH1C pin */
	gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE7, GPIO_MUX_1);
	gpio_init_struct.gpio_pins = GPIO_PINS_7;
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOA, &gpio_init_struct);

	/* configure the tmr1 CH2 pin */
	gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE9, GPIO_MUX_1);
	gpio_init_struct.gpio_pins = GPIO_PINS_9;
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOA, &gpio_init_struct);

	/* configure the tmr1 CH2C pin */
	gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE0, GPIO_MUX_1);
	gpio_init_struct.gpio_pins = GPIO_PINS_0;
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOB, &gpio_init_struct);

	/* configure the tmr1 CH3 pin */
	gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE10, GPIO_MUX_1);
	gpio_init_struct.gpio_pins = GPIO_PINS_10;
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOA, &gpio_init_struct);

	/* configure the tmr1 CH3C pin */
	gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE1, GPIO_MUX_1);
	gpio_init_struct.gpio_pins = GPIO_PINS_1;
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOB, &gpio_init_struct);

	/* configure the tmr1 BRK pin */
	gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE12, GPIO_MUX_1);
	gpio_init_struct.gpio_pins = GPIO_PINS_12;
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOB, &gpio_init_struct);

	return true;
}

void usb_delay_ms(uint32_t ms)
{
	corothread::thread_delay(ms);
}

#endif

