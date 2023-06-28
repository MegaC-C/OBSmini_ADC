// This code is written for the OBSmini with the aim to visualize the recieved echo signals as measured by the nRF52833. 
// The results can be used to tune limits/timings etc.
//
// Most code comes from official examples:
// NFC/Powermanagement: nrf/samples/nfc/system_off
// SAADC/PPI: https://github.com/NordicPlayground/nRF52-ADC-examples/tree/master/nrfx_saadc_multi_channel_ppi
// BLE: webinar: Developing Bluetooth Low Energy products using nRF Connect SDK
// PWM: nRF5_SDKv17.0.2/examples/peripheral/pwm_driver
// DFU_OTA: https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/working_with_nrf/nrf52/developing.html#fota-updates
// with DFU_OTA: the "merged.hex" is used to drag´n´drop flash (doesn't work at first flash, any "zephyr.bin" must be flashed first) and the "app_update.bin" is used for DFU.
// without DFU_OTA: the "zephyr.bin" is used to drag´n´drop flash (all these files are found in ...\myProjectFolder\build\zephyr)
// MCUBoot: https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/app_dev/bootloaders_and_dfu/index.html
//		and https://github.com/hellesvik-nordic/samples_for_nrf_connect_sdk/tree/1111836cd720127c7f2b0dc0bec9f7ef496b8954/bootloader_samples
//
// in general when starting from scratch only 4 files are touched: main.c, CMakeLists.txt, prj.conf and a new custom key in /custom_key_dir needs to be added
// the rest (build configurations, devicetree, ncs root files, etc.) are untouched (except for missed ncs/zephyr pull requests which are added manually, should be obsolete in future)
// https://github.com/zephyrproject-rtos/zephyr/pull/57886/commits/15e7ab19fc0b8d51942c3cca5d1b13df0ebdbec0
// https://github.com/zephyrproject-rtos/zephyr/pull/56309/commits/2094e19a3c58297125c1289ea0ddec89db317f96
//
//
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
// #include <zephyr/mgmt/mcumgr/transport/smp_bt.h>

#include <nfc_t2t_lib.h>
#include <nfc/ndef/msg.h>
#include <nfc/ndef/text_rec.h>

#include <hal/nrf_gpio.h>
#include <hal/nrf_power.h>

#include <nrfx_ppi.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <nrfx_pwm.h>
#include <nrfx_wdt.h>

#include <helpers/nrfx_reset_reason.h>


// forward declarations	------------------------------------------------------------------------------------------------------------------------
void ble_connected_handler(struct bt_conn *conn, uint8_t err);
void ble_disconnected_handler(struct bt_conn *conn, uint8_t reason);
void ble_chrc_ccc_cfg_changed_handler(const struct bt_gatt_attr *attr, uint16_t value);
void error_handling();


#define NRFX_ERR_CHECK(nrfx_err, msg); 	if (nrfx_err != NRFX_SUCCESS) {LOG_ERR(msg " - %d", nrfx_err); error_handling();}
#define ERR_CHECK(err, msg); 			if (err) {LOG_ERR(msg " - %d", err); error_handling();}
#define NRFX_PWM_VALUES_LENGTH(array)	(sizeof(array) / (sizeof(uint16_t)))

#define BT_UUID_REMOTE_SERV_VAL			BT_UUID_128_ENCODE(0xb088b5a1, 0x08fe, 0x46f2, 0xafaa, 0xc1c69c8917659)
#define BT_UUID_REMOTE_SERVICE  		BT_UUID_DECLARE_128(BT_UUID_REMOTE_SERV_VAL)

#define BT_UUID_REMOTE_BUTTON_CHRC_VAL	BT_UUID_128_ENCODE(0xb088b5a2, 0x08fe, 0x46f2, 0xafaa, 0xc1c69c8917659)	
#define BT_UUID_REMOTE_BUTTON_CHRC 		BT_UUID_DECLARE_128(BT_UUID_REMOTE_BUTTON_CHRC_VAL)

#define BLE_DEVICE_NAME 				CONFIG_BT_DEVICE_NAME
#define BLE_DEVICE_NAME_LEN 			(sizeof(BLE_DEVICE_NAME)-1)

#define BLUE_LED						NRF_GPIO_PIN_MAP(0, 17)
#define RED_LED							NRF_GPIO_PIN_MAP(0, 20)
#define SYSTEM_ON_LED					BLUE_LED
#define BLE_CONNECTED_LED				BLUE_LED
#define NFC_FIELD_LED		            RED_LED
#define ERROR_LED						RED_LED
#define TRAFO_L1						NRF_GPIO_PIN_MAP(0, 4)
#define TRAFO_L2						NRF_GPIO_PIN_MAP(0, 5)
#define TRAFO_R1						NRF_GPIO_PIN_MAP(0, 11)
#define TRAFO_R2						NRF_GPIO_PIN_MAP(1, 9)
#define OPAMPS_ON_OFF					NRF_GPIO_PIN_MAP(0, 15)

#define SAADC_BUF_SIZE					8000
#define SAADC_SAMPLINGRATE_US 			5	    // sample every 5 µs to get the max possible 200 kHz SAADC
#define MAX_REC_COUNT		            1
#define NDEF_MSG_BUF_SIZE	            128
#define PWM_MAX	                        25   	// must be lower than (2^16)/2 = 32768, 1 as MSB is the problem
#define TIME_TO_DEEPSLEEP_S            	30
#define WDT_TIME_TO_RESET_MS			300000	// 5min = 5*60000ms


// pre kernel initialization ------------------------------------------------------------------------------------------------------------------------
LOG_MODULE_REGISTER(logging, LOG_LEVEL_DBG);
int nrfx_err = NRFX_SUCCESS;
int err = 0;

// Prevent deep sleep (system off) from being entered on long timeouts
// or `K_FOREVER` due to the default residency policy.
// This has to be done before anything tries to sleep, which means
// before the threading system starts up between PRE_KERNEL_2 and
// POST_KERNEL.  Do it at the start of PRE_KERNEL_2.
int prevent_system_off(void) {
	pm_policy_state_lock_get(PM_STATE_SOFT_OFF, PM_ALL_SUBSTATES);
    return 0;
}
SYS_INIT(prevent_system_off, PRE_KERNEL_2, 0);

// if GPIO output voltage (REGOUT0) is still set to default '111' (1.8 volts) increase GPIO voltage to '100' (3.0 volts) to drive mosfets properly.
// it's in non-volatile register so we must use NVMC. Only bit flips from '1' to '0' are possible without debugger.
// this is copy paste from "zephyr/boards/arm/nrf52840dongle_nrf52840/board.c"
int set_REGOUT0_to_3V0(void) {
	if ((nrf_power_mainregstatus_get(NRF_POWER) == NRF_POWER_MAINREGSTATUS_HIGH) && 
        ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) == (UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos))) 
    {
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
		NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) | (UICR_REGOUT0_VOUT_3V0 << UICR_REGOUT0_VOUT_Pos);
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
		/* a reset is required for changes to take effect */
		NVIC_SystemReset();
	}
	return 0;
}
SYS_INIT(set_REGOUT0_to_3V0, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);


// BLE ------------------------------------------------------------------------------------------------------------------------
BT_GATT_SERVICE_DEFINE(remote_srv,
					BT_GATT_PRIMARY_SERVICE(BT_UUID_REMOTE_SERVICE),
    				BT_GATT_CHARACTERISTIC(BT_UUID_REMOTE_BUTTON_CHRC,
                    BT_GATT_CHRC_NOTIFY,
                    BT_GATT_PERM_READ,
                    NULL, NULL, NULL),
    				BT_GATT_CCC(ble_chrc_ccc_cfg_changed_handler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));	

struct bt_conn *current_conn;

const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, BLE_DEVICE_NAME, BLE_DEVICE_NAME_LEN)};

struct bt_conn_cb bluetooth_callbacks = {
	.connected 		= ble_connected_handler,
	.disconnected 	= ble_disconnected_handler};

const int8_t ble_even_flag[2] 	= {0b00000000, 0b10000000};
const int8_t ble_uneven_flag[2] = {0b00000001, 0b10000000};
int8_t ble_send_array[2];
bool ble_notif_enabled = false;


// SAADC ------------------------------------------------------------------------------------------------------------------------
nrfx_saadc_adv_config_t saadc_peripheral_config = {                                                      
        .oversampling      = NRF_SAADC_OVERSAMPLE_DISABLED,
        .burst             = NRF_SAADC_BURST_DISABLED,     
        .internal_timer_cc = 0,                            
        .start_on_end      = true};

nrfx_saadc_channel_t saadc_channel_0_config = {                                                  
    .channel_config = {                                              
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED, 
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED, 
        .gain       = NRF_SAADC_GAIN1_2,           
        .reference  = NRF_SAADC_REFERENCE_VDD4,
        .acq_time   = NRF_SAADC_ACQTIME_3US,      
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED, 
        .burst      = NRF_SAADC_BURST_DISABLED},                                             
    .pin_p          = (nrf_saadc_input_t)NRF_SAADC_INPUT_AIN1,   
    .pin_n          = NRF_SAADC_INPUT_DISABLED,    
    .channel_index  = 0};

nrfx_saadc_channel_t saadc_channel_1_config = {                                                  
    .channel_config = {                                              
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED, 
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED, 
        .gain       = NRF_SAADC_GAIN1_2,           
        .reference  = NRF_SAADC_REFERENCE_VDD4,
        .acq_time   = NRF_SAADC_ACQTIME_3US,      
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED, 
        .burst      = NRF_SAADC_BURST_DISABLED},                                             
    .pin_p          = (nrf_saadc_input_t)NRF_SAADC_INPUT_AIN4,   
    .pin_n          = NRF_SAADC_INPUT_DISABLED,    
    .channel_index  = 1};

nrf_saadc_value_t saadc_samples[SAADC_BUF_SIZE];
bool saadc_buffer_is_full = false;


// TIMER/PPI ------------------------------------------------------------------------------------------------------------------------
const nrfx_timer_t timer_to_sample_saadc = NRFX_TIMER_INSTANCE(1);

nrfx_timer_config_t timer_config = {                                                                
        .frequency          = NRF_TIMER_FREQ_1MHz,                  
        .mode               = NRF_TIMER_MODE_TIMER,                  
        .bit_width          = NRF_TIMER_BIT_WIDTH_8,                
        .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
        .p_context          = NULL};
		
nrf_ppi_channel_t timer_to_saadc_ppi_channel;


// PWM ------------------------------------------------------------------------------------------------------------------------
nrfx_pwm_t my_pwm = NRFX_PWM_INSTANCE(0);  

nrfx_pwm_config_t const pwm_peripheral_config = {
	.output_pins = {
	    TRAFO_L1,
        TRAFO_R1,
        TRAFO_L2,
        TRAFO_R2},
	.irq_priority 	= NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,
	.base_clock   	= NRF_PWM_CLK_2MHz,       
	.count_mode   	= NRF_PWM_MODE_UP,
	.top_value    	= PWM_MAX,                     
	.load_mode    	= NRF_PWM_LOAD_GROUPED,
	.step_mode    	= NRF_PWM_STEP_AUTO,
	.skip_gpio_cfg	= false};
// TRAFO_L1  NRFX_PWM_PIN_NOT_USED 
// TRAFO_R1  NRFX_PWM_PIN_NOT_USED
// TRAFO_L2  NRFX_PWM_PIN_NOT_USED
// TRAFO_R2  NRFX_PWM_PIN_NOT_USED

// nrf_pwm_values_grouped_t arrays cannot be allocated on stack (hence "static")  
// and they must be in RAM (hence no "const", though its content is not changed).
// the arrays are subsequently used to generate a pwm sequence, but we only use 0
// and the value of nrfx_pwm_config_t.top_value so we aren't really using the pwm 
// functionality but rather use 0% and 100% PWM to generate a given amount of ON/OFF 
// pulses. With the ON/OFF time being dependent on the nrfx_pwm_config_t.base_clock
// and .top_value. How the output pins are connected to the elements of
// nrf_pwm_values_grouped_t arrays is defined by .load_mode 
static nrf_pwm_values_grouped_t /*const*/ pwm_4plus1pulses[] = {
    {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX},
	{PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {PWM_MAX, 0}, {0, PWM_MAX}};

static nrf_pwm_values_grouped_t /*const*/ pwm_5pulses[] = {        
    {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX},
    {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}};

static nrf_pwm_values_grouped_t /*const*/ pwm_19plus1pulses[] = { 
	{0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX},
    {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0},  
	{0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX},
    {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0},     
    {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX},
    {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0},
	{0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX},
    {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {PWM_MAX, 0}, {0, PWM_MAX}};

static nrf_pwm_values_grouped_t /*const*/ pwm_20pulses[] = { 
	{0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX},
    {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0},  
	{0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX},
    {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0},     
    {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX},
    {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0},
	{0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX},
    {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}, {0, PWM_MAX}, {PWM_MAX, 0}};

const nrf_pwm_sequence_t pwm_4plus1pulses_seq = {
    .values.p_grouped = pwm_4plus1pulses,
    .length           = NRFX_PWM_VALUES_LENGTH(pwm_4plus1pulses),
    .repeats          = 0,
    .end_delay        = 0};

const nrf_pwm_sequence_t pwm_5pulses_seq = {
    .values.p_grouped = pwm_5pulses,
    .length           = NRFX_PWM_VALUES_LENGTH(pwm_5pulses),
    .repeats          = 0,
    .end_delay        = 0};

const nrf_pwm_sequence_t pwm_19plus1pulses_seq = {
    .values.p_grouped = pwm_19plus1pulses,
    .length           = NRFX_PWM_VALUES_LENGTH(pwm_19plus1pulses),
    .repeats          = 0,
    .end_delay        = 0};

const nrf_pwm_sequence_t pwm_20pulses_seq = {
    .values.p_grouped = pwm_20pulses,
    .length           = NRFX_PWM_VALUES_LENGTH(pwm_20pulses),
    .repeats          = 0,
    .end_delay        = 0};


// NFC ------------------------------------------------------------------------------------------------------------------------
// text messages and language code
const uint8_t pulses_4_payload[] = {'4', '+', '1', ' ', 'p', 'u', 'l', 's', 'e', 's'};
const uint8_t pulses_5_payload[] = {'5', ' ', 'p', 'u', 'l', 's', 'e', 's'};
const uint8_t pulses_19_payload[] = {'1', '9', '+', '1',' ', 'p', 'u', 'l', 's', 'e', 's'};
const uint8_t pulses_20_payload[] = {'2', '0', ' ', 'p', 'u', 'l', 's', 'e', 's'};
const uint8_t en_code[] = {'e', 'n'};

// buffers used to hold NFC NDEF messages
uint8_t pulses_4_buffer[NDEF_MSG_BUF_SIZE];
uint8_t pulses_5_buffer[NDEF_MSG_BUF_SIZE];
uint8_t pulses_19_buffer[NDEF_MSG_BUF_SIZE];
uint8_t pulses_20_buffer[NDEF_MSG_BUF_SIZE];

uint32_t pulses_4_buffer_length = sizeof(pulses_4_buffer);
uint32_t pulses_5_buffer_length = sizeof(pulses_5_buffer);
uint32_t pulses_19_buffer_length = sizeof(pulses_19_buffer);
uint32_t pulses_20_buffer_length = sizeof(pulses_20_buffer);

uint8_t pulse_number = 4;
bool nfc_detected = false;


// WDT ------------------------------------------------------------------------------------------------------------------------
const nrfx_wdt_t wdt_instance = NRFX_WDT_INSTANCE(0);
nrfx_wdt_channel_id wdt_channel_0;

nrfx_wdt_config_t wdt_config_normal = {
	.behaviour 			= NRF_WDT_BEHAVIOUR_PAUSE_SLEEP_HALT,
	.reload_value 		= WDT_TIME_TO_RESET_MS,
	.interrupt_priority = NRFX_WDT_DEFAULT_CONFIG_IRQ_PRIORITY};


// interrupt handlers ------------------------------------------------------------------------------------------------------------------------
void ble_connected_handler(struct bt_conn *conn, uint8_t err) {
	if (err) {
		LOG_ERR("connection err: %d", err);
		return;
	}
	LOG_INF("Connected.");
	current_conn = bt_conn_ref(conn);
	nrf_gpio_pin_clear(BLE_CONNECTED_LED);
    }

void ble_disconnected_handler(struct bt_conn *conn, uint8_t reason) {
	LOG_INF("Disconnected (reason: %d)", reason);
	nrf_gpio_pin_set(BLE_CONNECTED_LED);
    if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

void ble_chrc_ccc_cfg_changed_handler(const struct bt_gatt_attr *attr, uint16_t value) {
    ble_notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notifications %s", ble_notif_enabled? "enabled":"disabled");
}

void saadc_handler(nrfx_saadc_evt_t const * p_event) {
    switch (p_event->type) {
		case NRFX_SAADC_EVT_DONE:
			saadc_buffer_is_full = true;
			nrfx_timer_disable(&timer_to_sample_saadc);	// should be called in NRFX_SAADC_EVT_FINISHED, but that event was never generated,
			break;                                  // perhaps because one dimensional buffer was used instead of double buffer?
		case NRFX_SAADC_EVT_BUF_REQ:
			nrfx_err = nrfx_saadc_buffer_set(&saadc_samples[0], SAADC_BUF_SIZE);
			NRFX_ERR_CHECK(nrfx_err, " failed");
			break;
		case NRFX_SAADC_EVT_READY:
			nrfx_timer_enable(&timer_to_sample_saadc);
			break;
        default:
            LOG_INF("SAADC evt %d", p_event->type);
            break;
    }
}

void nfc_handler(void *context,
			 	 nfc_t2t_event_t event,
				 const uint8_t *data,
			 	 size_t data_length) {
	ARG_UNUSED(context);
	ARG_UNUSED(data);
	ARG_UNUSED(data_length);

	switch (event) {
	case NFC_T2T_EVENT_DATA_READ:
		err = nfc_t2t_emulation_stop();	// NFC emulation must be stopped to change payload
		ERR_CHECK(err, "Cannot stop NFC emulation!");
		switch (pulse_number) {
		case 4:
			pulse_number = 5;
			nrfx_pwm_simple_playback(&my_pwm, &pwm_5pulses_seq, 1, NRFX_PWM_FLAG_STOP);
			err = nfc_t2t_payload_set(pulses_19_buffer, pulses_19_buffer_length);
			ERR_CHECK(err, "Cannot set NFC payload!");
			break;		
		case 5:
			pulse_number = 19;
			nrfx_pwm_simple_playback(&my_pwm, &pwm_19plus1pulses_seq, 1, NRFX_PWM_FLAG_STOP);
			err = nfc_t2t_payload_set(pulses_20_buffer, pulses_20_buffer_length);
			ERR_CHECK(err, "Cannot set NFC payload!");
			break;
		case 19:
			pulse_number = 20;
			nrfx_pwm_simple_playback(&my_pwm, &pwm_20pulses_seq, 1, NRFX_PWM_FLAG_STOP);
			err = nfc_t2t_payload_set(pulses_4_buffer, pulses_4_buffer_length);
			ERR_CHECK(err, "Cannot set NFC payload!");
			break;
		case 20:
			pulse_number = 4;
			nrfx_pwm_simple_playback(&my_pwm, &pwm_4plus1pulses_seq, 1, NRFX_PWM_FLAG_STOP);
			err = nfc_t2t_payload_set(pulses_5_buffer, pulses_5_buffer_length);
			ERR_CHECK(err, "Cannot set NFC payload!");
			break;
		default:
			break;
		}
		err = nfc_t2t_emulation_start();
		ERR_CHECK(err, "Cannot start NFC emulation!");
		nfc_detected = true;
		nrf_gpio_pin_clear(NFC_FIELD_LED);
		LOG_INF("pulse_number = %d", pulse_number);
		break;
	case NFC_T2T_EVENT_FIELD_OFF:
		nfc_detected = false;
		nrf_gpio_pin_set(NFC_FIELD_LED);
		break;
	default:
		break;
	}
}

void wdt_handler(void) {
	// only two 32kHz ticks are left before WDT reset happens
	nrf_gpio_pin_set(OPAMPS_ON_OFF);
	nrf_gpio_pin_clear(RED_LED);
}


// functions ------------------------------------------------------------------------------------------------------------------------
void timer_init(void) {
    nrfx_err = nrfx_timer_init(&timer_to_sample_saadc, &timer_config, NULL);
    NRFX_ERR_CHECK(nrfx_err, " failed");

    nrfx_timer_extended_compare(&timer_to_sample_saadc,
                                NRF_TIMER_CC_CHANNEL0,
                                nrfx_timer_us_to_ticks(&timer_to_sample_saadc, SAADC_SAMPLINGRATE_US),
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                false);
}

void ppi_init(void) {
    nrfx_err = nrfx_ppi_channel_alloc(&timer_to_saadc_ppi_channel);
    NRFX_ERR_CHECK(nrfx_err, " failed");

    nrfx_err = nrfx_ppi_channel_assign(timer_to_saadc_ppi_channel, 
                                       nrfx_timer_event_address_get(&timer_to_sample_saadc, NRF_TIMER_EVENT_COMPARE0),
                                       nrf_saadc_task_address_get((NRF_SAADC_Type * const) NRF_SAADC_BASE, NRF_SAADC_TASK_SAMPLE));
    NRFX_ERR_CHECK(nrfx_err, " failed");

    nrfx_err = nrfx_ppi_channel_enable(timer_to_saadc_ppi_channel);
    NRFX_ERR_CHECK(nrfx_err, " failed");
}

void saadc_init(void) {
    nrfx_err = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
    NRFX_ERR_CHECK(nrfx_err, " failed");

	nrfx_err = nrfx_saadc_offset_calibrate(NULL);
	NRFX_ERR_CHECK(nrfx_err, "Cannot calibrate SAADC")

    nrfx_err = nrfx_saadc_channel_config(&saadc_channel_0_config);
    nrfx_err = nrfx_saadc_channel_config(&saadc_channel_1_config);
    NRFX_ERR_CHECK(nrfx_err, " failed");

    nrfx_err = nrfx_saadc_advanced_mode_set(nrfx_saadc_channels_configured_get(),
                                            NRF_SAADC_RESOLUTION_12BIT,
                                            &saadc_peripheral_config,
                                            saadc_handler);
    NRFX_ERR_CHECK(nrfx_err, " failed");
                                            
    nrfx_err = nrfx_saadc_buffer_set(&saadc_samples[0], SAADC_BUF_SIZE);
    NRFX_ERR_CHECK(nrfx_err, " failed");

    nrfx_err = nrfx_saadc_mode_trigger();
    NRFX_ERR_CHECK(nrfx_err, " failed");
}

void nfc_init(void) {
	NFC_NDEF_TEXT_RECORD_DESC_DEF(nfc_pulses_4_text_rec,
				      			  UTF_8,
				      			  en_code,
				      			  sizeof(en_code),
				      			  pulses_4_payload,
				      			  sizeof(pulses_4_payload));

	NFC_NDEF_TEXT_RECORD_DESC_DEF(nfc_pulses_5_text_rec,
				      			  UTF_8,
				      			  en_code,
				      			  sizeof(en_code),
				      			  pulses_5_payload,
				      			  sizeof(pulses_5_payload));

	NFC_NDEF_TEXT_RECORD_DESC_DEF(nfc_pulses_19_text_rec,
				     			  UTF_8,
				     			  en_code,
				     			  sizeof(en_code),
				     			  pulses_19_payload,
				     			  sizeof(pulses_19_payload));

	NFC_NDEF_TEXT_RECORD_DESC_DEF(nfc_pulses_20_text_rec,
				      			  UTF_8,
				      			  en_code,
				      			  sizeof(en_code),
				      			  pulses_20_payload,
				      			  sizeof(pulses_20_payload));

	NFC_NDEF_MSG_DEF(nfc_pulses_4_text_msg, MAX_REC_COUNT);
	NFC_NDEF_MSG_DEF(nfc_pulses_5_text_msg, MAX_REC_COUNT);
	NFC_NDEF_MSG_DEF(nfc_pulses_19_text_msg, MAX_REC_COUNT);
	NFC_NDEF_MSG_DEF(nfc_pulses_20_text_msg, MAX_REC_COUNT);

	err = nfc_t2t_setup(nfc_handler, NULL);
	ERR_CHECK(err, "Cannot setup NFC T2T library!");

	err = nfc_ndef_msg_record_add(&NFC_NDEF_MSG(nfc_pulses_4_text_msg), &NFC_NDEF_TEXT_RECORD_DESC(nfc_pulses_4_text_rec));
	ERR_CHECK(err, "Cannot add NFC record!");
	err = nfc_ndef_msg_record_add(&NFC_NDEF_MSG(nfc_pulses_5_text_msg), &NFC_NDEF_TEXT_RECORD_DESC(nfc_pulses_5_text_rec));
	ERR_CHECK(err, "Cannot add NFC record!");
	err = nfc_ndef_msg_record_add(&NFC_NDEF_MSG(nfc_pulses_19_text_msg), &NFC_NDEF_TEXT_RECORD_DESC(nfc_pulses_19_text_rec));
	ERR_CHECK(err, "Cannot add NFC record!");
	err = nfc_ndef_msg_record_add(&NFC_NDEF_MSG(nfc_pulses_20_text_msg), &NFC_NDEF_TEXT_RECORD_DESC(nfc_pulses_20_text_rec));
	ERR_CHECK(err, "Cannot add NFC record!");

	err = nfc_ndef_msg_encode(&NFC_NDEF_MSG(nfc_pulses_4_text_msg),
				            pulses_4_buffer, 
                            &pulses_4_buffer_length);
	ERR_CHECK(err, "Cannot encode NFC message!");
	err = nfc_ndef_msg_encode(&NFC_NDEF_MSG(nfc_pulses_5_text_msg),
				            pulses_5_buffer, 
                            &pulses_5_buffer_length);
	ERR_CHECK(err, "Cannot encode NFC message!");
	err = nfc_ndef_msg_encode(&NFC_NDEF_MSG(nfc_pulses_19_text_msg),
				            pulses_19_buffer, 
                            &pulses_19_buffer_length);
	ERR_CHECK(err, "Cannot encode NFC message!");
	err = nfc_ndef_msg_encode(&NFC_NDEF_MSG(nfc_pulses_20_text_msg),
				            pulses_20_buffer, 
                            &pulses_20_buffer_length);
	ERR_CHECK(err, "Cannot encode NFC message!");

	err = nfc_t2t_payload_set(pulses_5_buffer, pulses_5_buffer_length);
	ERR_CHECK(err, "Cannot set NFC payload");

	err = nfc_t2t_emulation_start();
	ERR_CHECK(err, "Cannot start NFC emulation!");

	LOG_INF("NFC configuration done");
}

void wdt_init() {
	nrfx_err = nrfx_wdt_init(&wdt_instance, &wdt_config_normal, wdt_handler);
	NRFX_ERR_CHECK(nrfx_err, "initializing watchdog failed");

	nrfx_err = nrfx_wdt_channel_alloc(&wdt_instance, &wdt_channel_0);
	NRFX_ERR_CHECK(nrfx_err, "allocationg watchdog channel failed");

	nrfx_wdt_enable(&wdt_instance);
}

void turn_opamps_on() {
	// simply setting the pin is to much inrush current resulting in a SOC reset, so the OpAmps must be slowly toggled on
	k_msleep(100);
	for (int i = 0; i < 1000; i++) {
		nrf_gpio_pin_toggle(OPAMPS_ON_OFF);
	}
	nrf_gpio_pin_clear(OPAMPS_ON_OFF);
	k_msleep(100);	// take a pause before and after turn on to stablize the LDO
}

void turn_system_off() {
	LOG_INF("Entering system off.\nApproach a NFC reader to restart.");

	nrf_gpio_pin_set(SYSTEM_ON_LED);
	nrf_gpio_pin_set(OPAMPS_ON_OFF);

	// needed to finish logging before system off
	k_msleep(1);		

	// Above we disabled entry to deep sleep based on duration of
	// controlled delay.  Here we need to override that, then
	// force entry to deep sleep (system off) on any delay.	 		
	pm_state_force(0, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});
	
	
	// Going into sleep will actually go to system off mode, because we
	// forced it above.			
	k_msleep(1);
	
	// k_sleep will never exit, so below two lines will never be executed
	// if system off was correct. On the other hand if someting gone wrong
	// we will see it on terminal and LED.			
	nrf_gpio_pin_clear(SYSTEM_ON_LED);
	LOG_ERR("ERROR: System off failed\n");
}

// https://devzone.nordicsemi.com/f/nordic-q-a/50415/sending-32-bit-of-data-over-ble-onto-nrf52832
void encode_16bit_to_8bit_array(int16_t *data_array, int8_t *ble_send_array, uint16_t data_length) {
    memcpy(ble_send_array, data_array, data_length);
}

void error_handling() {
	for (int i = 0; i < 5; i++) {
		nrf_gpio_pin_toggle(ERROR_LED);
		k_msleep(200);
	}
	nrf_gpio_pin_set(ERROR_LED);
}

void main(void) {
    uint8_t system_off_counter = TIME_TO_DEEPSLEEP_S;

	LOG_INF("Hello World! %s\n", CONFIG_BOARD);

	IRQ_CONNECT(SAADC_IRQn, IRQ_PRIO_LOWEST, nrfx_saadc_irq_handler, NULL, 0);

    nrf_gpio_cfg_output(TRAFO_L1);
	nrf_gpio_cfg_output(TRAFO_L2);
	nrf_gpio_cfg_output(TRAFO_R1);
	nrf_gpio_cfg_output(TRAFO_R2);
	nrf_gpio_cfg_output(BLUE_LED);
	nrf_gpio_cfg_output(RED_LED);
	nrf_gpio_cfg_output(OPAMPS_ON_OFF);

	nrf_gpio_pin_clear(TRAFO_L1);
	nrf_gpio_pin_clear(TRAFO_L2);
	nrf_gpio_pin_clear(TRAFO_R1);
	nrf_gpio_pin_clear(TRAFO_R2);
	nrf_gpio_pin_clear(BLUE_LED);
	nrf_gpio_pin_set(RED_LED);
	nrf_gpio_pin_set(OPAMPS_ON_OFF);

	// err = smp_bt_register();
	// ERR_CHECK(err, "Cannot enable Simple Management Protocol!");
	bt_conn_cb_register(&bluetooth_callbacks);
	err = bt_enable(NULL);
	ERR_CHECK(err, "Cannot enable BLE!");
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	ERR_CHECK(err, "Cannot start advertising");

	wdt_init();
	timer_init();
    ppi_init();
	saadc_init();
    nfc_init();
    nrfx_err = nrfx_pwm_init(&my_pwm, &pwm_peripheral_config, NULL, NULL);
	NRFX_ERR_CHECK(nrfx_err, "Cannot start PWM peripheral")
	
	turn_opamps_on();

	while(true) {
		if (ble_notif_enabled && saadc_buffer_is_full) {		
			// send left sensor values	
			err = bt_gatt_notify(current_conn, &remote_srv.attrs[2], ble_even_flag, sizeof(ble_even_flag));
			ERR_CHECK(err, "BLE notification failed");

			for (int i = 0; i < SAADC_BUF_SIZE; i+=2) {
				encode_16bit_to_8bit_array(&saadc_samples[i], ble_send_array, 2);
				if (!ble_notif_enabled) break;
				err = bt_gatt_notify(current_conn, &remote_srv.attrs[2], ble_send_array, sizeof(ble_send_array));
				ERR_CHECK(err, "BLE notification failed");
			}

			// send right sensor values
			err = bt_gatt_notify(current_conn, &remote_srv.attrs[2], ble_uneven_flag, sizeof(ble_uneven_flag));
			ERR_CHECK(err, "BLE notification failed");

			for (int i = 1; i < SAADC_BUF_SIZE; i+=2) {
				encode_16bit_to_8bit_array(&saadc_samples[i], ble_send_array, 2);
				if (!ble_notif_enabled) break;
				err = bt_gatt_notify(current_conn, &remote_srv.attrs[2], ble_send_array, sizeof(ble_send_array));
				ERR_CHECK(err, "BLE notification failed");
			}
			saadc_buffer_is_full = false;
			k_msleep(1000);
			nrfx_timer_enable(&timer_to_sample_saadc);
			switch (pulse_number) {
			case 4:
				nrfx_pwm_simple_playback(&my_pwm, &pwm_4plus1pulses_seq, 1, NRFX_PWM_FLAG_STOP);
				break;		
			case 5:
				nrfx_pwm_simple_playback(&my_pwm, &pwm_5pulses_seq, 1, NRFX_PWM_FLAG_STOP);
				break;
			case 19:
				nrfx_pwm_simple_playback(&my_pwm, &pwm_19plus1pulses_seq, 1, NRFX_PWM_FLAG_STOP);
				break;
			case 20:
				nrfx_pwm_simple_playback(&my_pwm, &pwm_20pulses_seq, 1, NRFX_PWM_FLAG_STOP);
				break;
			default:
				break;
			}
		}

        if (NULL == current_conn) {
			--system_off_counter;
			if (0 == system_off_counter) {
				turn_system_off();
			} 			
		} else {
			system_off_counter = TIME_TO_DEEPSLEEP_S;
		}

		nrf_gpio_pin_toggle(SYSTEM_ON_LED);
		nrfx_wdt_feed(&wdt_instance);
		k_msleep(1000);
	}
}
