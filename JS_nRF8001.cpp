#include "JS_nRF8001.h"


#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
    static services_pipe_type_mapping_t
        services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
    #define NUMBER_OF_PIPES 0
    static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif

static hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;

static struct aci_state_t aci_state;

static hal_aci_evt_t  aci_data;

static bool timing_change_done = false;

#define MAX_RX_BUFF 64

static uint8_t rx_buff[MAX_RX_BUFF+1];
static uint8_t rx_buffer_len = 0;
static uint8_t *p_before = &rx_buff[0] ;
static uint8_t *p_back = &rx_buff[0];

static unsigned char is_connected = 0;

uint8_t reqn_pin = BOARD_REQN;
uint8_t rdyn_pin = BOARD_RDYN;

static unsigned char spi_old;

void __ble_assert(const char *file, uint16_t line)
{
    Serial.print("ERROR ");
    Serial.print(file);
    Serial.print(": ");
    Serial.print(line);
    Serial.print("\n");
    while(1);
}

void BLE_initialize()
{
    spi_old = SPCR;
    SPI.setBitOrder(LSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    SPI.setDataMode(SPI_MODE0);
    if (NULL != services_pipe_type_mapping)
    {
        aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
    }
    else
    {
        aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
    }
    aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
    aci_state.aci_setup_info.setup_msgs         = setup_msgs;
    aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;

    aci_state.aci_pins.board_name = BOARD_DEFAULT;
    aci_state.aci_pins.reqn_pin   = 10;
    aci_state.aci_pins.rdyn_pin   = 3;
    aci_state.aci_pins.mosi_pin   = MOSI;
    aci_state.aci_pins.miso_pin   = MISO;
    aci_state.aci_pins.sck_pin    = SCK;

    aci_state.aci_pins.spi_clock_divider     = SPI_CLOCK_DIV8;

    aci_state.aci_pins.reset_pin             = 4;
    aci_state.aci_pins.active_pin            = UNUSED;
    aci_state.aci_pins.optional_chip_sel_pin = UNUSED;

    aci_state.aci_pins.interface_is_interrupt	  = false;
    aci_state.aci_pins.interrupt_number			  = 1;//4;

    lib_aci_init(&aci_state, false);

    SPCR = spi_old;
    SPI.begin();
}

static volatile byte ack = 0;

int BLE_get()
{
	int data;
	if(rx_buffer_len == 0) return -1;
	if(p_before == &rx_buff[MAX_RX_BUFF])
	{
        p_before = &rx_buff[0];
	}
	data = *p_before;
	p_before ++;
	rx_buffer_len--;
	return data;
}

unsigned char BLE_free()
{
	return rx_buffer_len;
}

static void aci_loop()
{
    static bool setup_required = false;
    
    if (lib_aci_event_get(&aci_state, &aci_data))
    {
        aci_evt_t  *aci_evt;
        aci_evt = &aci_data.evt;
        switch(aci_evt->evt_opcode)
        {
            case ACI_EVT_DEVICE_STARTED:
                aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
                switch(aci_evt->params.device_started.device_mode)
                {
                    case ACI_DEVICE_SETUP:
                        Serial.println(F("Evt Device Started: Setup"));
                        setup_required = true;
                        break;
                    case ACI_DEVICE_STANDBY:
                        Serial.println(F("Evt Device Started: Standby"));
                        if (aci_evt->params.device_started.hw_error)
                        {
                            delay(20); //Magic number used to make sure the HW error event is handled correctly.
                        }
                        else
                        { 
                            lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
                            Serial.println(F("Advertising started"));
                        }
                        break;
                }
                break; 

            case ACI_EVT_CMD_RSP:
                if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
                {
                    Serial.print(F("ACI Command "));
                    Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
                    Serial.print(F("Evt Cmd respone: Status "));
                    Serial.println(aci_evt->params.cmd_rsp.cmd_status, HEX);
                }
                if (ACI_CMD_GET_DEVICE_VERSION == aci_evt->params.cmd_rsp.cmd_opcode)
                {
                    lib_aci_set_local_data(&aci_state, PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET,
                    (uint8_t *)&(aci_evt->params.cmd_rsp.params.get_device_version), sizeof(aci_evt_cmd_rsp_params_get_device_version_t));
                }
                break;

            case ACI_EVT_CONNECTED:
                is_connected = 1;
                Serial.println(F("Evt Connected"));
                timing_change_done = false;
                aci_state.data_credit_available = aci_state.data_credit_total;
                lib_aci_device_version();
                break;

            case ACI_EVT_PIPE_STATUS:
                Serial.println(F("Evt Pipe Status"));
                if (lib_aci_is_pipe_available(&aci_state, PIPE_UART_OVER_BTLE_UART_TX_TX) && (false == timing_change_done))
                {
                    lib_aci_change_timing_GAP_PPCP();                                
                    timing_change_done = true;
                }
                break;

            case ACI_EVT_TIMING:
                Serial.println(F("Evt link connection interval changed"));
                break;

            case ACI_EVT_DISCONNECTED:
                is_connected = 0;
                ack = 1;
                Serial.println(F("Evt Disconnected/Advertising timed out"));
                lib_aci_connect(30/* in seconds */, 0x0050 /* advertising interval 100ms*/);
                Serial.println(F("Advertising started"));
                break;

            case ACI_EVT_DATA_RECEIVED:
                Serial.print(F("Pipe Number: "));
                Serial.println(aci_evt->params.data_received.rx_data.pipe_number, DEC);
                for(int i=0; i<aci_evt->len - 2; i++)
                {
                    if(rx_buffer_len == MAX_RX_BUFF)
                    {
                        break;
                    }
                    else
                    {
                        if(p_back == &rx_buff[MAX_RX_BUFF])
                        {
                            p_back = &rx_buff[0];
                        }
                        *p_back = aci_evt->params.data_received.rx_data.aci_data[i];
                        rx_buffer_len++;
                        p_back++;
                    }
                }
                break;

            case ACI_EVT_DATA_CREDIT:
                aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
                Serial.print("ACI_EVT_DATA_CREDIT     ");
                Serial.print("Data Credit available: ");
                Serial.println(aci_state.data_credit_available,DEC);
                ack=1;
                break;

            case ACI_EVT_PIPE_ERROR:
                Serial.print(F("ACI Evt Pipe Error: Pipe #:"));
                Serial.print(aci_evt->params.pipe_error.pipe_number, DEC);
                Serial.print(F("  Pipe Error Code: 0x"));
                Serial.println(aci_evt->params.pipe_error.error_code, HEX);

                if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code)
                {
                    aci_state.data_credit_available++;
                }
                Serial.print("Data Credit available: ");
                Serial.println(aci_state.data_credit_available,DEC);
                break;

            case ACI_EVT_HW_ERROR:
                Serial.print(F("HW error: "));
                Serial.println(aci_evt->params.hw_error.line_num, DEC);

                for(uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
                {
                  Serial.write(aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
                }
                Serial.println();
                lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
                Serial.println(F("Advertising started"));
                break;
        }
    }
    else
    {
        //Serial.println(F("No ACI Events available"));
        // No event in the ACI Event queue and if there is no event in the ACI command queue the arduino can go to sleep
        // Arduino can go to sleep now
        // Wakeup from sleep from the RDYN line
    }
    
    if(setup_required)
    {
        if (SETUP_SUCCESS == do_aci_setup(&aci_state))
        {
            setup_required = false;
        }
    }
}

void BLE_process()
{
    spi_old = SPCR;
    SPI.setBitOrder(LSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    SPI.setDataMode(SPI_MODE0);

    aci_loop();

    SPCR = spi_old;
}