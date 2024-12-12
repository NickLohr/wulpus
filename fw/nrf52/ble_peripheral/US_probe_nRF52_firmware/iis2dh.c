#include "iis2dh.h"

/* TWI instance */
static const nrf_drv_twi_t IIS2DH_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Flag to know when a I2C transfer has been completed */
static volatile bool IIS2DH_xfer_done = false;


uint8_t IIS2DH_buffer[805] = {};
uint16_t IIS2DH_buffer_index =0;
/* Frame counter for IIS2DH over bluetooth */
uint16_t IIS2DH_frame_number =0;


//Event Handler
static void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    //Check the event to see what type of event occurred
    switch (p_event->type)
    {
        //If data transmission or receiving is finished
	case NRF_DRV_TWI_EVT_DONE:
        IIS2DH_xfer_done = true;//Set the flag
        break;
        
        default:
        // do nothing
          break;
    }
}


/**
 * @brief TWI initialization function for the MCP9808 sensor
 *
 */
static void iis2dh_twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = PIN_IIS2DH_SCL,
       .sda                = PIN_IIS2DH_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_100K,  // TODO change
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST, // TODO change
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&IIS2DH_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&IIS2DH_twi);
}

void IIS2DH_init()
{
    //nrf_delay_ms(500); // TODO maybe read
    iis2dh_twi_init();
    IIS2DH_buffer[0] = 0xFF;
    IIS2DH_buffer[1] = 128;
    //((uint16_t*)IIS2DH_buffer)[1] = IIS2DH_frame_number++;
        IIS2DH_buffer[3] = 0;
    IIS2DH_buffer[2] = IIS2DH_frame_number++;

    IIS2DH_buffer_index = 4;
    nrf_delay_ms(500);
    // TODO write setup (control registers)
}

bool IIS2DH_register_read(uint8_t register_address, uint8_t * rx_buffer, uint8_t number_of_bytes)
{
    ret_code_t err_code;

    //Set the flag to false to show the receiving is not yet completed
    IIS2DH_xfer_done = false;

    // Send the register address where we want to read the data from
    err_code = nrf_drv_twi_tx(&IIS2DH_twi, IIS2DH_ADDRESS, &register_address, 1, true);

    //Wait for the transmission to be completed
    while (IIS2DH_xfer_done == false){}
    
    // If transmission was not successful, exit the function and return false
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }

    //reset the flag so that we can read data from the IIS2DH's internal register
    IIS2DH_xfer_done = false;
	  
    // Receive the data from the IIS2DH
    err_code = nrf_drv_twi_rx(&IIS2DH_twi, IIS2DH_ADDRESS, rx_buffer, number_of_bytes);
    //wait until the transmission is completed
    while (IIS2DH_xfer_done == false){}
	
    // if data was successfully read, return true else return false
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }
    
    return true;
}
bool IIS2DH_register_write(uint8_t register_address, uint8_t * wx_buffer, uint8_t number_of_bytes)
{
    ret_code_t err_code;

    //Set the flag to false to show the receiving is not yet completed
    IIS2DH_xfer_done = false;

    uint8_t wxbuffer_with_address[10];
    wxbuffer_with_address[0] = register_address;
    if (number_of_bytes>8) return false;
    for (int i = 0; i<number_of_bytes; i++){
      wxbuffer_with_address[i+1] = wx_buffer[i];
    }
   
    // Send the register address where we want to read the data from
    err_code = nrf_drv_twi_tx(&IIS2DH_twi, IIS2DH_ADDRESS, wxbuffer_with_address, number_of_bytes+1, true);

    //Wait for the transmission to be completed
    while (IIS2DH_xfer_done == false){}
    
    // If transmission was not successful, exit the function and return false
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }
    return true;
}

bool setupTemp(){
  uint8_t wx_buffer[1] = {0};
  wx_buffer[0] = 0b11000000;
  bool check = IIS2DH_register_write(IIS2DH_REG_TEMP_CFG_REG,wx_buffer , 1);
  if (!check){
    return false;
  }
  uint8_t rx_buffer[1] = {0};

  check = IIS2DH_register_read(IIS2DH_REG_CTRL_REG4,rx_buffer , 1);
  if (!check){
    return false;
  }
  wx_buffer[0] = rx_buffer[0] | 0x80;
  check = IIS2DH_register_write(IIS2DH_REG_CTRL_REG4,wx_buffer , 1);
  if (!check){
    return false;
  }


}

int8_t getTemp(){
  uint8_t rx_buffer[1] = {0};
  bool check = IIS2DH_register_read(IIS2DH_REG_OUT_TEMP_H, rx_buffer, 1);
  if (!check){
    return 0;
  }
  int8_t res = rx_buffer[0];
  check = IIS2DH_register_read(IIS2DH_REG_OUT_TEMP_L, rx_buffer, 1);
  if (!check){
    return 0;
  }
  //res|= rx_buffer[0]; // TODO Temperature data is stored inside OUT_TEMP_H as two’s complement data in 8-bit format left-justified.
  return res;
  

}

bool setupAccelormeter(IIS2DH_OperatingModes mode, IIS2DH_DataRate rate, IIS2DH_FullScale fs){
  uint8_t wx_reg1_buffer[1];
  uint8_t wx_reg4_buffer[1];

  if (mode == IIS2DH_LowPowerMode){
    wx_reg1_buffer[0] = 0x0F;
  }else{
    wx_reg1_buffer[0] = 0x07;
  }
  wx_reg1_buffer[0] = wx_reg1_buffer[0] | rate; // set the odr bits
  if (mode == IIS2DH_HighResolutionMode){
    wx_reg4_buffer[0] = 0b00001000;
  }else{
    wx_reg4_buffer[0] = 0b00000000;
  }
  wx_reg4_buffer[0] |= wx_reg4_buffer[0] | fs;
  
 
  bool res = IIS2DH_register_write(IIS2DH_REG_CTRL_REG1, wx_reg1_buffer, 1);
  res &= IIS2DH_register_write(IIS2DH_REG_CTRL_REG4, wx_reg4_buffer, 1);
  return true;
}


double convert2mg(uint16_t data_in, IIS2DH_FullScale range){
  if (range == IIS2DH_Precision_2g){
    return ((double) data_in)*0.06125;
  }else if (range == IIS2DH_Precision_4g){
    return ((double) data_in)*0.121875;
  }else if (range == IIS2DH_Precision_8g){
    return ((double) data_in)*0.244375;
  }else if (range == IIS2DH_Precision_16g){
    return ((double) data_in)*0.7325;
  }
}

uint16_t getHighandLow(uint8_t register_address_L, uint8_t register_address_H, IIS2DH_OperatingModes mode){ // TODO add precision
  uint8_t rx_buffer[1] ={0};

  IIS2DH_register_read(register_address_H, rx_buffer, 1);
  uint16_t res = rx_buffer[0]<<8;
  rx_buffer[0] =0;
  IIS2DH_register_read(register_address_L, rx_buffer, 1);
  res += rx_buffer[0];

  if (mode == IIS2DH_LowPowerMode){
    res = res>>8;
  }else if (mode == IIS2DH_NormalMode){
    res = res>>6;
  }else if (mode == IIS2DH_HighResolutionMode){
    res = res>>4;
  }
  return res;

}

bool getAccelerationData(uint16_t* X, uint16_t* Y, uint16_t* Z, IIS2DH_OperatingModes mode, IIS2DH_FullScale range){
  *X = getHighandLow(IIS2DH_REG_OUT_X_L, IIS2DH_REG_OUT_X_H, mode);  
  *Y = getHighandLow(IIS2DH_REG_OUT_Y_L, IIS2DH_REG_OUT_Y_H, mode);
  *Z = getHighandLow(IIS2DH_REG_OUT_Z_L, IIS2DH_REG_OUT_Z_H, mode);
  //*X = convert2mg(X_in, range);
  //*Y = convert2mg(Y_in, range);
  //*Z = convert2mg(Z_in, range);


  return true;

}


void getIIS2DHData2Buffer(){
  IIS2DH_register_read(IIS2DH_REG_OUT_X_L, &IIS2DH_buffer[IIS2DH_buffer_index++], 1); 
  IIS2DH_register_read(IIS2DH_REG_OUT_X_H, &IIS2DH_buffer[IIS2DH_buffer_index++], 1); 
  IIS2DH_register_read(IIS2DH_REG_OUT_Y_L, &IIS2DH_buffer[IIS2DH_buffer_index++], 1); 
  IIS2DH_register_read(IIS2DH_REG_OUT_Y_H, &IIS2DH_buffer[IIS2DH_buffer_index++], 1); 
  IIS2DH_register_read(IIS2DH_REG_OUT_Z_L, &IIS2DH_buffer[IIS2DH_buffer_index++], 1); 
  IIS2DH_register_read(IIS2DH_REG_OUT_Z_H, &IIS2DH_buffer[IIS2DH_buffer_index++], 1); 
  NRF_LOG_INFO("Received: %u", IIS2DH_buffer_index);

  //IIS2DH_buffer_index +=6;
  if (IIS2DH_buffer_index >=805){
  NRF_LOG_INFO("Sending...");
    send_packet(IIS2DH_buffer, 202);
    send_packet(&IIS2DH_buffer[201], 201);
    send_packet(&IIS2DH_buffer[402], 201);
    send_packet(&IIS2DH_buffer[603], 201);
    //((uint16_t*)&IIS2DH_buffer)[1] = IIS2DH_frame_number++;
    IIS2DH_buffer[3] = 0;
    IIS2DH_buffer[2] = IIS2DH_frame_number++;

    IIS2DH_buffer_index = 4;

  }







}