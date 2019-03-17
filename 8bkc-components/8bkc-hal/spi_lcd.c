// Copyright 2016-2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_heap_alloc_caps.h"
#include "ili9225.h"

#include "sdkconfig.h"


#define PIN_NUM_MOSI CONFIG_HW_LCD_MOSI_GPIO
#define PIN_NUM_CLK  CONFIG_HW_LCD_CLK_GPIO
#define PIN_NUM_CS   CONFIG_HW_LCD_CS_GPIO
#define PIN_NUM_DC   CONFIG_HW_LCD_DC_GPIO
#define PIN_NUM_RST  CONFIG_HW_LCD_RESET_GPIO
#define PIN_NUM_BCKL CONFIG_HW_LCD_BL_GPIO

/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} ili_init_cmd_t;

static const ili_init_cmd_t ili_init_cmds[]={
    {ILI9225_POWER_CTRL1, {0x00,0x00}, 2}, // Set SAP,DSTB,STB
    {ILI9225_POWER_CTRL2, {0x00,0x00}, 2}, // Set APON,PON,AON,VCI1EN,VC
    {ILI9225_POWER_CTRL3, {0x00,0x00}, 2}, // Set BT,DC1,DC2,DC3
    {ILI9225_POWER_CTRL4, {0x00,0x00}, 2}, // Set GVDD
    {ILI9225_POWER_CTRL5, {0x00,0x00}, 0x82}, // Set VCOMH/VCOML voltage
    // Power-on sequence
    {ILI9225_POWER_CTRL2, {0x00,0x18}, 2}, // Set APON,PON,AON,VCI1EN,VC
    {ILI9225_POWER_CTRL3, {0x61,0x21}, 2}, // Set BT,DC1,DC2,DC3
    {ILI9225_POWER_CTRL4, {0x00,0x6F}, 2}, // Set GVDD   /*007F 0088 */
    {ILI9225_POWER_CTRL5, {0x49,0x5F}, 2}, // Set VCOMH/VCOML voltage
    {ILI9225_POWER_CTRL1, {0x08,0x00}, 0x82}, // Set SAP,DSTB,STB

    {ILI9225_POWER_CTRL2, {0x10,0x3B}, 0x82}, // Set APON,PON,AON,VCI1EN,VC

    {ILI9225_DRIVER_OUTPUT_CTRL, {0x01,0x1C}, 2}, // set the display line number and display direction
    {ILI9225_LCD_AC_DRIVING_CTRL, {0x01,0x00}, 2}, // set 1 line inversion
    {ILI9225_ENTRY_MODE, {0x10,0x38}, 2}, // set GRAM write direction and BGR=1. This is
    {ILI9225_DISP_CTRL1, {0x00,0x00}, 2}, // Display off
    {ILI9225_BLANK_PERIOD_CTRL1, {0x04,0x04}, 2}, // set the back porch and front porch
    {ILI9225_FRAME_CYCLE_CTRL, {0x11, 0x00}, 2}, // set the clocks number per line
    {ILI9225_INTERFACE_CTRL, {0x00,0x00}, 2}, // CPU interface
    {ILI9225_OSC_CTRL, {0x0D,0x01}, 2}, // Set Osc  /*0e01*/
    {ILI9225_VCI_RECYCLING, {0x00,0x20}, 2}, // Set VCI recycling
    {ILI9225_RAM_ADDR_SET1, {0x00,0x00}, 2}, // RAM Address
    {ILI9225_RAM_ADDR_SET2, {0x00,0x00}, 2}, // RAM Address

    /* Set GRAM area */
    {ILI9225_GATE_SCAN_CTRL, {0x00,0x00}, 2}, 
    {ILI9225_VERTICAL_SCROLL_CTRL1, {0x00,0xDB}, 2}, 
    {ILI9225_VERTICAL_SCROLL_CTRL2, {0x00, 0x00}, 2}, 
    {ILI9225_VERTICAL_SCROLL_CTRL3, {0x00, 0x00}, 2}, 
    {ILI9225_PARTIAL_DRIVING_POS1, {0x00, 0xDB}, 2}, 
    {ILI9225_PARTIAL_DRIVING_POS2, {0x00, 0x00}, 2}, 
    {ILI9225_HORIZONTAL_WINDOW_ADDR1, {0x00, 0xAF}, 2}, 
    {ILI9225_HORIZONTAL_WINDOW_ADDR2, {0x00, 0x00}, 2}, 
    {ILI9225_VERTICAL_WINDOW_ADDR1, {0x00, 0xDB}, 2}, 
    {ILI9225_VERTICAL_WINDOW_ADDR2, {0x00,0x00}, 2}, 

    /* Set GAMMA curve */
    {ILI9225_GAMMA_CTRL1, {0x00, 0x00}, 2},     
    {ILI9225_GAMMA_CTRL2, {0x08, 0x08}, 2},  
    {ILI9225_GAMMA_CTRL3, {0x08, 0x0A}, 2}, 
    {ILI9225_GAMMA_CTRL4, {0x00, 0x0A}, 2}, 
    {ILI9225_GAMMA_CTRL5, {0x0A, 0x08}, 2}, 
    {ILI9225_GAMMA_CTRL6, {0x08, 0x08}, 2}, 
    {ILI9225_GAMMA_CTRL7, {0x00, 0x00}, 2}, 
    {ILI9225_GAMMA_CTRL8, {0x0A, 0x00}, 2}, 
    {ILI9225_GAMMA_CTRL9, {0x07, 0x10}, 2}, 
    {ILI9225_GAMMA_CTRL10, {0x07, 0x10}, 2}, 

    {ILI9225_DISP_CTRL1, {0x00,0x12}, 0x82}, 
    {ILI9225_DISP_CTRL1, {0x10,0x17}, 0x82},

	 {ILI9225_GRAM_DATA_REG,{0},0xFF}
};

static const ili_init_cmd_t ili_window_cmds[]={
	 //Set rotation and stuff
	{ILI9225_ENTRY_MODE, {0x10,0x28}, 0x82},
   {ILI9225_HORIZONTAL_WINDOW_ADDR1,{0x00,0x9F}, 2},
   {ILI9225_HORIZONTAL_WINDOW_ADDR2,{0x00, 0x10}, 0x82},

   {ILI9225_VERTICAL_WINDOW_ADDR1,{0x00, 0xBD}, 2},
   {ILI9225_VERTICAL_WINDOW_ADDR2,{0x00, 0x1E},0x82},

   //THIS NEEDS to be changed according to the orientaiton set in entry mode above
   {ILI9225_RAM_ADDR_SET1,{0x00,0x9F}, 2},
   {ILI9225_RAM_ADDR_SET2,{0x00,0x1E}, 0x82},
	{ILI9225_GRAM_DATA_REG,{0},0xFF}
};



static spi_device_handle_t spi;


//Send a command to the ILI9341. Uses spi_device_transmit, which waits until the transfer is complete.
void ili_cmd(spi_device_handle_t spi, const uint8_t cmd) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//Send data to the ILI9341. Uses spi_device_transmit, which waits until the transfer is complete.
void ili_data(spi_device_handle_t spi, const uint8_t *data, int len) 
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void ili_spi_pre_transfer_callback(spi_transaction_t *t) 
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

//Initialize the display
void ili_init(spi_device_handle_t spi) 
{
    int cmd=0;
    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    //Send all the commands
    while (ili_init_cmds[cmd].databytes!=0xff) {
        uint8_t dmdata[16];

        ili_cmd(spi, ili_init_cmds[cmd].cmd);
        //Need to copy from flash to DMA'able memory
        memcpy(dmdata, ili_init_cmds[cmd].data, 16);
        ili_data(spi, dmdata, ili_init_cmds[cmd].databytes&0x1F);
        if (ili_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
		  cmd++;
    }

	 //Blank the entire LCD
	 ili_cmd(spi,ILI9225_GRAM_DATA_REG);
	 #define FILLCOLOR 0x00
	 for(int c=0; c < (220*176*2);c++) {
	     uint8_t dmdata[16];
		  for(int index=0; index < 16; index++) {
		     dmdata[index] = FILLCOLOR;
		  }
		  ili_data(spi,dmdata,16);
	 }

	 cmd=0;		//reset counter
    //Copy data to set window
	 while (ili_window_cmds[cmd].databytes!=0xff) { 
        uint8_t dmdata[16];

        ili_cmd(spi, ili_window_cmds[cmd].cmd);
        //Need to copy from flash to DMA'able memory
        memcpy(dmdata, ili_window_cmds[cmd].data, 16);
        ili_data(spi, dmdata, ili_window_cmds[cmd].databytes&0x1F);
        if (ili_window_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
		  cmd++;
    }
    ///Enable backlight
#if CONFIG_HW_INV_BL
    gpio_set_level(PIN_NUM_BCKL, 0);
#else
    gpio_set_level(PIN_NUM_BCKL, 1);
#endif
	/*
	for(int i=0; i < 1000; i++) {
		vTaskDelay(100 / portTICK_RAdd`TE_MS);
		gpio_set_level(PIN_NUM_DC, 1);
	   gpio_set_level(PIN_NUM_CS, 0);
	   uint16_t data = COLOR_RED;
	   for(uint16_t bit = 0x8000; bit; bit >>= 1){
         if((data) & bit){
            gpio_set_level(PIN_NUM_MOSI,1);
         } else {
            gpio_set_level(PIN_NUM_MOSI,0);
         }
         gpio_set_level(PIN_NUM_CLK,1);
     	   gpio_set_level(PIN_NUM_CLK,0);
   	}
		gpio_set_level(PIN_NUM_CS,1);
	}
	*/
	

}


static void send_header_start(spi_device_handle_t spi, int xpos, int ypos, int w, int h)
{
    esp_err_t ret;
    int x;
    //Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
    //function is finished because the SPI driver needs access to it even while we're already calculating the next line.
    static spi_transaction_t trans[1];

    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. 
    for (x=0; x<1; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x&1)==0) {
            //Even transfers are commands
            trans[x].length=8;
            trans[x].user=(void*)0;
        } else {
            //Odd transfers are data
            trans[x].length=8*4;
            trans[x].user=(void*)1;
        }
        trans[x].flags=SPI_TRANS_USE_TXDATA;
    }
	 /*
    trans[0].tx_data[0]=0x2A;           //Column Address Set
    trans[1].tx_data[0]=xpos>>8;              //Start Col High
    trans[1].tx_data[1]=xpos;              //Start Col Low
    trans[1].tx_data[2]=(xpos+w-1)>>8;       //End Col High
    trans[1].tx_data[3]=(xpos+w-1)&0xff;     //End Col Low
    trans[2].tx_data[0]=0x2B;           //Page address set
    trans[3].tx_data[0]=ypos>>8;        //Start page high
    trans[3].tx_data[1]=ypos&0xff;      //start page low
    trans[3].tx_data[2]=(ypos+h-1)>>8;    //end page high
    trans[3].tx_data[3]=(ypos+h-1)&0xff;  //end page low
    trans[4].tx_data[0]=0x2C;           //memory write

	 trans[0].tx_data[0]=ILI9225_ENTRY_MODE;
	 trans[1].tx_data[0]=0x00;
	 trans[1].tx_data[1]=0x10;

	 trans[2].tx_data[0]=ILI9225_HORIZONTAL_WINDOW_ADDR1;
	 //trans[3].tx_data[0]=(xpos+w-1)>>8;
	 //trans[3].tx_data[1]=(xpos+w-1)&0xff;
	 trans[3].tx_data[0]=0x00;
	 trans[3].tx_data[1]=0xAF;
	 trans[4].tx_data[0]=ILI9225_HORIZONTAL_WINDOW_ADDR2;
	 trans[5].tx_data[0]=0x00;
	 trans[5].tx_data[1]=0x00;
	 trans[6].tx_data[0]=ILI9225_VERTICAL_WINDOW_ADDR1;
	 //trans[5].tx_data[0]=(ypos+h-1)>>8;
	 //trans[5].tx_data[1]=(ypos+h-1)&0xff;
	 trans[7].tx_data[0]=0x00;
	 trans[7].tx_data[1]=0xDB;
	 trans[8].tx_data[0]=ILI9225_VERTICAL_WINDOW_ADDR2;
	 trans[9].tx_data[0]=0x00;
	 trans[9].tx_data[1]=0x00;*/
	 trans[0].tx_data[0]=ILI9225_GRAM_DATA_REG;

    //Queue all transactions.
    for (x=0; x<1; x++) {
        ret=spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
        assert(ret==ESP_OK);
    }

    //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
    //mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
    //finish because we may as well spend the time calculating the next line. When that is done, we can call
    //send_line_finish, which will wait for the transfers to be done and check their status.
}


void send_header_cleanup(spi_device_handle_t spi) 
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 5 transactions to be done and get back the results.
    for (int x=0; x<1; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}

void spi_lcd_send(int x, int y, int w, int h, uint16_t *scr) {
    esp_err_t ret;
    spi_transaction_t trans={0};
    spi_transaction_t *rtrans;
	send_header_start(spi, x, y, w, h);
	trans.length=w*h*16;
	trans.user=(void*)1;
	trans.tx_buffer=scr;
    ret=spi_device_queue_trans(spi, &trans, portMAX_DELAY);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
	send_header_cleanup(spi);
    ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
    assert(ret==ESP_OK);
}

void spi_lcd_init() {
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=-1,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        #if (CONFIG_HW_LCD_TYPE == 2)
        .max_transfer_sz=(160*128*2)+16
        #elif (CONFIG_HW_LCD_TYPE == 3)
		  .max_transfer_sz=(220*176*2)+16
		  #else
        .max_transfer_sz=(320*240*2)+16
        #endif
    };
    spi_device_interface_config_t devcfg={
	 		#if (CONFIG_HW_LCD_TYPE == 3)
        .clock_speed_hz=26000000,               //Clock out at 26 MHz. Yes, that's heavily overclocked.
        #else
		  .clock_speed_hz=26000000,               //Clock out at 26 MHz. Yes, that's heavily overclocked.
        #endif
		  .mode=0,          								//SPI mode 0
		  .flags=SPI_DEVICE_HALFDUPLEX,
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=10,               //We want to be able to queue this many transfers
        .pre_cb=ili_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };

    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);
    //Initialize the LCD
    ili_init(spi);
}
