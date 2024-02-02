/*******************************************************************************
Created by profi-max (Oleg Linnik) 2024
https://profimaxblog.ru
https://github.com/profi-max

*******************************************************************************/
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "SoftwareSerial.h"
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager


// modbus defines
#define MB_PORT 502
#define MB_BUFFER_SIZE 256
#define MB_RTU_SLAVE_RESPONSE_TIMEOUT_MS 1000

#define  MB_FC_NONE                     0
#define  MB_FC_READ_REGISTERS           3
#define  MB_FC_WRITE_REGISTER           6
#define  MB_FC_WRITE_MULTIPLE_REGISTERS 16
#define  MB_FC_ERROR_MASK 				128

#define DPS_COMM_WIFI_RESET 0x20
#define DPS_COMM_WIFI_MODE 0x18

//*************  USER DEFINES  *************
#define WIFI_RESET_BUTTON_PRESS_TIME_MS 3000 // long press the button for 3 sec
#define WIFI_MANAGER_TIMEOUT 120 // seconds wifi manager runs
#define START_MODBUS_DELAY_MS 2000 // modbus delay after power up in milliseconds

// comment the line below if your DPSxxxx device has chinesse firmware
#define DPS_ALTERNATIVE_FIRMWARE

//  set define below to minus one if you dont need reset button
#define WIFI_RESET_BUTTON_PIN 0

// uncomment the line below if you need debug via USB/UART0
//#define MB_DEBUG

// uncomment the line bellow if you need buil-in LED
//#define MB_USE_LED

const char wifi_manager_name[] = "DPS TCP bridge";
//*************  END of USER DEFINES  *************

WiFiServer mb_server(MB_PORT);
WiFiClient client;

SoftwareSerial swSerial;

uint8_t mb_buffer[MB_BUFFER_SIZE]; // send and recieve buffer
uint8_t tcp_req[9];
bool wait_for_rtu_response = false;
bool wait_for_slave_read_response = false;
bool wait_for_slave_write_response = false;
uint16_t * read_buffer;
uint32_t time_stamp;
uint16_t dps_comm_reg = 0;

/*=================================================================================================*/
static const char aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

static const char aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};

/*=================================================================================================*/
/* Calculate CRC16 */
uint16_t crc16( uint8_t * pucFrame, uint16_t usLen )
{
    uint8_t           ucCRCHi = 0xFF;
    uint8_t           ucCRCLo = 0xFF;
    int             iIndex;

    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( uint8_t )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( uint16_t )( ucCRCHi << 8 | ucCRCLo );
}

/*=================================================================================================*/
void mb_debug(String str, uint16_t start, uint16_t len)
{
#ifdef MB_DEBUG
	swSerial.print(str);
	for (uint16_t i = 0; i < len; i++) 
	{
		swSerial.printf(" %02X", mb_buffer[i + start]);
	}
	swSerial.println("");
#endif
}

/*=================================================================================================*/
void mb_led_on(bool on)
{
#ifdef MB_USE_LED
	if (on) digitalWrite(LED_BUILTIN, LOW); // LED on
	else digitalWrite(LED_BUILTIN, HIGH); // LED off
#endif	
}

/*=================================================================================================*/
void mb_serial_write(const uint8_t* buf, size_t size)
{
	mb_debug("RTU TX:", 6, size);
	while(Serial.available())
		Serial.read();
	Serial.write(buf, size);
	Serial.flush();
	time_stamp = millis();
	mb_led_on(false);
}

/*=================================================================================================*/
/* TCP client poll, if TCP request received -> send RTU request */
void modbusTcpServerBridgeTask(void)
{
  	uint16_t crc;
  	uint8_t mb_func = MB_FC_NONE;
  	uint16_t len = 0;
  
	if (mb_server.hasClient()) // Check if there is new client
	{
		// if client is free - connect
		if (!client || !client.connected()){
			if(client) client.stop();
			client = mb_server.accept(); //.available();
			
		// client is not free - reject
		} 
		else 
		{
			WiFiClient serverClient = mb_server.accept(); //.available();
			serverClient.stop();
		}
	}

  	//-------------------- Read from socket --------------------
  	if (client && client.connected() && client.available())
  	{
		delay(1);
		uint16_t bytesReady;
		while((bytesReady = client.available()) && (len < MB_BUFFER_SIZE))
		{
			len += client.readBytes(&mb_buffer[len], bytesReady);
		}
		if (len > 8)  mb_func = mb_buffer[7];  //Byte 7 of request is FC
		mb_debug("TCP RX:", 0, len);
	}
	else 
	{
		return;
	}	
        //-------------------- Read Registers (3) --------------------
        //-------------------- Write Register (6) --------------------
    if ((mb_func == MB_FC_READ_REGISTERS) || (mb_func == MB_FC_WRITE_REGISTER) ) 
	{
		crc =  crc16(&mb_buffer[6], 6);
		mb_buffer[12] = lowByte(crc);
		mb_buffer[13] = highByte(crc);
		mb_serial_write(&mb_buffer[6], 8);
		memcpy(tcp_req, mb_buffer, sizeof(tcp_req));
		wait_for_rtu_response = true;
    }
		//-------------------- Write Multiple Registers (16) --------------------
	else if (mb_func == MB_FC_WRITE_MULTIPLE_REGISTERS) 
	{
		uint16_t len = 7 + mb_buffer[12];
		crc =  crc16(&mb_buffer[6], len);
		mb_buffer[6 + len] = lowByte(crc);
		mb_buffer[7 + len] = highByte(crc);
		mb_serial_write(&mb_buffer[6], len + 2);
		memcpy(tcp_req, mb_buffer, sizeof(tcp_req));
		wait_for_rtu_response = true;
	}
}
/*=================================================================================================*/
void mb_client_write(const uint8_t * buf, size_t size)
{
	if (client && client.connected()) 
	{
		client.write(buf, size);
		client.flush();
		if (buf == mb_buffer)
			mb_debug("TCP TX:", 0, size);
	}
}

/*=================================================================================================*/
/* RTU master poll, if RTU response received -> send TCP response */
void modbusRtuMasterBridgeTask(void)
{
	uint16_t len = 0;
	bool timeout = true;
	bool crc_err = false;
  	uint8_t mb_func = MB_FC_NONE;

	while(millis() - time_stamp < MB_RTU_SLAVE_RESPONSE_TIMEOUT_MS)
	{
		uint16_t bytesReady = Serial.available();
		if (bytesReady > 0)
		{
			len += Serial.readBytes(&mb_buffer[6 + len], bytesReady);
			mb_func = mb_buffer[7];
			if ((len >= 5) && (mb_func >= MB_FC_ERROR_MASK)) // error code 
				{timeout = false; crc_err = crc16(&mb_buffer[6], 5) != 0; break;} 
			if ((len >= 8) && ((mb_func == MB_FC_WRITE_MULTIPLE_REGISTERS) || (mb_func == MB_FC_WRITE_REGISTER))) 
				{timeout = false; crc_err = crc16(&mb_buffer[6], 8) != 0; break;} 
			if ((len >= 7) && (mb_func == MB_FC_READ_REGISTERS) && (len >= mb_buffer[8] + 3 + 2)) 
				{timeout = false; crc_err = crc16(&mb_buffer[6], mb_buffer[8] + 3 + 2) != 0; break;} 
		}
	}

	if (timeout)
	{
		mb_debug("RTU RX timeout", 6, len);
		tcp_req[5] = 3; // rest bytes
		tcp_req[7] |= MB_FC_ERROR_MASK;
		tcp_req[8] = 0x04; // error code
		mb_client_write(tcp_req, 9);
	}
	else if (crc_err)
	{
		mb_debug("RTU RX:", 6, len);
		tcp_req[5] = 3; // rest bytes
		tcp_req[7] |= MB_FC_ERROR_MASK;
		tcp_req[8] = 0x08; // error code
		mb_client_write(tcp_req, 9);
	}
	else
	{
		mb_debug("RTU RX:", 6, len);
		mb_buffer[4] = 0;
		if (mb_func >= MB_FC_ERROR_MASK)
		{
			mb_buffer[5] = 3; // rest bytes
			mb_client_write(mb_buffer, 9);
		}
        	//-------------------- Response Write Register (6) --------------------
			//-------------------- Response Write Multiple Registers (16) --------------------
		else if ((mb_func == MB_FC_WRITE_MULTIPLE_REGISTERS) || (mb_func == MB_FC_WRITE_REGISTER))
		{
			mb_buffer[5] = 6; // rest bytes
			mb_client_write(mb_buffer, 12);
		}
        	//-------------------- Response Read Registers (3) --------------------
		else if (mb_func == MB_FC_READ_REGISTERS) 
		{
			mb_buffer[5] = mb_buffer[8] + 3; // rest bytes
			mb_client_write(mb_buffer, mb_buffer[8] + 3 + 6);
		}
	}
	wait_for_rtu_response = false;
	mb_led_on(true);
}

#ifdef DPS_ALTERNATIVE_FIRMWARE
/*=================================================================================================*/
void modbusRtuMasterReadRegisters(uint16_t regAddress, uint16_t regCount, uint16_t * buf)
{
	read_buffer = buf;
	mb_buffer[6] = 1;
	mb_buffer[7] = MB_FC_READ_REGISTERS;
	mb_buffer[8] = highByte( regAddress );
	mb_buffer[9] = lowByte( regAddress );
	mb_buffer[10] = highByte( regCount );
	mb_buffer[11] = lowByte( regCount );
	uint16_t crc = crc16(&mb_buffer[6], 6);
	mb_buffer[12] = lowByte(crc);
	mb_buffer[13] = highByte(crc);
	mb_serial_write(&mb_buffer[6], 8);
	wait_for_slave_read_response = true;
}

/*=================================================================================================*/
void modbusRtuMasterWriteRegisters(uint16_t regAddress, uint16_t regCount, uint16_t * buf)
{
	mb_buffer[6] = 1;
	mb_buffer[7] = MB_FC_WRITE_MULTIPLE_REGISTERS;
	mb_buffer[8] = highByte( regAddress );
	mb_buffer[9] = lowByte( regAddress );
	mb_buffer[10] = highByte( regCount );
	mb_buffer[11] = lowByte( regCount );
	mb_buffer[12] = regCount * 2;
	for (uint16_t i = 0; i < regCount; i++)
	{
		mb_buffer[13 + i * 2]	= highByte(buf[i]);
		mb_buffer[14 + i * 2]	= lowByte(buf[i]);
	}
	uint16_t len = 7 + mb_buffer[12];
	uint16_t crc = crc16(&mb_buffer[6], len);
	mb_buffer[6 + len] = lowByte(crc);
	mb_buffer[7 + len] = highByte(crc);
	mb_serial_write(&mb_buffer[6], len + 2);
	wait_for_slave_write_response = true;
}

/*=================================================================================================*/
uint8_t modbusRtuMasterPoll(void)
{
	uint16_t len = 0;
	bool timeout = true;
	bool crc_err = false;
	uint8_t ret = 0;
  	uint8_t mb_func = MB_FC_NONE;

	while(millis() - time_stamp < MB_RTU_SLAVE_RESPONSE_TIMEOUT_MS)
	{
		uint16_t bytesReady = Serial.available();
		if (bytesReady > 0)
		{
			len += Serial.readBytes(&mb_buffer[6 + len], bytesReady);
			mb_func = mb_buffer[7];
			if ((len >= 5) && (mb_func >= MB_FC_ERROR_MASK)) // error code 
				{timeout = false; crc_err = crc16(&mb_buffer[6], 5) != 0; break;} 
			if ((len >= 8) && ((mb_func == MB_FC_WRITE_MULTIPLE_REGISTERS) || (mb_func == MB_FC_WRITE_REGISTER))) 
				{timeout = false; crc_err = crc16(&mb_buffer[6], 8) != 0; break;} 
			if ((len >= 7) && (mb_func == MB_FC_READ_REGISTERS) && (len >= mb_buffer[8] + 3 + 2)) 
				{timeout = false; crc_err = crc16(&mb_buffer[6], mb_buffer[8] + 3 + 2) != 0; break;} 
		}
	}

	if (timeout)
	{
		mb_debug("RTU RX timeout: ", 6, len);
		ret = 0;
	}
	else
	{
		if ((mb_func >= MB_FC_ERROR_MASK) || crc_err)
		{
			mb_debug("RTU RX error:", 6, len);
			ret = 0;
		}
        	//-------------------- Response Write Register (6) --------------------
			//-------------------- Response Write Multiple Registers (16) --------------------
		else if ((mb_func == MB_FC_WRITE_MULTIPLE_REGISTERS) || (mb_func == MB_FC_WRITE_REGISTER))
		{
			mb_debug("RTU RX:", 6, len);
			if (wait_for_slave_write_response)
				ret = mb_func;
		}
        	//-------------------- Response Read Registers (3) --------------------
		else if (mb_func == MB_FC_READ_REGISTERS) 
		{
			mb_debug("RTU RX:", 6, len);
			if (wait_for_slave_read_response)
			{
				uint8_t pos = 0;
    			for (uint8_t i = 0; i < mb_buffer[8]; i += 2) 
				{
        			read_buffer[pos++] = (mb_buffer[9 + i] << 8) | mb_buffer[10 + i];
    			}
				ret = mb_func;
			}
		}
	}
	wait_for_slave_write_response = false;
	wait_for_slave_read_response = false;
	mb_led_on(true);
	return ret;
}
#endif

/*=================================================================================================*/
void wifi_manager_task(void)
{
	WiFiManager wm;    
	wm.setDebugOutput(false);  //  turn off debug output to hardware Serial
	//reset settings - for testing
	//wm.resetSettings();
	mb_led_on(false);
	// set configportal timeout
	wm.setConfigPortalTimeout(WIFI_MANAGER_TIMEOUT);

	if (wm.startConfigPortal(wifi_manager_name)) 
	{
		swSerial.println(WiFi.localIP());

	}
	else
	{
		swSerial.println("failed to connect and hit timeout");
		delay(1000);
		//reset and try again, or maybe put it to deep sleep
		ESP.restart();
	}

	//if you get here you have connected to the WiFi
	mb_led_on(true);
	dps_comm_reg = 0;
}

/*=================================================================================================*/
void wifi_manager_autoconnect(void)
{
	WiFiManager wifiManager;
	wifiManager.setDebugOutput(false); //  turn off debug output to hardware Serial
	// fetches ssid and pass from eeprom and tries to connect
	// if it does not connect it starts an access point with the specified name
	// and goes into a blocking loop awaiting configuration
	wifiManager.autoConnect(wifi_manager_name);
	// if you get here you have connected to the WiFi
	swSerial.println(WiFi.localIP());
/*
  	WiFi.begin("ZyXEL2","mypass12");   // start etehrnet interface
  	while (WiFi.status() != WL_CONNECTED) {
    	delay(1000);
    	Serial.print(".");
  	}
*/
  	mb_server.begin();
  	mb_server.setNoDelay(true);
}

/*=================================================================================================*/
void setup()
{

#ifdef MB_USE_LED
	pinMode(LED_BUILTIN, OUTPUT);
  	digitalWrite(LED_BUILTIN, HIGH); // LED off
#endif

	if (WIFI_RESET_BUTTON_PIN >= 0)
		pinMode(WIFI_RESET_BUTTON_PIN, INPUT_PULLUP);	

#ifdef DPS_ALTERNATIVE_FIRMWARE
	Serial.begin(115200);
#else
	Serial.begin(9600);
#endif

  	WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP  
	WiFi.hostname("DPS TCP server");
	WiFi.setOutputPower(10); // 0 - lowest output power (supply current ~70mA)  20 - highest output power (supply current ~85mA)

#ifndef DPS_ALTERNATIVE_FIRMWARE
	wifi_manager_autoconnect();
#endif

	Serial.flush(); // clear UART tx buffer
	Serial.swap();	// switch to UART2 GPIO15 (TX) and GPIO13 (RX)
	swSerial.begin(115200, SWSERIAL_8N1, -1, 1); // for debus messages via USB/UART0
	swSerial.enableRx(false);
	while (millis() < START_MODBUS_DELAY_MS) // delay 2 sec after power up
		delay(100);	
	mb_led_on(true);
}

/*=================================================================================================*/
void loop()
{
	static uint32_t wifi_reset_btn_time = millis();
	
	if (WiFi.status() == WL_CONNECTED) modbusTcpServerBridgeTask();
	if (wait_for_rtu_response) 	modbusRtuMasterBridgeTask();	

#ifdef DPS_ALTERNATIVE_FIRMWARE

	static bool dps_ready = false;
	static uint32_t last_time = 0;
	static uint32_t local_ip =0;
	static uint32_t old_ip = 0;
	uint32_t cur_time = millis();

	if (cur_time - last_time >= 1000) // one second timer
	{
		last_time = cur_time;	
		if (WiFi.status() == WL_CONNECTED) 
		{
			local_ip = WiFi.localIP();
		}
		else local_ip = 0;

		if (dps_ready)
		{
			if (old_ip != local_ip)
			{
				uint16_t reg[2]; 
				reg[0] = local_ip >> 16;
				reg[1] = local_ip & 0xFFFF;
				modbusRtuMasterWriteRegisters(44, 2, reg);	
			}
		}
		else 
		{
			dps_comm_reg = 0;
			modbusRtuMasterReadRegisters(18, 1, &dps_comm_reg);	
		}
	}

	if (wait_for_slave_read_response)
	{
		dps_ready =  modbusRtuMasterPoll() == MB_FC_READ_REGISTERS;
		if ((dps_comm_reg & DPS_COMM_WIFI_MODE) == DPS_COMM_WIFI_MODE) 
		{
			if ((dps_comm_reg & DPS_COMM_WIFI_RESET) == 0) 
			{
				wifi_manager_autoconnect();
			}
		}
		last_time = millis();
	}
	else if (wait_for_slave_write_response)
	{
		if (modbusRtuMasterPoll() == MB_FC_WRITE_MULTIPLE_REGISTERS) 
		{
			old_ip = local_ip;	
		}	
		last_time = millis();
	}

#endif

	if (WIFI_RESET_BUTTON_PIN >= 0) 
	{
		if ( digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) 
		{
			if ((millis() - wifi_reset_btn_time) > WIFI_RESET_BUTTON_PRESS_TIME_MS)
			{
				dps_comm_reg = DPS_COMM_WIFI_RESET;
			}
		}
		else 
			wifi_reset_btn_time = millis();

	}	

	if (dps_comm_reg & DPS_COMM_WIFI_RESET)
	{
		wifi_manager_task();	
	}
}

/*=================================================================================================*/
