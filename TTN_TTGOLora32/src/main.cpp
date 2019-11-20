// MIT License
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
// Modified by github FGCDAM user https://github.com/fcgdam

#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include <SPI.h>
#include <SSD1306.h>
#include "soc/efuse_reg.h"

#define LEDPIN 2

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

unsigned int counter = 0;

SSD1306 display (OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

/*************************************
 * TODO: Change the following keys
 * NwkSKey: network session key, AppSKey: application session key, and DevAddr: end-device address
 *************************************/
static u1_t NWKSKEY[16] = { ... };  // Paste here the key in MSB format

static u1_t APPSKEY[16] = { ... };  // Paste here the key in MSB format

static u4_t DEVADDR = 0x00000000;   // Put here the device id in hexadecimal form.

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;
char TTN_response[30];

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

void do_send(osjob_t* j){
    // Payload to send (uplink)
    static uint8_t message[] = "Hello World!";

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, message, sizeof(message)-1, 0);
        Serial.println(F("Sending uplink packet..."));
        digitalWrite(LEDPIN, HIGH);
        display.clear();
        display.drawString (0, 0, "Sending uplink packet...");
        display.drawString (0, 50, String (++counter));
        display.display ();
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    if (ev == EV_TXCOMPLETE) {
        display.clear();
        display.drawString (0, 0, "EV_TXCOMPLETE event!");


        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK) {
          Serial.println(F("Received ack"));
          display.drawString (0, 20, "Received ACK.");
        }

        if (LMIC.dataLen) {
          int i = 0;
          // data received in rx slot after tx
          Serial.print(F("Data Received: "));
          Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
          Serial.println();

          display.drawString (0, 20, "Received DATA.");
          for ( i = 0 ; i < LMIC.dataLen ; i++ )
            TTN_response[i] = LMIC.frame[LMIC.dataBeg+i];

          TTN_response[i] = 0;
          display.drawString (0, 32, String(TTN_response));
        }

        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
        digitalWrite(LEDPIN, LOW);
        display.drawString (0, 50, String (counter));
        display.display ();
    }
}

int getChipRevision()
{
  return (REG_READ(EFUSE_BLK0_RDATA3_REG) >> (EFUSE_RD_CHIP_VER_REV1_S)&&EFUSE_RD_CHIP_VER_REV1_V) ;
}

void printESPRevision() {
  Serial.print("REG_READ(EFUSE_BLK0_RDATA3_REG): ");
  Serial.println(REG_READ(EFUSE_BLK0_RDATA3_REG), BIN);

  Serial.print("EFUSE_RD_CHIP_VER_REV1_S: ");
  Serial.println(EFUSE_RD_CHIP_VER_REV1_S, BIN);

  Serial.print("EFUSE_RD_CHIP_VER_REV1_V: ");
  Serial.println(EFUSE_RD_CHIP_VER_REV1_V, BIN);

  Serial.println();

  Serial.print("Chip Revision (official version): ");
  Serial.println(getChipRevision());

  Serial.print("Chip Revision from shift Operation ");
  Serial.println(REG_READ(EFUSE_BLK0_RDATA3_REG) >> 15, BIN);

}

void setup() {
    Serial.begin(115200);
    delay(1500);   // Give time for the seral monitor to start up
    Serial.println(F("Starting..."));

    printESPRevision();

    // Use the Blue pin to signal transmission.
    pinMode(LEDPIN,OUTPUT);

   // reset the OLED
    pinMode(OLED_RESET,OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(50);
    digitalWrite(OLED_RESET, HIGH);

    display.init ();
    display.flipScreenVertically ();
    display.setFont (ArialMT_Plain_10);

    display.setTextAlignment (TEXT_ALIGN_LEFT);

    display.drawString (0, 0, "Init!");
    display.display ();
    
    if ( DEVADDR == 0x00 )   {  // Device is misconfigured with an invalid or not defined device id.
        int i = 0;

        while ( true ) {
            display.clear();
            display.drawString (0, 0, "INIT FAILED! ");
            display.drawString (0, 9, "Device ID is not SET!");
          
            display.drawString (0, 18, "Invalid TTN settings!");
            display.drawString (0, 27, "Set NWSKEY and APPSKEY!");
        
            display.drawString( 0, 36, "Correct and reset.");

            display.drawString( 0, 45, "Waiting for user action...");
            if ( i == 0 )
                display.drawString( 0, 54, "." );
            else 
                display.drawString( 0, 54, "*" );
            i++; 
            i = i % 2; 
            display.display ();
            delay( 1000 );
        }

    }

    // LMIC init
    os_init();

    display.drawString (0, 8, "LMIC was initiated!");
    display.display ();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    display.drawString (0, 16, "LMIC reset -> OK!");
    display.display ();

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    display.drawString (0, 24, "LMIC setChannels -> OK!");
    display.display ();


    // Set static session parameters.
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    //LMIC_setDrTxpow(DR_SF11,14);
    LMIC_setDrTxpow(DR_SF9,14);

    display.drawString (0, 32, "LMIC setup -> OK!");
    display.display ();

    for ( int i = 0 ; i < 5 ; i++ ) {
        delay ( 1000 );
        
        display.drawString (0+i*8, 50, ".");
        display.display ();
    }

    Serial.println("LMIC setup done.");
    Serial.println("Starting transmit job.");

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
