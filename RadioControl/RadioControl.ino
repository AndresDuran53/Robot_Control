#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

/************ Radio Setup ***************/

// Frecuencia de Comunicacion
#define RF69_FREQ 915.0

// Direcion del Robot
#define DEST_ADDRESS   5
// Direcion del Control (Propia)
#define MY_ADDRESS     2


#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
  #define LED           13
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT     3  // 
  #define RFM69_CS      4  //
  #define RFM69_RST     2  // "A"
  #define LED           13
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_IRQ     15   // "B"
  #define RFM69_RST     16   // "D"
  #define LED           0
#endif

#if defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
  #define LED           13
#endif

// Instancia Singleton del Driver de Radio
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Clase para manejar el envio y recepcion de mensajes, usando el driver declarado abajo
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

int16_t packetnum = 0;  // contador de paquetes, incrementa una vez por envio

void setup() 
{
  Serial.begin(115200);

  pinMode(A0, INPUT); //Entrada Analogica Joystick Y
  pinMode(A1, INPUT); //Entrada Analogica Joystick X
  pinMode(12, INPUT_PULLUP); //Entrada Digital Boton Josystick
  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  //Serial.println("Feather Addressed RFM69 TX Test!");
  //Serial.println();

  // Reset Manual
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init Fallo");
    while (1);
  }
  //Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency Fallo");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // Llave de cifrado.
  uint8_t key[] = { 0xb9, 0xbc, 0xf9, 0xf0, 0xea, 0x26, 0x5d, 0xde,
                    0x2a, 0x91, 0x6d, 0x10, 0xb8, 0x3c, 0xf1, 0x3d};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);
  
  //Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";

void loop(){
  
  int posX=map(analogRead(A1),0,1023,0,255);
  int posY=map(analogRead(A0),0,1023,0,255);
  int botonPress=digitalRead(12);
  
  //Serial.print("X: ");
  //Serial.println(posX);
  //Serial.print("Y: ");
  //Serial.println(posY);
  //Serial.print("B: ");
  //Serial.println(botonPress);
  
  char radiopacket[4];
  radiopacket[0]=(char)posX;
  radiopacket[1]=(char)posY;
  radiopacket[2]=(char)botonPress;
  
  //Enviamos el Mensaje
  if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS)) {
    // Esperamos la respuesta del Robot
    uint8_t len = sizeof(buf);
    uint8_t from;   
    if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
      buf[len] = 0; // zero out remaining string      
      //Serial.print(" [RSSI :");
      //Serial.print(rf69.lastRssi());
      //Serial.print("] : ");
      //Serial.println((char*)buf);     
    } else {
      //Serial.println("No reply, is anyone listening?");
    }
  } else {
    //Serial.println("Sending failed (no ack)");
  }
  delay(10);
}
