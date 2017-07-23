#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

/************ Radio Setup ***************/

// Frecuencia de Comunicacion
#define RF69_FREQ 915.0

// Direcion del Robot (Propia)
#define MY_ADDRESS     5

#if defined(ARDUINO_SAMD_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     3
  #define RFM69_RST     4
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

  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // Reset Manual
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init Fallo");
    while (1);
  }
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  if (!rf69.setFrequency(RF69_FREQ)) {
    //Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // Llave de cifrado.
  uint8_t key[] = { 0xb9, 0xbc, 0xf9, 0xf0, 0xea, 0x26, 0x5d, 0xde,
                    0x2a, 0x91, 0x6d, 0x10, 0xb8, 0x3c, 0xf1, 0x3d};
  rf69.setEncryptionKey(key);
  
  pinMode(LED, OUTPUT);
  
}

uint8_t data[] = "k";
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void loop() {
  if (rf69_manager.available())
  {
    // Espera un mensaje entrante
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rf69_manager.recvfromAck(buf, &len, &from)) {
      buf[len] = 0;
      if(from==2){ //Valida que el texto venga del Control
        //Serial.print("Paquete Recibido del #"); Serial.print(from);
        //Serial.print(" [RSSI :");
        //Serial.print(rf69.lastRssi());
        //Serial.print("] : ");
        //Serial.println((char*)buf);
        //Serial.print("X: ");
        //Serial.println((int)buf[0]);
        //Serial.print("Y: ");
        //Serial.println((int)buf[1]);
        //Serial.print("B: ");
        //Serial.println((int)buf[2]);
        if (!rf69_manager.sendtoWait(data, sizeof(data), from))
          Serial.println("Sending failed (no ack)");
        if((int)buf[2]) 
          digitalWrite(LED,LOW);
        else
          digitalWrite(LED,HIGH);
      }
    }
  }
}
