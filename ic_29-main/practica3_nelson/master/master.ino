#include <SPI.h>             
#include <LoRa.h>
#include <Arduino_PMIC.h>
#include <RTCZero.h>

#define TX_LAPSE_MS          10000

RTCZero rtc;

// NOTA: Ajustar estas variables 
const uint8_t localAddress = 0xB0;     // DirecciÃ³n de este dispositivo
uint8_t destination = 0xFF;            // DirecciÃ³n de destino, 0xFF es la direcciÃ³n de broadcast

volatile bool txDoneFlag = true;       // Flag para indicar cuando ha finalizado una transmisiÃ³n
volatile bool transmitting = false;
volatile bool waitingAck = false;

// Estructura para almacenar la configuraciÃ³n de la radio
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower;
} LoRaConfig_t;

double bandwidth_kHz[10] = { 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };

LoRaConfig_t thisNodeConf = { 5, 8, 5, 2 };
LoRaConfig_t BackupConf = { 9, 7, 5, 2 };
LoRaConfig_t remoteNodeConf = { 0,  0, 0, 0 };
volatile int primerAjuste = 1;

int remoteRSSI = 0;
float remoteSNR = 0;

#define MIN_RSSI  -120
#define MIN_SNR -7
#define MAX_TX_POWER  20
#define MIN_TX_POWER 2 
#define MAX_BANDWIDTH_INDEX 9
#define MIN_BANDWIDTH_INDEX 0
#define MAX_SPREADING 12
#define MIN_SPREADING 7

uint8_t tiempoEspera = 2; //minutos
volatile bool sinRespuesta = false;


void ajustarConfiguraciones() {

  ajustarPotencia();
  ajustarFactorDispersion();
  gestionarAnchoBanda();
  modificarTasaCodificaciones();

}

void ajustarPotencia() {

  int ajuste = remoteRSSI > MIN_RSSI ?
  (thisNodeConf.txPower > MIN_TX_POWER ? -1 : 0)
  :(thisNodeConf.txPower < MAX_TX_POWER ? 1 : 0);
  actualizarPotenciaTx(ajuste);

}

void actualizarPotenciaTx(int cambio) {
  thisNodeConf.txPower += cambio;
  if (cambio != 0) {
    Serial.print(cambio > 0 ? "Potencia Tx aumentada" : "Potencia Tx disminuida");
    Serial.println(". Nueva potencia: ");
    Serial.println(thisNodeConf.txPower);
  }
}

void ajustarFactorDispersion() {
  int cambio = calcularCambioSpreading();
  if (cambio != 0) {
    thisNodeConf.spreadingFactor += cambio;
    Serial.print(cambio > 0 ? "Factor de dispersion aumentado" : "Factor de dispersion disminuido");
    Serial.print(". Nuevo factor: ");
    Serial.println(thisNodeConf.spreadingFactor);
  }
}

int calcularCambioSpreading() {
  if (remoteSNR > MIN_SNR && thisNodeConf.spreadingFactor > MIN_SPREADING) return -1;
  if (remoteSNR < MIN_SNR && thisNodeConf.spreadingFactor < MAX_SPREADING) return 1;
  return 0;
}

void gestionarAnchoBanda() {
  int delta = calcularCambiosBandwidth();/*(remoteRSSI > MIN_RSSI && thisNodeConf.bandwidth_index < 9) ? 1 : (remoteSNR < MIN_SNR < 0 ? -1 : 0);*/
  if (delta != 0) {
    thisNodeConf.bandwidth_index += delta;
    Serial.print(delta > 0 ? "Ancho de banda incrementado" : "Ancho de banda reducido");
    Serial.print(". Nuevo indice: ");
    Serial.println(thisNodeConf.bandwidth_index);
  }
}

int calcularCambiosBandwidth() {
  if (remoteSNR < MIN_SNR && thisNodeConf.bandwidth_index > MIN_BANDWIDTH_INDEX) return -1;
  if (remoteSNR > MIN_SNR && thisNodeConf.bandwidth_index < MAX_BANDWIDTH_INDEX) return 1;
  return 0;
}

void modificarTasaCodificaciones() {
  /*int ajuste = (remoteSNR > 0 && thisNodeConf.codingRate > 5) ? -1 : (remoteSNR < 0 && thisNodeConf.codingRate < 8) ? 1 : 0;
  if(ajuste != 0){
    thisNodeConf.codingRate += ajuste;
    Serial.print(ajuste > 0 ? "Tasa de codificacion incrementada" : "Tasa de codificacion reducida");
    Serial.println(". Nueva tasa: ");*/
  // Serial.print(thisNodeConf.codingRate);
}


void actualizarConfiguraciones() {
  Serial.print("Actualizando parametros ");
  ajustarConfiguraciones();
  reiniciarTiempoRTC();
}

void restaurarConfiguracion() {
  Serial.println("Reestableciendo configuracion principal.");
  thisNodeConf = BackupConf;
  sinRespuesta = false;
  aplicarParametrosLora();
  reiniciarTiempoRTC();
}

void NoRespuesta() {
  sinRespuesta = true;
}

void reiniciarTiempoRTC() {
  Serial.println("Reiniciando temporizador.");
  rtc.setMinutes(0);
}

void aplicarParametrosLora() {
  Serial.println("Aplicando configuracion de comunicacion LoRa.");
  ajustarAnchoBanda();
  establecerFactorSpreading();
  definirTasaCodificacion();
  configurarPotenciaTransmision();
}

void ajustarAnchoBanda() {
  double anchoBanda = obtenerAnchoBanda(thisNodeConf.bandwidth_index);
  LoRa.setSignalBandwidth(anchoBanda);
}

double obtenerAnchoBanda(int indice) {
  return bandwidth_kHz[indice] * 2;
}

void establecerFactorSpreading() {
  LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);
}

void definirTasaCodificacion() {
  LoRa.setCodingRate4(thisNodeConf.codingRate);
}

void configurarPotenciaTransmision() {
  LoRa.setTxPower(thisNodeConf.txPower, PA_OUTPUT_PA_BOOST_PIN);
}

// --------------------------------------------------------------------
// Setup function
// --------------------------------------------------------------------
void setup()
{
  Serial.begin(9600);
  while (!Serial) {}

  Serial.println("LoRa Duplex with TxDone and Receive callbacks");
  Serial.println("Using binary packets");

  // Es posible indicar los pines para CS, reset e IRQ pins (opcional)
  // LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  }
  else {
    Serial.println("Initilization of BQ24195L succeeded!");
  }

  if (!LoRa.begin(868E6)) {      // Initicializa LoRa a 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }

  aplicarParametrosLora();
  LoRa.setSyncWord(0x12);         // Palabra de sincronizaciÃ³n privada por defecto para SX127X 
  // Usaremos la palabra de sincronizaciÃ³n para crear diferentes
  // redes privadas por equipos
  LoRa.setPreambleLength(8);      // NÃºmero de sÃ­mbolos a usar como preÃ¡mbulo


  // Indicamos el callback para cuando se reciba un paquete
  LoRa.onReceive(onReceive);

  // Activamos el callback que nos indicarÃ¡ cuando ha finalizado la 
  // transmisiÃ³n de un mensaje
  LoRa.onTxDone(TxFinished);

  // NÃ³tese que la recepciÃ³n estÃ¡ activada a partir de este punto
  LoRa.receive();

  Serial.println("LoRa init succeeded.\n");

  //Temporizador
  rtc.begin();
  rtc.setMinutes(0);
  rtc.enableAlarm(rtc.MATCH_MMSS);
  rtc.setAlarmMinutes(tiempoEspera);
  rtc.attachInterrupt(&NoRespuesta);

  Serial.print("Temporizador iniciado: " + String(tiempoEspera));
  Serial.println(" Minutos sin recibir datos para restaurar sistema.");
}

// --------------------------------------------------------------------
// Loop function
// --------------------------------------------------------------------
void loop()
{
  static uint32_t lastSendTime_ms = 0;
  static uint16_t msgCount = 0;
  static uint32_t txInterval_ms = TX_LAPSE_MS;
  static uint32_t tx_begin_ms = 0;

  if (!transmitting && ((millis() - lastSendTime_ms) > txInterval_ms)) {

    uint8_t payload[50];
    uint8_t payloadLength = 0;

    payload[payloadLength] = (thisNodeConf.bandwidth_index << 4);
    payload[payloadLength++] |= ((thisNodeConf.spreadingFactor - 6) << 1);
    payload[payloadLength] = ((thisNodeConf.codingRate - 5) << 6);
    payload[payloadLength++] |= ((thisNodeConf.txPower - 2) << 1);

    // Incluimos el RSSI y el SNR del Ãºltimo paquete recibido
    // RSSI puede estar en un rango de [0, -127] dBm
    payload[payloadLength++] = uint8_t(-LoRa.packetRssi() * 2);
    // SNR puede estar en un rango de [20, -148] dBm
    payload[payloadLength++] = uint8_t(148 + LoRa.packetSnr());

    transmitting = true;
    txDoneFlag = false;
    tx_begin_ms = millis();

    sendMessage(payload, payloadLength, msgCount);
    Serial.print("Sending packet ");
    Serial.print(msgCount++);
    Serial.print(": ");
    printBinaryPayload(payload, payloadLength);
  }

  if (transmitting && txDoneFlag) {
    uint32_t TxTime_ms = millis() - tx_begin_ms;
    Serial.print("----> TX completed in ");
    Serial.print(TxTime_ms);
    Serial.println(" msecs");

    // Ajustamos txInterval_ms para respetar un duty cycle del 1% 
    uint32_t lapse_ms = tx_begin_ms - lastSendTime_ms;
    lastSendTime_ms = tx_begin_ms;
    float duty_cycle = (100.0f * TxTime_ms) / lapse_ms;

    Serial.print("Duty cycle: ");
    Serial.print(duty_cycle, 1);
    Serial.println(" %\n");

    // Solo si el ciclo de trabajo es superior al 1% lo ajustamos
    if (duty_cycle > 1.0f) {
      txInterval_ms = TxTime_ms * 100;
    }

    transmitting = false;

    // Reactivamos la recepciÃ³n de mensajes, que se desactiva
    // en segundo plano mientras se transmite
    Serial.println("Estoy escuchando");
    LoRa.receive();

  }

  if (sinRespuesta) {
    Serial.println("Se ha disparado el timer, restaurando la configuración a la anterior");
    restaurarConfiguracion();
  }
}

// --------------------------------------------------------------------
// Sending message function
// --------------------------------------------------------------------
void sendMessage(uint8_t* payload, uint8_t payloadLength, uint16_t msgCount)
{
  Serial.println("Estoy enviando un mensaje");
  while (!LoRa.beginPacket()) {            // Comenzamos el empaquetado del mensaje
    delay(10);                            // 
  }
  LoRa.write(destination);                // AÃ±adimos el ID del destinatario
  LoRa.write(localAddress);               // AÃ±adimos el ID del remitente
  LoRa.write((uint8_t)(msgCount >> 7));   // AÃ±adimos el Id del mensaje (MSB primero)
  LoRa.write((uint8_t)(msgCount & 0xFF));
  LoRa.write(payloadLength);              // AÃ±adimos la longitud en bytes del mensaje
  LoRa.write(payload, (size_t)payloadLength); // AÃ±adimos el mensaje/payload 
  LoRa.endPacket(true);                   // Finalizamos el paquete, pero no esperamos a
  // finalice su transmisiÃ³n

  delay(100);

}

// --------------------------------------------------------------------
// Receiving message function
// --------------------------------------------------------------------
void onReceive(int packetSize)
{
  if (transmitting && !txDoneFlag) txDoneFlag = true;

  if (packetSize == 0) return;          // Si no hay mensajes, retornamos

  // Leemos los primeros bytes del mensaje
  uint8_t buffer[10];                   // Buffer para almacenar el mensaje
  int recipient = LoRa.read();          // DirecciÃ³n del destinatario
  uint8_t sender = LoRa.read();         // DirecciÃ³n del remitente
  // msg ID (High Byte first)
  uint16_t incomingMsgId = ((uint16_t)LoRa.read() << 7) |
    (uint16_t)LoRa.read();

  uint8_t incomingLength = LoRa.read(); // Longitud en bytes del mensaje

  uint8_t receivedBytes = 0;            // Leemos el mensaje byte a byte
  while (LoRa.available() && (receivedBytes < uint8_t(sizeof(buffer) - 1))) {
    buffer[receivedBytes++] = (char)LoRa.read();
  }

  if (incomingLength != 0 && incomingLength != receivedBytes) {// Verificamos la longitud del mensaje
    Serial.print("Receiving error: declared message length " + String(incomingLength));
    Serial.println(" does not match length " + String(receivedBytes));
    return;
  }

  // Verificamos si se trata de un mensaje en broadcast o es un mensaje
  // dirigido especÃ­ficamente a este dispositivo.
  // NÃ³tese que este mecanismo es complementario al uso de la misma
  // SyncWord y solo tiene sentido si hay mÃ¡s de dos receptores activos
  // compartiendo la misma palabra de sincronizaciÃ³n
  if ((recipient & localAddress) != localAddress) {
    Serial.println("Receiving error: This message is not for me.");
    return;
  }
  Serial.println("---------Mensaje----------");
  // Imprimimos los detalles del mensaje recibido
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Payload length: " + String(incomingLength));
  Serial.print("Payload: ");
  printBinaryPayload(buffer, receivedBytes);
  Serial.print("\nRSSI: " + String(LoRa.packetRssi()));
  Serial.print(" dBm\nSNR: " + String(LoRa.packetSnr()));
  Serial.println(" dB");


  // Actualizamos remoteNodeConf y lo mostramos
  if (incomingLength == 0) {
    Serial.println("Ack Received!");
  }
  else if (incomingLength == 4) {
    remoteNodeConf.bandwidth_index = buffer[0] >> 4;
    remoteNodeConf.spreadingFactor = 6 + ((buffer[0] & 0x0F) >> 1);
    remoteNodeConf.codingRate = 5 + (buffer[1] >> 6);
    remoteNodeConf.txPower = 2 + ((buffer[1] & 0x3F) >> 1);
    remoteRSSI = -int(buffer[2]) / 2.0f;
    remoteSNR = int(buffer[3]) - 148;

    Serial.print("Remote config: BW: ");
    Serial.print(bandwidth_kHz[remoteNodeConf.bandwidth_index]);
    Serial.print(" kHz, SPF: ");
    Serial.print(remoteNodeConf.spreadingFactor);
    Serial.print(", CR: ");
    Serial.print(remoteNodeConf.codingRate);
    Serial.print(", TxPwr: ");
    Serial.print(remoteNodeConf.txPower);
    Serial.print(" dBm, RSSI: ");
    Serial.print(remoteRSSI);
    Serial.print(" dBm, SNR: ");
    Serial.print(remoteSNR, 1);
    Serial.println(" dB\n");
    Serial.println("---------Fin del mensaje----------");
    Serial.println("Se actualiza la configuración del Nodo");
    actualizarConfiguraciones();
    primerAjuste = 0;

  }
  else {
    Serial.print("Unexpected payload size: ");
    Serial.print(receivedBytes);
    Serial.println(" bytes\n");
  }
}

void TxFinished()
{
  if (primerAjuste == 0) {
    Serial.println("Se modifica la configuracion");
    aplicarParametrosLora();
  }
  txDoneFlag = true;

}

void printBinaryPayload(uint8_t* payload, uint8_t payloadLength)
{
  for (int i = 0; i < payloadLength; i++) {
    Serial.print((payload[i] & 0xF0) >> 4, HEX);
    Serial.print(payload[i] & 0x0F, HEX);
    Serial.print(" ");
  }
}
