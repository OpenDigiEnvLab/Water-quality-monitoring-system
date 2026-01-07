// WATER QUALITY MONITORING SYSTEM
// Biofloc-based recirculating aquaculture (B-RAS)
// On-device TSS prediction (linear (simple) + multiple regression)
// NOTE: Coefficients are dataset-specific. Re-fit for your system and update here or via serial.
// Robel Kahsu

#include <ModbusMaster.h>
#include <Notecard.h>
#include <Wire.h>
#include "wiring_private.h"
#include <Ezo_i2c.h>
#include <Ezo_i2c_util.h>

ModbusMaster node;
Notecard notecard;

#define PRODUCT_UID "user_email:biofloc_aquaculture"
#define NOTECARD_I2C_ADDRESS 0x17
#define SLAVE_ID 1

const uint16_t TURBIDITY_REGISTER = 0x0000;
const uint16_t CORRECTION_FACTOR_REGISTER = 0x0004;

Uart Serial2(&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
void SERCOM1_Handler() { Serial2.IrqHandler(); }

unsigned long lastReadTime = 0;
const unsigned long readInterval = 600000;  // 10-minute interval

Ezo_board ORP(0x62, "ORP");
Ezo_board DO(0x61, "DO");
Ezo_board pH(0x63, "pH");
Ezo_board Ec(0x64, "Ec");
Ezo_board atlasTemp(0x66, "Temp");

float Ec_float = 0.0, Temp_float = 0.0, pH_float = 0.0, DO_float = 0.0, ORP_float = 0.0;
String Time;

// ---------- TSS models (from figure; edit or set via serial) ----------
float LIN_b0 = 481.196f;   // Simple:   TSS = 481.196 + 5.122*NTU
float LIN_b1 = 5.122f;

float MLR_b0 = 12336.052f; // Multiple: TSS = 12336.052 + 6.042*NTU + 94.312*EC(mS/cm) - 430.775*T
float MLR_b1 = 6.042f;
float MLR_b2 = 94.312f;    // EC in mS/cm (Ec_float / 1000.0)
float MLR_b3 = -430.775f;  // Temperature in °C

static inline bool validFloat(float v){ return !isnan(v) && isfinite(v); }

float tssSimple(float ntu){
  if(!validFloat(ntu)) return NAN;
  return LIN_b0 + LIN_b1 * ntu;
}

float tssMultiple(float ntu, float ec_mS, float T_C){
  if(!validFloat(ntu) || !validFloat(ec_mS) || !validFloat(T_C)) return NAN;
  return MLR_b0 + MLR_b1 * ntu + MLR_b2 * ec_mS + MLR_b3 * T_C;
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);

  node.begin(SLAVE_ID, Serial2);
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

  Wire.begin();
  notecard.begin(Serial1, 9600);
  notecard.setDebugOutputStream(Serial);

  J *req = notecard.newRequest("hub.set");
  if (req) {
    JAddStringToObject(req, "product", PRODUCT_UID);
    JAddStringToObject(req, "mode", "periodic");
    notecard.sendRequest(req);
  }

  lastReadTime = millis() - readInterval;
}

void loop() {
  if (millis() - lastReadTime >= readInterval) {
    lastReadTime = millis();

    unsigned long epochTime = getNotecardTime();
    Time = convertEpochToTime(epochTime);

    readTemperature();
    readSensors();
    float turbidity = getCurrentTurbidity();

    // EC from Atlas is µS/cm; convert to mS/cm for the equation.
    float ec_mS = validFloat(Ec_float) ? (Ec_float / 1000.0f) : NAN;

    float tss_lin = tssSimple(turbidity);
    float tss_mlr = tssMultiple(turbidity, ec_mS, Temp_float);

    Serial.print("EC (µS/cm): "); Serial.print(Ec_float, 1);
    Serial.print(" | EC (mS/cm): "); Serial.print(ec_mS, 3);
    Serial.print(" | TSS_simple: "); Serial.print(tss_lin, 1);
    Serial.print(" | TSS_multiple: "); Serial.println(tss_mlr, 1);

    sendToNotecard(turbidity, ec_mS, tss_lin, tss_mlr);
  }

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    processCommand(input);
  }
}

void readTemperature() {
  atlasTemp.send_read_cmd();
  delay(600);
  if (atlasTemp.receive_read_cmd() == Ezo_board::SUCCESS) {
    Temp_float = atlasTemp.get_last_received_reading();
    Serial.print("Temperature (C): "); Serial.println(Temp_float, 2);
  } else {
    Serial.println("Failed to read temperature");
    Temp_float = NAN;
  }
}

void readSensors() {
  float temp_comp = (Temp_float > 0.0f && Temp_float < 100.0f) ? Temp_float : 25.0f;
  Serial.print("Compensation temperature: "); Serial.println(temp_comp, 2);

  Ec.send_read_with_temp_comp(temp_comp);
  delay(900);
  if (Ec.receive_read_cmd() == Ezo_board::SUCCESS) Ec_float = Ec.get_last_received_reading();
  else { Serial.println("Failed EC"); Ec_float = NAN; }
  Serial.print("EC (µS/cm): "); Serial.println(Ec_float, 1);

  DO.send_read_with_temp_comp(temp_comp);
  delay(900);
  if (DO.receive_read_cmd() == Ezo_board::SUCCESS) DO_float = DO.get_last_received_reading();
  else { Serial.println("Failed DO"); DO_float = NAN; }

  pH.send_read_with_temp_comp(temp_comp);
  delay(900);
  if (pH.receive_read_cmd() == Ezo_board::SUCCESS) pH_float = pH.get_last_received_reading();
  else { Serial.println("Failed pH"); pH_float = NAN; }

  ORP.send_read_cmd();
  delay(900);
  if (ORP.receive_read_cmd() == Ezo_board::SUCCESS) ORP_float = ORP.get_last_received_reading();
  else { Serial.println("Failed ORP"); ORP_float = NAN; }
}

void processCommand(String input) {
  input.trim(); input.toLowerCase();

  if (input.startsWith("corr,")) {
    float factor = input.substring(5).toFloat();
    if (factor > 0) sendCorrectionFactor(factor);
    else Serial.println("Invalid correction factor.");
    return;
  }

  if (input.startsWith("setlin,")) {
    float b0, b1;
    if (sscanf(input.c_str(), "setlin,%f,%f", &b0, &b1) == 2) {
      LIN_b0 = b0; LIN_b1 = b1;
      Serial.print("LIN set: b0="); Serial.print(LIN_b0);
      Serial.print(", b1="); Serial.println(LIN_b1);
    } else Serial.println("Format: setlin,b0,b1");
    return;
  }

  if (input.startsWith("setmlr,")) {
    float b0, b1, b2, b3;
    if (sscanf(input.c_str(), "setmlr,%f,%f,%f,%f", &b0, &b1, &b2, &b3) == 4) {
      MLR_b0=b0; MLR_b1=b1; MLR_b2=b2; MLR_b3=b3;
      Serial.print("MLR set: b0="); Serial.print(MLR_b0);
      Serial.print(", b1="); Serial.print(MLR_b1);
      Serial.print(", b2="); Serial.print(MLR_b2);
      Serial.print(", b3="); Serial.println(MLR_b3);
    } else Serial.println("Format: setmlr,b0,b1,b2,b3");
    return;
  }

  Serial.println("Commands: corr,<f> | setlin,b0,b1 | setmlr,b0,b1,b2,b3");
}

void sendCorrectionFactor(float factor) {
  uint32_t rawValue = *(uint32_t*)&factor;
  uint16_t buf[2] = { (uint16_t)(rawValue & 0xFFFF), (uint16_t)((rawValue >> 16) & 0xFFFF) };
  node.setTransmitBuffer(0, buf[0]);
  node.setTransmitBuffer(1, buf[1]);
  uint8_t result = node.writeMultipleRegisters(CORRECTION_FACTOR_REGISTER, 2);
  if (result == node.ku8MBSuccess) Serial.println("Correction factor written.");
  else { Serial.print("Write CF error "); Serial.println(result, HEX); printErrorDetails(result); }
}

float getCurrentTurbidity() {
  uint8_t result = node.readHoldingRegisters(TURBIDITY_REGISTER, 2);
  if (result == node.ku8MBSuccess) {
    uint16_t r1 = node.getResponseBuffer(0);
    uint16_t r2 = node.getResponseBuffer(1);
    uint32_t combined = ((uint32_t)r2 << 16) | r1;
    float turbidity = *(float*)&combined;
    Serial.print("Turbidity (NTU): "); Serial.println(turbidity, 2);
    return turbidity;
  } else {
    Serial.print("Read turbidity error "); Serial.println(result, HEX);
    printErrorDetails(result);
    return NAN;
  }
}

void sendToNotecard(float turbidity, float ec_mS, float tss_lin, float tss_mlr) {
  J *req = NoteNewRequest("note.add");
  if (!req) { Serial.println("note.add alloc failed"); return; }
  JAddStringToObject(req, "file", "turbidity.qo");
  JAddBoolToObject(req, "sync", true);

  J *body = JCreateObject();
  JAddStringToObject(body, "timestamp", Time.c_str());
  JAddNumberToObject(body, "turbidity", turbidity);
  JAddNumberToObject(body, "ec_uScm", Ec_float);  // raw EC (µS/cm)
  JAddNumberToObject(body, "ec_mScm", ec_mS);     // EC for model (mS/cm)
  JAddNumberToObject(body, "temp", Temp_float);
  JAddNumberToObject(body, "DO", DO_float);
  JAddNumberToObject(body, "pH", pH_float);
  JAddNumberToObject(body, "ORP", ORP_float);

  if (validFloat(tss_lin)) JAddNumberToObject(body, "tss_simple", tss_lin);
  if (validFloat(tss_mlr)) JAddNumberToObject(body, "tss_multiple", tss_mlr);

  JAddItemToObject(req, "body", body);
  if (!NoteRequest(req)) Serial.println("Failed to send data to Notecard.");
  else Serial.println("Data sent to Notecard.");
}

unsigned long getNotecardTime() {
  J *req = notecard.newRequest("card.time");
  J *rsp = (req) ? notecard.requestAndResponse(req) : NULL;
  if (!rsp) { Serial.println("Failed to get time"); return 0; }
  unsigned long epochTime = JGetNumber(rsp, "time");
  notecard.deleteResponse(rsp);
  return epochTime;
}

String convertEpochToTime(unsigned long epochTime) {
  if (epochTime == 0) return "Invalid Time";
  epochTime += 3 * 3600;  // local offset (UTC+3)

  int s = epochTime % 60;
  int m = (epochTime / 60) % 60;
  int h = (epochTime / 3600) % 24;
  int days = epochTime / 86400;

  int year = 1970;
  auto leap = [](int y){ return ((y%4==0 && y%100!=0) || (y%400==0)); };
  while (days >= (leap(year) ? 366 : 365)) { days -= (leap(year) ? 366 : 365); year++; }

  int md[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (leap(year)) md[1] = 29;
  int mon = 0; while (days >= md[mon]) { days -= md[mon]; mon++; }
  int day = days + 1;

  char buf[20];
  sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d", year, mon+1, day, h, m, s);
  return String(buf);
}

void printErrorDetails(uint8_t error) {
  switch (error) {
    case node.ku8MBIllegalFunction:    Serial.println("Error: Illegal function."); break;
    case node.ku8MBIllegalDataAddress: Serial.println("Error: Illegal data address."); break;
    case node.ku8MBIllegalDataValue:   Serial.println("Error: Illegal data value."); break;
    case node.ku8MBSlaveDeviceFailure: Serial.println("Error: Slave device failure."); break;
    case node.ku8MBInvalidSlaveID:     Serial.println("Error: Invalid slave ID."); break;
    case node.ku8MBInvalidFunction:    Serial.println("Error: Invalid function."); break;
    case node.ku8MBResponseTimedOut:   Serial.println("Error: Response timed out."); break;
    case node.ku8MBInvalidCRC:         Serial.println("Error: Invalid CRC."); break;
    default:                           Serial.println("Error: Unknown error."); break;
  }
}
