#include "RF433send.h"
#include "RF433any.h"

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Define captured command code signals here
constexpr byte A_On[] = {0xc4, 0x6e, 0x70};
constexpr byte A_Off[] = {0xc9, 0x8c, 0x10};
constexpr byte B_On[] = {0xc6, 0x9f, 0xb4};
constexpr byte B_Off[] = {0xcd, 0xfa, 0x24};
constexpr byte C_On[] = {0xc6, 0x9f, 0xbc};
constexpr byte C_Off[] = {0xca, 0x41, 0xac};
constexpr byte D_On[] = {0xca, 0x41, 0xa2};
constexpr byte D_Off[] = {0xc4, 0x6e, 0x72};
constexpr byte Master_On[] = {0xca, 0x41, 0xaa};
constexpr byte Master_Off[] = {0xc4, 0x6e, 0x7a};

constexpr byte GATE_OPEN[] = { 0x0D, 0x76 };

// GLOBALS
// GPIO pin attached to RF transmitter output
constexpr uint8_t txPin =  47;
constexpr uint8_t rxPin = 45;

// RF433 "Track" object used to decode signals
Track rfReceiver(rxPin);

// Create a sender object to emulate the transmitting device
// We'll populate the protocol it uses in setup() later
RfSend *tx_device;
RfSend *tx_device_gate;


Adafruit_VL53L0X lox = Adafruit_VL53L0X();



// Create a buffer to store commands input from serial connection
char cmdBuffer[32];
uint8_t cmdIndex = 0;

// Debug heartbeat
unsigned long lastDbg = 0;
const unsigned long DBG_INTERVAL = 1000; // ms


// Convert encoding code to name
const char *getEncodingName(char c) {
    if (c == 'T')
        return "RFMOD_TRIBIT";
    else if (c == 'N')
        return "RFMOD_TRIBIT_INVERTED";
    else if (c == 'M')
        return "RFMOD_MANCHESTER";
    return "UNKNOWN ENCODING";
}


/**
 * Print decoded data
 */
void printDecodedFrame(Decoder* decoder) {
    if (!decoder) return;
    // Number of bits in frame
    //int bits = decoder->get_nb_bits();

    BitVector* bitData = decoder->take_away_data();
    if (!bitData) return;

    int bits = bitData->get_nb_bits();

    char* payload = nullptr;
    if (bitData) {
        payload = bitData->to_str();
    }
    // Extract timing information
    TimingsExt t;
    decoder->get_tsext(&t);
    // Get name of encoding system
    const char* enc = getEncodingName(decoder->get_id_letter());
    // Print values
    Serial.println();
    Serial.println("mod\t\tinit\tlo_pre\thi_pre\tign\tloS\tloL\thiS\thiL\tlast\tsep\tbits\tpayload(hex)\tpayload(bits)");
    Serial.print(enc);               Serial.print("\t");
    Serial.print(t.initseq);         Serial.print("\t");
    Serial.print(t.first_low);       Serial.print("\t");
    Serial.print(t.first_high);      Serial.print("\t");
    Serial.print(t.first_low_ignored); Serial.print("\t");
    Serial.print(t.low_short);       Serial.print("\t");
    Serial.print(t.low_long);        Serial.print("\t");
    Serial.print(t.high_short);      Serial.print("\t");
    Serial.print(t.high_long);       Serial.print("\t");
    Serial.print(t.last_low);        Serial.print("\t");
    Serial.print(t.sep);             Serial.print("\t");
    Serial.print(bits);             Serial.print("\t");
    Serial.print(payload);             Serial.print("\t");
    for (int i = bits - 1; i >= 0; --i) {   // MSB first
      Serial.print(bitData->get_nth_bit(i) ? '1' : '0');
      // Group bytes visually
      if ((i % 8) == 0 && i != 0) Serial.print(' ');
    }
    Serial.println();

    // Cleanup
    if (payload) free(payload);
    if (bitData) delete bitData;
}


// Send the specfied code
void sendCode(const byte *code) {
    byte n = tx_device->send(3, code);
    Serial.println("Sent RF packet ");
}

// Invia il comando cancello
void sendGate() { 
    tx_device_gate->send(2, GATE_OPEN); // 2 byte (12 bit usati) Serial.println("Gate signal sent"); 
    Serial.print("Segnale Inviato");
}

// Read serial command
void processSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdIndex == 0) return;  // ignore empty lines
            cmdBuffer[cmdIndex] = '\0';  // null-terminate
            cmdIndex = 0;
            // Convert to uppercase (in place)
            for (int i = 0; cmdBuffer[i]; i++) {
                cmdBuffer[i] = toupper(cmdBuffer[i]);
            }
            // Match commands
            if      (strcmp(cmdBuffer, "A_ON") == 0)       sendCode(A_On);
            else if (strcmp(cmdBuffer, "A_OFF") == 0)      sendCode(A_Off);
            else if (strcmp(cmdBuffer, "B_ON") == 0)       sendCode(B_On);
            else if (strcmp(cmdBuffer, "B_OFF") == 0)      sendCode(B_Off);
            else if (strcmp(cmdBuffer, "C_ON") == 0)       sendCode(C_On);
            else if (strcmp(cmdBuffer, "C_OFF") == 0)      sendCode(C_Off);
            else if (strcmp(cmdBuffer, "D_ON") == 0)       sendCode(D_On);
            else if (strcmp(cmdBuffer, "D_OFF") == 0)      sendCode(D_Off);
            else if (strcmp(cmdBuffer, "MASTER_ON") == 0)  sendCode(Master_On);
            else if (strcmp(cmdBuffer, "MASTER_OFF") == 0) sendCode(Master_Off);
            else if (strcmp(cmdBuffer, "GATE_OPEN") == 0) sendGate();
            else {
                Serial.print("Unknown command: ");
                Serial.println(cmdBuffer);
            }
        }
        else {
            // Append to buffer
            if (cmdIndex < sizeof(cmdBuffer) - 1) {
                cmdBuffer[cmdIndex++] = c;
            }
            // Reset if buffer overflows
            else {
                cmdIndex = 0;
            }
        }
    }
}

void setup() {
    // Serial interface used for debugging/control
    Serial.begin(115200);
    Serial.println(__FILE__ __DATE__);

    Serial.println("Starting ESP32");
/*
    // Intialise GPIO pins
    pinMode(txPin, OUTPUT);
    // Use INPUT_PULLUP for more stable readings from some 433MHz receivers
    pinMode(rxPin, INPUT_PULLUP);
*/

    //TOF Wire.begin(8, 9); // SDA, SCL
    Wire.begin(21, 42); // SDA, SCL

    if (!lox.begin()) {
        Serial.println("VL53L0X non trovato!");
    }
    Serial.println("VL53L0X pronto");

    // Reset the RF receiver object for a fresh capture
    rfReceiver.treset();
    Serial.println("RF433 Listener Ready.");
    Serial.println("Waiting for signal...");

/*

    // Construct the encoder
    tx_device_gate = rfsend_builder(
        RfSendEncoding::TRIBIT_INVERTED,
        txPin,
        RFSEND_DEFAULT_CONVENTION,  // Do we want to invert 0 and 1 bits? No.
        6,       // Number of sendings
        nullptr, // No callback to keep/stop sending
        15670,    // initseq
        0,       // lo_prefix
        0,       // hi_prefix
        368,       // first_lo_ign
        368,    // lo_short
        725,    // lo_long
        0,       // hi_short
        0,       // hi_long
        724,       // lo_last
        15670,    // sep
        12       // nb_bits
    );

    Serial.println("tx_device_gate created");

    tx_device = rfsend_builder(
        RfSendEncoding::TRIBIT,
        txPin,
        RFSEND_DEFAULT_CONVENTION,  // Do we want to invert 0 and 1 bits? No.
        4,       // Number of sendings
        nullptr, // No callback to keep/stop sending
        2300,    // initseq
        0,       // lo_prefix
        0,       // hi_prefix
        0,       // first_lo_ign
        384,    // lo_short
        1132,    // lo_long
        0,       // hi_short
        0,       // hi_long
        300,       // lo_last
        2300,    // sep
        24       // nb_bits
    );

    Serial.println("tx_device created");
    */
}


void loop() {



    // Periodic debug: print rx pin state so we can see activity
    if (millis() - lastDbg >= DBG_INTERVAL) {
        lastDbg = millis();
        Serial.print("DBG millis="); Serial.print(lastDbg);
        Serial.print(" rx="); Serial.println(digitalRead(rxPin));
    }

    if (rfReceiver.do_events()) {
        Serial.print("RF event detected at "); Serial.println(millis());

        // Fetch decoded blocks (could be several messages from the same button press)
        Decoder* allDecoders = rfReceiver.get_data(RF433ANY_FD_ALL);
        if (!allDecoders) {
            Serial.println("RF event: no decoders returned");
        }
    Decoder* decoder = allDecoders;

    while (decoder != nullptr) {
        printDecodedFrame(decoder);
        
        #ifdef OUTPUT_FIRST_DECODED_ONLY
          break;  // Stop after the first decoded message
        #else
          decoder = decoder->get_next();
        #endif  
    }

    // Clean up the decoders
    delete allDecoders;

    // Reset the RF receiver object for a fresh capture
    rfReceiver.treset();
  }
  




  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    Serial.print("Distanza: ");
    Serial.print(measure.RangeMilliMeter);
    Serial.println(" mm");

    if(measure.RangeMilliMeter<60){
        sendCode(A_On);
    }

  } else {
    Serial.println("Fuori portata");
  }

  delay(500);

    // Handle incoming serial commands
    //processSerial();

}
