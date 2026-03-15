/**
 * TI-83 Plus Link Protocol — Seeed XIAO ESP32-C3
 * ================================================
 * Implements the TI "Silent Link" DBus protocol over two open-drain GPIO pins.
 *
 * Wiring (2.5 mm stereo jack → XIAO):
 *   TIP  (white wire) → D4 / GPIO6  (TI_TIP_PIN)
 *   RING (red wire)   → D3 / GPIO5  (TI_RING_PIN)
 *   GND  (sleeve)     → GND
 *
 * Both lines are open-drain: the ESP32 either drives them LOW or lets them
 * float HIGH (the calculator's internal pull-ups hold idle lines high).
 * We use INPUT_PULLUP when "releasing" a line and OUTPUT+LOW when pulling it down.
 *
 * Bit encoding (LSB first, 8 bits per byte):
 *   Send '0': pull TIP low → wait for RING low (ACK) → release TIP → wait RING high
 *   Send '1': pull RING low → wait for TIP low (ACK) → release RING → wait TIP high
 *
 * Packet structure (TI-83+ "Silent Link"):
 *   [machine_id][cmd][len_lo][len_hi][data…][chk_lo][chk_hi]
 *
 * This firmware exposes a Serial (USB-CDC) command interface so you can
 * drive it from a PC or test interactively:
 *   'D' → request directory listing from calculator
 *   'R <name>' → request real variable from calculator (e.g. "R A")
 *   'S <name> <value>' → send real number to calculator (e.g. "S A 3.14")
 *   '?' → print help
 */

#include <Arduino.h>
#include <cstring>
#include <cstdio>
#include <cmath>
#include "esp_task_wdt.h"   // watchdog control during spin-loops
#include "driver/gpio.h"    // gpio_set_direction / gpio_set_pull_mode
#include "rom/ets_sys.h"    // ets_delay_us
#include "esp_timer.h"      // esp_timer_get_time() — hardware µs counter, non-blocking

// ── Pin definitions (XIAO ESP32-C3 Arduino pin numbers) ──────────────────────
#define TI_TIP_PIN   6   // D4 / GPIO6 — 2.5 mm TIP
#define TI_RING_PIN  5   // D3 / GPIO5 — 2.5 mm RING

// ── Protocol constants ────────────────────────────────────────────────────────
#define MACHINE_PC   0x23  // "Computer sending TI-83+ data"
#define MACHINE_TI   0x73  // TI-83+ / TI-84+

// Command IDs
#define CMD_VAR      0x06  // Variable header
#define CMD_CTS      0x09  // Clear to send
#define CMD_DATA     0x15  // Data packet
#define CMD_SKIP     0x36  // Skip / Exit
#define CMD_ACK      0x56  // Acknowledge
#define CMD_ERR      0x5A  // Checksum error (request retransmit)
#define CMD_RDY      0x68  // Check ready
#define CMD_SCR      0x6D  // Request screenshot (silent)
#define CMD_DEL      0x88  // Delete variable (silent)
#define CMD_EOT      0x92  // End of transmission
#define CMD_REQ      0xA2  // Request variable (silent)
#define CMD_RTS      0xC9  // Request to send (silent)

// Type IDs
#define TYPE_REAL    0x00
#define TYPE_DIR     0x19  // Directory request

// Timeouts
#define BIT_TIMEOUT_US   2000000UL  // 2 s per bit (TI spec maximum)
#define BYTE_TIMEOUT_US  3000000UL  // 3 s per byte receive

// Variable header length (13 bytes of data)
#define VAR_HEADER_LEN 13

// ── GPIO register access for tight spin-loops ────────────────────────────────
// On ESP32-C3 (RISC-V), direct register reads are ~3 ns vs ~200 ns for
// digitalRead(). This is critical: the TI-83+ asserts its ACK within ~100 µs
// and releases it quickly. A slow poll loop misses transitions entirely.
#include "hal/gpio_hal.h"
static inline int fastRead(uint8_t pin) {
    return (GPIO.in.val >> pin) & 1;
}

// ── Open-drain helpers ────────────────────────────────────────────────────────
// Drive low  — set pin as OUTPUT and pull it to GND
static inline void driveLoTip() {
    gpio_set_direction((gpio_num_t)TI_TIP_PIN,  GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)TI_TIP_PIN,  0);
}
static inline void driveLoRing() {
    gpio_set_direction((gpio_num_t)TI_RING_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)TI_RING_PIN, 0);
}
// Release — switch to input with the ESP32 pull-up enabled.
// The calculator's own pull-up (~3.3 V) also holds the line high.
static inline void releaseTip() {
    gpio_set_direction((gpio_num_t)TI_TIP_PIN,  GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)TI_TIP_PIN,  GPIO_PULLUP_ONLY);
}
static inline void releaseRing() {
    gpio_set_direction((gpio_num_t)TI_RING_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)TI_RING_PIN, GPIO_PULLUP_ONLY);
}
// Convenience wrappers used by the probe/toggle commands
static inline int readTip()  { return fastRead(TI_TIP_PIN);  }
static inline int readRing() { return fastRead(TI_RING_PIN); }

// ── Low-level bit I/O ─────────────────────────────────────────────────────────

// ── Timing config ─────────────────────────────────────────────────────────────
// Spin timeout: ~2 seconds at 160 MHz (no-sleep loop, ~3-5 ns per iteration)
#define SPIN_TIMEOUT 400000000UL

/**
 * TI-83 Plus bit encoding (confirmed from observed hardware behaviour):
 *   Sender pulls TIP  low → bit 0  (receiver ACKs by pulling RING low)
 *   Sender pulls RING low → bit 1  (receiver ACKs by pulling TIP  low)
 * Both sides use the same convention regardless of direction.
 */

/**
 * Send one bit — no Serial calls in the hot path.
 */
static bool sendBit(int bit) {
    uint32_t n = 0;
    // Wait for bus idle
    while (fastRead(TI_TIP_PIN) == 0 || fastRead(TI_RING_PIN) == 0) {
        if (++n > SPIN_TIMEOUT) { Serial.println("[ERR] sendBit: bus stuck low"); return false; }
    }
    ets_delay_us(10);
    n = 0;

    if (bit == 0) {
        // Send 0: pull TIP low, wait for RING low (ACK), release TIP, wait RING high
        driveLoTip();
        while (fastRead(TI_RING_PIN) == 1) {
            if (++n > SPIN_TIMEOUT) { releaseTip(); Serial.println("[ERR] send0: no ACK"); return false; }
        }
        releaseTip();
        while (fastRead(TI_RING_PIN) == 0) {}
    } else {
        // Send 1: pull RING low, wait for TIP low (ACK), release RING, wait TIP high
        driveLoRing();
        while (fastRead(TI_TIP_PIN) == 1) {
            if (++n > SPIN_TIMEOUT) { releaseRing(); Serial.println("[ERR] send1: no ACK"); return false; }
        }
        releaseRing();
        while (fastRead(TI_TIP_PIN) == 0) {}
    }
    ets_delay_us(6);
    return true;
}

// Timing log for last recvBit — populated during the hot path using
// hardware timer reads (no Serial, no blocking), printed after the bit.
static int64_t rbt0, rbt1, rbt2, rbt3, rbt4;  // us timestamps

/**
 * Receive one bit — no Serial calls in the hot path.
 * Timestamps recorded via esp_timer_get_time() (hardware, ~1us resolution, non-blocking).
 */
static int recvBit() {
    uint32_t n = 0;
    rbt0 = esp_timer_get_time();   // T0: enter recvBit

    while (true) {
        int tip  = fastRead(TI_TIP_PIN);
        int ring = fastRead(TI_RING_PIN);

        if (tip == 0 && ring == 1) {
            rbt1 = esp_timer_get_time();   // T1: detected TIP low (bit start)
            // Calc pulled TIP low → bit 0, ACK on RING
            driveLoRing();
            rbt2 = esp_timer_get_time();   // T2: we drove RING low
            n = 0;
            while (fastRead(TI_TIP_PIN)  == 0) { if (++n > SPIN_TIMEOUT) { releaseRing(); return -1; } }
            rbt3 = esp_timer_get_time();   // T3: calc released TIP
            releaseRing();
            n = 0;
            while (fastRead(TI_RING_PIN) == 0) { if (++n > SPIN_TIMEOUT) return -1; }
            rbt4 = esp_timer_get_time();   // T4: RING back high (bit complete)
            return 0;
        }
        if (ring == 0 && tip == 1) {
            rbt1 = esp_timer_get_time();
            // Calc pulled RING low → bit 1, ACK on TIP
            driveLoTip();
            rbt2 = esp_timer_get_time();
            n = 0;
            while (fastRead(TI_RING_PIN) == 0) { if (++n > SPIN_TIMEOUT) { releaseTip(); return -1; } }
            rbt3 = esp_timer_get_time();
            releaseTip();
            n = 0;
            while (fastRead(TI_TIP_PIN)  == 0) { if (++n > SPIN_TIMEOUT) return -1; }
            rbt4 = esp_timer_get_time();
            return 1;
        }
        if (++n > SPIN_TIMEOUT) return -1;
    }
}

// ── Byte-level I/O ────────────────────────────────────────────────────────────

/** Send one byte LSB-first. Returns false on error. */
static bool sendByte(uint8_t b) {
    // Disable task watchdog for the duration — spin-loops would otherwise
    // trigger a WDT reset on ESP32-C3 after ~5 s of continuous polling.
    esp_task_wdt_reset();
    for (int i = 0; i < 8; i++) {
        if (!sendBit((b >> i) & 1)) {
            Serial.printf("[ERR] sendByte timeout on bit %d of 0x%02X\n", i, b);
            return false;
        }
    }
    return true;
}

/** Receive one byte LSB-first. Returns false on error. */
static bool recvByte(uint8_t &out) {
    esp_task_wdt_reset();
    out = 0;
    for (int i = 0; i < 8; i++) {
        int b = recvBit();
        if (b < 0) {
            Serial.printf("[ERR] recvByte timeout on bit %d (got %d bits ok, partial=0x%02X)\n",
                          i, i, out);
            return false;
        }
        out |= (uint8_t)(b << i);
        // Print timing after each bit - Serial.printf here is fine since
        // the calculator won't start the next bit until we ACK, which we already did.
        Serial.printf("  [BYTE] bit%d=%d partial=0x%02X | "
                      "wait=%lldus ack=%lldus hold=%lldus settle=%lldus | "
                      "tip=%d ring=%d\n",
                      i, b, out,
                      (long long)(rbt1 - rbt0),   // us waiting for bit start
                      (long long)(rbt2 - rbt1),   // us to drive our ACK
                      (long long)(rbt3 - rbt2),   // us calc held its line low
                      (long long)(rbt4 - rbt3),   // us for ACK line to go high
                      fastRead(TI_TIP_PIN), fastRead(TI_RING_PIN));
    }
    return true;
}

// ── Checksum ──────────────────────────────────────────────────────────────────
static uint16_t calcChecksum(const uint8_t *data, uint16_t len) {
    uint16_t sum = 0;
    for (uint16_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

// ── Packet I/O ────────────────────────────────────────────────────────────────

/**
 * Send a TI-83+ packet.
 *   machineId : MACHINE_PC
 *   cmd       : command byte
 *   data      : payload (may be nullptr if dataLen == 0)
 *   dataLen   : number of payload bytes
 */
// ── Bus reset / wakeup ────────────────────────────────────────────────────────
// Pull both lines low briefly then release — this resets the calculator's
// link port state machine and is required before initiating a new transaction
// after any error or timeout.
static void busReset() {
    driveLoTip();
    driveLoRing();
    ets_delay_us(300);   // hold both low for 300 µs
    releaseTip();
    releaseRing();
    ets_delay_us(500);   // let lines settle back high
    Serial.println("  [BUS] reset sent (both lines pulsed low 300us)");
}

static bool sendPacket(uint8_t machineId, uint8_t cmd,
                       const uint8_t *data, uint16_t dataLen) {
    busReset();   // wake calculator link port before every new packet
    // Header
    if (!sendByte(machineId)) return false;
    if (!sendByte(cmd))       return false;
    if (!sendByte(dataLen & 0xFF))        return false;
    if (!sendByte((dataLen >> 8) & 0xFF)) return false;

    if (dataLen == 0 || data == nullptr) return true;

    // Data
    for (uint16_t i = 0; i < dataLen; i++) {
        if (!sendByte(data[i])) return false;
    }

    // Checksum
    uint16_t chk = calcChecksum(data, dataLen);
    if (!sendByte(chk & 0xFF))        return false;
    if (!sendByte((chk >> 8) & 0xFF)) return false;

    return true;
}

/**
 * Receive a TI-83+ packet.
 * dataBuf must be large enough for the incoming payload + 2-byte checksum.
 * Returns false on comm error or checksum mismatch.
 */
static bool recvPacket(uint8_t &machineId, uint8_t &cmd,
                       uint8_t *dataBuf, uint16_t &dataLen,
                       uint16_t bufSize) {
    if (!recvByte(machineId)) return false;
    Serial.printf("  [PKT] machineId=0x%02X\n", machineId);
    if (!recvByte(cmd)) return false;
    Serial.printf("  [PKT] cmd=0x%02X\n", cmd);

    uint8_t lo, hi;
    if (!recvByte(lo)) return false;
    if (!recvByte(hi)) return false;
    dataLen = (uint16_t)lo | ((uint16_t)hi << 8);

    if (dataLen == 0) return true;  // no data, no checksum needed

    if (dataLen > bufSize) {
        Serial.printf("[ERR] recvPacket: data too large (%u > %u)\n", dataLen, bufSize);
        return false;
    }

    for (uint16_t i = 0; i < dataLen; i++) {
        if (!recvByte(dataBuf[i])) return false;
    }

    uint8_t ckLo, ckHi;
    if (!recvByte(ckLo)) return false;
    if (!recvByte(ckHi)) return false;

    uint16_t recvChk = (uint16_t)ckLo | ((uint16_t)ckHi << 8);
    uint16_t calcChk = calcChecksum(dataBuf, dataLen);
    if (recvChk != calcChk) {
        Serial.printf("[ERR] Checksum mismatch: recv=0x%04X calc=0x%04X\n", recvChk, calcChk);
        return false;
    }
    return true;
}

// ── ACK helpers ───────────────────────────────────────────────────────────────

static bool sendACK() {
    return sendPacket(MACHINE_PC, CMD_ACK, nullptr, 0);
}

static bool recvACK() {
    uint8_t mid, cmd;
    uint8_t buf[32];   // ACK carries no data but give room for unexpected payloads
    uint16_t dlen;
    if (!recvPacket(mid, cmd, buf, dlen, sizeof(buf))) return false;
    if (cmd != CMD_ACK) {
        Serial.printf("[WARN] Expected ACK, got machineId=0x%02X cmd=0x%02X dlen=%u\n", mid, cmd, dlen);
        return false;
    }
    return true;
}

// ── Build a variable header ────────────────────────────────────────────────────

/**
 * Build the 13-byte variable header used in REQ / RTS / VAR packets.
 *   varSize  : size of the variable data in bytes
 *   typeId   : e.g. TYPE_REAL
 *   name     : NUL-terminated variable name (max 8 bytes, padded with NUL)
 *   archived : set bit 7 of the last byte if archived
 */
static void buildVarHeader(uint8_t *hdr, uint16_t varSize,
                           uint8_t typeId, const char *name, bool archived = false) {
    memset(hdr, 0, VAR_HEADER_LEN);
    hdr[0] = varSize & 0xFF;
    hdr[1] = (varSize >> 8) & 0xFF;
    hdr[2] = typeId;
    strncpy((char *)&hdr[3], name, 8);   // bytes 3-10, NUL-padded
    hdr[11] = 0x00;                       // version
    hdr[12] = archived ? 0x80 : 0x00;    // type byte 2 / archive flag
}

// ── TI real-number encoding ────────────────────────────────────────────────────
/**
 * The TI-83+ stores real numbers in a 9-byte format:
 *   byte 0   : flags (0x00 = positive, 0x80 = negative)
 *   byte 1   : biased exponent  (0x80 = 10^0, 0x7F = 10^-1, 0x81 = 10^1 …)
 *   bytes 2-8: 14 BCD digits, two per byte, most significant first
 */
static void encodeTIReal(uint8_t *out, double val) {
    memset(out, 0, 9);

    if (val < 0) { out[0] = 0x80; val = -val; }

    if (val == 0.0) { out[1] = 0x80; return; }  // zero special case

    // Find exponent (base 10)
    int exp10 = (int)floor(log10(val));
    // Normalise to 14 significant digits: val = mantissa * 10^exp10
    // We want mantissa in [1, 10)
    double mantissa = val / pow(10.0, exp10);
    // Guard rounding
    if (mantissa >= 10.0) { mantissa /= 10.0; exp10++; }
    if (mantissa < 1.0)   { mantissa *= 10.0; exp10--; }

    out[1] = (uint8_t)(exp10 + 0x80);  // biased exponent

    // Encode 14 BCD digits
    for (int i = 0; i < 7; i++) {
        int hi = (int)mantissa;
        mantissa = (mantissa - hi) * 10.0;
        int lo = (int)mantissa;
        mantissa = (mantissa - lo) * 10.0;
        out[2 + i] = (uint8_t)((hi << 4) | lo);
    }
}

/**
 * Decode a TI real (9 bytes) back to double.
 */
static double decodeTIReal(const uint8_t *in) {
    bool negative = (in[0] & 0x80) != 0;
    int  exp10    = (int)in[1] - 0x80;

    double mantissa = 0.0;
    double factor   = 1.0;
    for (int i = 0; i < 7; i++) {
        int hi = (in[2 + i] >> 4) & 0x0F;
        int lo =  in[2 + i]       & 0x0F;
        mantissa += hi * factor;   factor /= 10.0;
        mantissa += lo * factor;   factor /= 10.0;
    }

    double val = mantissa * pow(10.0, exp10);
    return negative ? -val : val;
}

// ── High-level protocol operations ───────────────────────────────────────────

/**
 * Passive receive: wait for the calculator to initiate a transfer.
 * Put the calculator in Link->Send mode BEFORE calling this.
 * The calculator sends: VAR header, we ACK, we send CTS, calc sends DATA, we ACK.
 * Prints the variable name, type, and for reals, the decoded value.
 */
static bool receiveFromCalc() {
    Serial.println("[RECV] Waiting for calculator to initiate (30 s timeout)...");
    Serial.println("[RECV] On calculator: 2nd -> LINK -> SEND -> select var -> TRANSMIT");

    uint8_t mid, cmd;
    uint8_t buf[256];
    uint16_t dlen;

    // Step 1: receive VAR header from calculator
    if (!recvPacket(mid, cmd, buf, dlen, sizeof(buf))) {
        Serial.println("[ERR] No packet received from calculator");
        return false;
    }
    Serial.printf("[RECV] Got packet: machineId=0x%02X cmd=0x%02X dlen=%u\n", mid, cmd, dlen);

    if (cmd == CMD_VAR) {
        uint16_t varSize = buf[0] | ((uint16_t)buf[1] << 8);
        uint8_t  typeId  = buf[2];
        char     name[9]; memset(name, 0, 9);
        memcpy(name, &buf[3], 8);
        // trim trailing nulls for display
        for (int i = 7; i >= 0; i--) { if (name[i] == 0) name[i] = '\0'; else break; }
        bool archived = dlen >= 13 && (buf[12] & 0x80);
        Serial.printf("[RECV] VAR: name='%s' type=0x%02X size=%u%s\n",
                      name, typeId, varSize, archived ? " (archived)" : "");

        // Step 2: ACK the VAR header
        if (!sendACK()) { Serial.println("[ERR] Failed to ACK VAR"); return false; }

        // Step 3: send CTS
        if (!sendPacket(MACHINE_PC, CMD_CTS, nullptr, 0)) {
            Serial.println("[ERR] Failed to send CTS"); return false;
        }

        // Step 4: receive ACK for our CTS
        if (!recvACK()) { Serial.println("[ERR] No ACK for CTS"); return false; }

        // Step 5: receive DATA
        if (!recvPacket(mid, cmd, buf, dlen, sizeof(buf))) {
            Serial.println("[ERR] No DATA packet"); return false;
        }
        if (cmd != CMD_DATA) {
            Serial.printf("[ERR] Expected DATA, got 0x%02X\n", cmd); return false;
        }

        // Step 6: ACK the data
        if (!sendACK()) { Serial.println("[ERR] Failed to ACK DATA"); return false; }

        // Decode if it's a real number
        if (typeId == TYPE_REAL && dlen >= 11) {
            double val = decodeTIReal(&buf[2]);
            Serial.printf("[RECV] Real value: %s = %.14g\n", name, val);
        } else {
            Serial.printf("[RECV] Received %u bytes of type 0x%02X\n", dlen, typeId);
        }

        // Step 7: wait for EOT
        if (!recvPacket(mid, cmd, buf, dlen, sizeof(buf))) {
            Serial.println("[WARN] No EOT received"); return true;
        }
        if (cmd == CMD_EOT) {
            sendACK();
            Serial.println("[RECV] Transfer complete.");
        }
        return true;

    } else if (cmd == CMD_EOT) {
        sendACK();
        Serial.println("[RECV] Calculator sent EOT with no variables.");
        return true;
    } else {
        Serial.printf("[ERR] Unexpected first packet cmd=0x%02X\n", cmd);
        sendACK();  // try to ack anyway to keep calc happy
        return false;
    }
}


/**
 * Request a directory listing from the calculator.
 * Prints variable names and types to Serial.
 */
static bool requestDirectory() {
    Serial.println("[DIR] Requesting directory…");

    // Build REQ packet with type=0x19 (directory)
    uint8_t hdr[VAR_HEADER_LEN];
    buildVarHeader(hdr, 0x0000, TYPE_DIR, "");

    if (!sendPacket(MACHINE_PC, CMD_REQ, hdr, VAR_HEADER_LEN)) {
        Serial.println("[ERR] sendPacket REQ failed"); return false;
    }
    if (!recvACK()) { Serial.println("[ERR] No ACK for REQ"); return false; }

    // Receive DATA: 2-byte free memory count
    uint8_t mid, cmd;
    uint8_t buf[256];
    uint16_t dlen;
    if (!recvPacket(mid, cmd, buf, dlen, sizeof(buf))) return false;
    if (cmd != CMD_DATA) { Serial.printf("[ERR] Expected DATA, got 0x%02X\n", cmd); return false; }
    uint16_t freeMem = buf[0] | ((uint16_t)buf[1] << 8);
    Serial.printf("[DIR] Free memory: %u bytes\n", freeMem);
    if (!sendACK()) return false;

    // Receive variable headers until EOT
    int count = 0;
    while (true) {
        if (!recvPacket(mid, cmd, buf, dlen, sizeof(buf))) return false;
        if (cmd == CMD_EOT) { sendACK(); break; }
        if (cmd != CMD_VAR) { Serial.printf("[ERR] Expected VAR, got 0x%02X\n", cmd); return false; }

        // Parse the variable header
        uint16_t varSize = buf[0] | ((uint16_t)buf[1] << 8);
        uint8_t  typeId  = buf[2];
        char     name[9]; memset(name, 0, 9);
        memcpy(name, &buf[3], 8);
        bool archived = (buf[12] & 0x80) != 0;

        Serial.printf("[DIR]  [%2d] name='%s'  type=0x%02X  size=%u%s\n",
                      count++, name, typeId, varSize, archived ? "  (ARCH)" : "");
        if (!sendACK()) return false;
    }

    Serial.printf("[DIR] Done. %d variables listed.\n", count);
    return true;
}

/**
 * Request (download) a real variable by single-letter name.
 * Prints the decoded value to Serial.
 */
static bool requestReal(const char *varName) {
    Serial.printf("[GET] Requesting real '%s'…\n", varName);

    uint8_t hdr[VAR_HEADER_LEN];
    buildVarHeader(hdr, 9 /*real=9 bytes*/, TYPE_REAL, varName);

    if (!sendPacket(MACHINE_PC, CMD_REQ, hdr, VAR_HEADER_LEN)) return false;
    if (!recvACK()) { Serial.println("[ERR] No ACK for REQ"); return false; }

    // Calculator may send EXIT if variable not found
    uint8_t mid, cmd;
    uint8_t buf[256];
    uint16_t dlen;
    if (!recvPacket(mid, cmd, buf, dlen, sizeof(buf))) return false;
    if (cmd == CMD_SKIP) {
        sendACK();
        Serial.printf("[GET] Variable '%s' not found on calculator.\n", varName);
        return false;
    }
    if (cmd != CMD_VAR) { Serial.printf("[ERR] Expected VAR, got 0x%02X\n", cmd); return false; }
    if (!sendACK()) return false;

    // Send CTS
    if (!sendPacket(MACHINE_PC, CMD_CTS, nullptr, 0)) return false;
    if (!recvACK()) { Serial.println("[ERR] No ACK for CTS"); return false; }

    // Receive DATA
    if (!recvPacket(mid, cmd, buf, dlen, sizeof(buf))) return false;
    if (cmd != CMD_DATA) { Serial.printf("[ERR] Expected DATA, got 0x%02X\n", cmd); return false; }
    if (!sendACK()) return false;

    // The first two bytes of TI variable data are a size word, followed by 9 bytes of real
    // Structure: [size_lo][size_hi][9 real bytes]
    if (dlen < 11) { Serial.println("[ERR] DATA too short for real"); return false; }
    double val = decodeTIReal(&buf[2]);
    Serial.printf("[GET] %s = %.14g\n", varName, val);
    return true;
}

/**
 * Send (upload) a real number to the calculator under varName.
 */
static bool sendReal(const char *varName, double value) {
    Serial.printf("[PUT] Sending real '%s' = %.14g\n", varName, value);

    // Encode the real (9 bytes)
    uint8_t realBytes[9];
    encodeTIReal(realBytes, value);

    // Variable data = 2-byte little-endian size + 9 real bytes = 11 bytes total
    uint8_t varData[11];
    varData[0] = 9; varData[1] = 0;  // data size = 9
    memcpy(&varData[2], realBytes, 9);

    // Build RTS header (data size = 9 for the real itself)
    uint8_t hdr[VAR_HEADER_LEN];
    buildVarHeader(hdr, 9, TYPE_REAL, varName);

    // Step 1: Send RTS
    if (!sendPacket(MACHINE_PC, CMD_RTS, hdr, VAR_HEADER_LEN)) return false;
    if (!recvACK()) { Serial.println("[ERR] No ACK for RTS"); return false; }

    // Step 2: Receive CTS or SKIP/EXIT
    uint8_t mid, cmd;
    uint8_t buf[32];
    uint16_t dlen;
    if (!recvPacket(mid, cmd, buf, dlen, sizeof(buf))) return false;
    if (cmd == CMD_SKIP) {
        sendACK();
        Serial.println("[PUT] Calculator rejected: out of memory or skipped.");
        return false;
    }
    if (cmd != CMD_CTS) { Serial.printf("[ERR] Expected CTS, got 0x%02X\n", cmd); return false; }
    if (!sendACK()) return false;

    // Step 3: Send DATA
    if (!sendPacket(MACHINE_PC, CMD_DATA, varData, sizeof(varData))) return false;
    if (!recvACK()) { Serial.println("[ERR] No ACK for DATA"); return false; }

    // Step 4: Send EOT
    if (!sendPacket(MACHINE_PC, CMD_EOT, nullptr, 0)) return false;

    Serial.println("[PUT] Done.");
    return true;
}

// ── Serial command interface ──────────────────────────────────────────────────

static void printHelp() {
    Serial.println();
    Serial.println("TI-83+ Link Interface - Commands:");
    Serial.println("  D           - Request directory listing");
    Serial.println("  R <n>       - Read real variable  (e.g. 'R A')");
    Serial.println("  S <n> <v>   - Send real variable  (e.g. 'S A 3.14')");
    Serial.println("  P           - Probe: sample TIP/RING for 2 s (wiring check)");
    Serial.println("  T           - Toggle TIP low 500 ms then release (GPIO test)");
    Serial.println("  L           - Loopback: raw line drive/read + calc ACK watch");
    Serial.println("  V           - Receive: wait for calc to send (Link->Send on calc)");
    Serial.println("  W           - Watch: passive listen for 10 s (use Link->Send on calc)");
    Serial.println("  ?           - This help");
    Serial.println();
}

static char cmdBuf[64];
static uint8_t cmdLen = 0;

static void processCommand(const char *line) {
    char cmd = line[0];
    if (cmd == 'D' || cmd == 'd') {
        requestDirectory();
    } else if (cmd == 'R' || cmd == 'r') {
        char name[9] = {0};
        sscanf(line + 1, " %8s", name);
        if (name[0]) requestReal(name);
        else Serial.println("[ERR] Usage: R <n>");
    } else if (cmd == 'S' || cmd == 's') {
        char name[9] = {0}; double val = 0;
        if (sscanf(line + 1, " %8s %lf", name, &val) == 2) sendReal(name, val);
        else Serial.println("[ERR] Usage: S <n> <value>");

    } else if (cmd == 'P' || cmd == 'p') {
        // Probe: check idle line states
        releaseTip(); releaseRing();
        Serial.println("[PROBE] Sampling TIP (GPIO6) and RING (GPIO5) for 2 s.");
        Serial.println("        At idle both lines should read 1 (HIGH).");
        Serial.println("        A stuck 0 means a short or wrong GND connection.");
        for (int i = 0; i < 20; i++) {
            Serial.printf("  [%2d] TIP=%d  RING=%d\n", i, readTip(), readRing());
            delay(100);
        }
        Serial.println("[PROBE] Done.");

    } else if (cmd == 'T' || cmd == 't') {
        // Toggle: drive TIP low, verify GPIO is actually driving
        releaseTip(); releaseRing();
        Serial.printf("[TOGGLE] TIP idle=%d  RING idle=%d\n", readTip(), readRing());
        Serial.println("[TOGGLE] Driving TIP LOW for 500 ms...");
        driveLoTip();
        Serial.printf("[TOGGLE] TIP driven=%d  RING during=%d\n", readTip(), readRing());
        delay(500);
        releaseTip();
        Serial.printf("[TOGGLE] TIP released=%d  RING after=%d\n", readTip(), readRing());
        Serial.println("[TOGGLE] Expected: driven=0, released=1.");
        Serial.println("         TIP never 0 -> GPIO6 not driving. Check solder joint.");
        Serial.println("         TIP stuck 0 -> line is shorted to GND.");

    } else if (cmd == 'V' || cmd == 'v') {
        // Passive receive: ESP32 listens, calculator initiates
        receiveFromCalc();

    } else if (cmd == 'W' || cmd == 'w') {
        // ── Passive listen: watch lines for 10 s without transmitting anything ──
        // On the calculator, go to Link -> Send -> Real -> A -> Transmit
        // while this is running. We should see TIP or RING go low as the
        // calculator starts sending. If we see nothing, the port is dead or
        // our GND reference is wrong.
        releaseTip(); releaseRing();
        Serial.println("[WATCH] Listening on TIP+RING for 10 s — do Link->Send on calc now");
        Serial.println("[WATCH] Will print any state change detected...");
        int lastTip = 1, lastRing = 1;
        uint32_t changes = 0;
        unsigned long start = millis();
        while (millis() - start < 10000) {
            int t = fastRead(TI_TIP_PIN);
            int r = fastRead(TI_RING_PIN);
            if (t != lastTip || r != lastRing) {
                Serial.printf("[WATCH] %4lums  TIP=%d RING=%d\n",
                              (unsigned long)(millis() - start), t, r);
                lastTip = t; lastRing = r;
                changes++;
            }
        }
        if (changes == 0)
            Serial.println("[WATCH] No changes detected. Calculator produced no signal.");
        else
            Serial.printf("[WATCH] Done. %lu transitions detected.\n", (unsigned long)changes);

    } else if (cmd == 'L' || cmd == 'l') {
        // ── Raw loopback / signal test ──────────────────────────────────────
        // Drives each line low in turn and checks if the OTHER line sees it.
        // On a working TI-83+ the calculator pulls both lines to ~3.3V.
        // If we drive TIP low, the calculator should NOT pull RING low on its
        // own — but if TIP and RING are bridged by a short, RING will also drop.
        // This test also verifies each GPIO can independently drive and read.
        Serial.println("[LOOP] --- Raw line test ---");
        releaseTip(); releaseRing();
        ets_delay_us(500);
        Serial.printf("[LOOP] Idle:            TIP=%d RING=%d  (expect 1 1)\n",
                      fastRead(TI_TIP_PIN), fastRead(TI_RING_PIN));

        // Drive TIP low, read both
        driveLoTip();
        ets_delay_us(200);
        int t1 = fastRead(TI_TIP_PIN), r1 = fastRead(TI_RING_PIN);
        releaseTip();
        ets_delay_us(200);
        int t2 = fastRead(TI_TIP_PIN), r2 = fastRead(TI_RING_PIN);
        Serial.printf("[LOOP] TIP driven low:  TIP=%d RING=%d  (expect TIP=0)\n", t1, r1);
        Serial.printf("[LOOP] TIP released:    TIP=%d RING=%d  (expect 1 1)\n",   t2, r2);
        if (t1 != 0) Serial.println("[LOOP] !! TIP didn't go low — GPIO6 not driving");
        if (r1 == 0) Serial.println("[LOOP] !! RING also went low — possible short between lines");
        if (t2 != 1) Serial.println("[LOOP] !! TIP stuck low after release — short to GND");

        // Drive RING low, read both
        driveLoRing();
        ets_delay_us(200);
        int t3 = fastRead(TI_TIP_PIN), r3 = fastRead(TI_RING_PIN);
        releaseRing();
        ets_delay_us(200);
        int t4 = fastRead(TI_TIP_PIN), r4 = fastRead(TI_RING_PIN);
        Serial.printf("[LOOP] RING driven low: TIP=%d RING=%d  (expect RING=0)\n", t3, r3);
        Serial.printf("[LOOP] RING released:   TIP=%d RING=%d  (expect 1 1)\n",    t4, r4);
        if (r3 != 0) Serial.println("[LOOP] !! RING didn't go low — GPIO5 not driving");
        if (t3 == 0) Serial.println("[LOOP] !! TIP also went low — possible short between lines");
        if (r4 != 1) Serial.println("[LOOP] !! RING stuck low after release — short to GND");

        // Now drive TIP low and wait up to 2s to see if calculator ACKs on RING
        Serial.println("[LOOP] Driving TIP low — watching for calc to ACK on RING (2s)...");
        driveLoTip();
        ets_delay_us(200);
        bool calcAcked = false;
        uint32_t waited = 0;
        while (waited < 400000000UL) {
            if (fastRead(TI_RING_PIN) == 0) { calcAcked = true; break; }
            waited++;
        }
        int tipNow = fastRead(TI_TIP_PIN), ringNow = fastRead(TI_RING_PIN);
        releaseTip();
        if (calcAcked) {
            Serial.printf("[LOOP] Calculator ACKed on RING after ~%lu us!\n", waited / 160);
            Serial.println("[LOOP] Link port IS responding. Protocol issue, not hardware.");
        } else {
            Serial.printf("[LOOP] No ACK from calculator. TIP=%d RING=%d at timeout.\n",
                          tipNow, ringNow);
            Serial.println("[LOOP] Calculator link port is not responding to TIP going low.");
            Serial.println("[LOOP] Possible causes:");
            Serial.println("[LOOP]   1. TIP wire not making contact with calculator port");
            Serial.println("[LOOP]   2. Calculator link port hardware damaged");
            Serial.println("[LOOP]   3. TIP and RING labels are correct but jack wiring reversed");
        }
        Serial.println("[LOOP] --- Done ---");

    } else if (cmd == '?') {
        printHelp();
    } else {
        Serial.printf("[ERR] Unknown command '%c'. Type '?' for help.\n", cmd);
    }
}

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(1500);   // wait for USB-CDC to enumerate

    // Both lines start released (HIGH)
    releaseTip();
    releaseRing();

    Serial.println("\n=== TI-83+ Link Interface Ready ===");
    printHelp();
}

void loop() {
    // Collect a line from USB-Serial
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            cmdBuf[cmdLen] = '\0';
            if (cmdLen > 0) processCommand(cmdBuf);
            cmdLen = 0;
        } else if (cmdLen < (sizeof(cmdBuf) - 1)) {
            cmdBuf[cmdLen++] = c;
        }
    }
}