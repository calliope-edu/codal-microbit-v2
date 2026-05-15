/*
The MIT License (MIT)

Copyright (c) 2016 British Broadcasting Corporation.
This software is provided by Lancaster University by arrangement with the BBC.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#ifndef MICROBIT_MBITMORE_SERVICE_H
#define MICROBIT_MBITMORE_SERVICE_H

#include "MicroBitConfig.h"

#if CONFIG_ENABLED(DEVICE_BLE)

#include "MicroBitBLEManager.h"
#include "MicroBitBLEService.h"

// Per-characteristic buffer sizes from the MbitMore wire protocol.
#define MICROBIT_MBITMORE_BUF_COMMAND   20
#define MICROBIT_MBITMORE_BUF_STATE      7
#define MICROBIT_MBITMORE_BUF_MOTION    18
#define MICROBIT_MBITMORE_BUF_NOTIFY    20
#define MICROBIT_MBITMORE_BUF_ANALOG_IN  2

// Worst-case characteristic size — sized to hold the largest buffer.
#define MICROBIT_MBITMORE_BUF_MAX       20

namespace codal
{

/**
 * Always-on stub of the MbitMore Bluetooth service used by the Scratch /
 * pxt-blocks blocks editor.
 *
 * Registered unconditionally at boot whenever BLE is enabled so that every
 * MakeCode/CODAL build presents the same GATT shape — partial-flash DAL
 * hashes stay aligned across "plain MakeCode", "MakeCode + blocks
 * extension", and "blocks-only" hexes. Without that alignment the
 * partial-flashing service refuses to flash between any two of them.
 *
 * The default behaviour is intentionally inert: reads return whatever was
 * last set (zeros at boot), writes update the buffer and fire any
 * registered handler, and no notifications are emitted until external
 * code drives them. The pxt-blocks extension's `MbitMoreDevice` hooks in
 * via the public API to read sensors / drive actuators / emit events.
 *
 * Wire protocol identifiers — must stay in sync with
 *   - scratch-vm/src/extensions/calliopeMini/index.js (CHARACTERISTIC_UUID enum)
 *   - pxt-blocks/MbitMoreCommon.h (channel constants)
 */
class MicroBitMbitMoreService : public MicroBitBLEService
{
public:
    typedef enum {
        mmCommand,
        mmState,
        mmMotion,
        mmPinEvent,
        mmActionEvent,
        mmAnalogInP0,
        mmAnalogInP1,
        mmAnalogInP2,
        mmAnalogInP3,
        mmData,
        mmCount
    } CharId;

    /**
     * Write handler invoked when a central writes to a characteristic.
     * `data` lives only for the duration of the call — copy if you need it.
     * Return is reserved (currently unused; pass 0).
     */
    typedef int (*WriteHandler)(const uint8_t *data, size_t len);

    /**
     * Read-auth handler invoked when a central performs an authenticated
     * read on an analog-in characteristic. The handler should fill `outBuf`
     * with up to `bufSize` bytes and return the number of bytes written.
     * `outBuf` lives only for the duration of the call.
     */
    typedef size_t (*ReadAuthHandler)(uint8_t *outBuf, size_t bufSize);

    /**
     * Create the singleton instance. Idempotent — repeat calls are no-ops.
     * Called from `MicroBit::init()` after `bleManager.init(...)`.
     */
    static void createShared(BLEDevice &ble);

    /**
     * Get the shared instance, or null if BLE never initialised.
     */
    static MicroBitMbitMoreService *sharedInstance();

    /**
     * Update a characteristic value without notifying. Useful when the
     * external driver maintains a buffer that the central reads on demand
     * (e.g. STATE, MOTION).
     */
    int setCharValue(CharId ch, const uint8_t *data, size_t len);

    /**
     * Update and notify a characteristic value. Used for event-driven
     * channels (PIN_EVENT, ACTION_EVENT, DATA).
     */
    int notifyChar(CharId ch, const uint8_t *data, size_t len);

    /**
     * Register a handler for incoming writes on a characteristic. Replaces
     * any previously-registered handler for that channel. Pass NULL to
     * clear.
     */
    int registerWriteHandler(CharId ch, WriteHandler handler);

    /**
     * Register a read-auth handler for an analog-in characteristic.
     * Replaces any previously-registered handler for that channel.
     */
    int registerReadAuthHandler(CharId ch, ReadAuthHandler handler);

    // ---- MicroBitBLEService overrides ----------------------------------

    virtual void onDataWritten(const microbit_ble_evt_write_t *params);
    virtual void onDataRead(microbit_onDataRead_t *params);

    int characteristicCount() { return mmCount; }
    MicroBitBLEChar *characteristicPtr(int idx) { return &chars[idx]; }

private:
    MicroBitMbitMoreService(BLEDevice &ble);
    static MicroBitMbitMoreService *instance;

    static const uint8_t baseUUID[16];
    static const uint16_t serviceUUID;
    static const uint16_t charUUID[mmCount];
    static const uint8_t charBufSize[mmCount];

    uint8_t buffers[mmCount][MICROBIT_MBITMORE_BUF_MAX];
    WriteHandler writeHandlers[mmCount];
    ReadAuthHandler readAuthHandlers[mmCount];

    MicroBitBLEChar chars[mmCount];
};

} // namespace codal

#endif // CONFIG_ENABLED(DEVICE_BLE)
#endif // MICROBIT_MBITMORE_SERVICE_H
