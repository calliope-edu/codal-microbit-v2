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

#include "MicroBitConfig.h"

#if CONFIG_ENABLED(DEVICE_BLE)

#include "MicroBitMbitMoreService.h"

using namespace codal;

// Base UUID: 0b50f3e4-607f-4151-9091-7d008d6ffc5c
// Characteristics extend it by replacing bytes 2..3 with `charUUID[idx]`.
const uint8_t MicroBitMbitMoreService::baseUUID[16] = {
    0x0b, 0x50, 0xf3, 0xe4, 0x60, 0x7f, 0x41, 0x51,
    0x90, 0x91, 0x7d, 0x00, 0x8d, 0x6f, 0xfc, 0x5c
};

const uint16_t MicroBitMbitMoreService::serviceUUID = 0xf3e4;

const uint16_t MicroBitMbitMoreService::charUUID[MicroBitMbitMoreService::mmCount] = {
    0x0100, // mmCommand
    0x0101, // mmState
    0x0102, // mmMotion
    0x0110, // mmPinEvent
    0x0111, // mmActionEvent
    0x0120, // mmAnalogInP0
    0x0121, // mmAnalogInP1
    0x0122, // mmAnalogInP2
    0x0123, // mmAnalogInP3
    0x0130  // mmData
};

const uint8_t MicroBitMbitMoreService::charBufSize[MicroBitMbitMoreService::mmCount] = {
    MICROBIT_MBITMORE_BUF_COMMAND,
    MICROBIT_MBITMORE_BUF_STATE,
    MICROBIT_MBITMORE_BUF_MOTION,
    MICROBIT_MBITMORE_BUF_NOTIFY,
    MICROBIT_MBITMORE_BUF_NOTIFY,
    MICROBIT_MBITMORE_BUF_ANALOG_IN,
    MICROBIT_MBITMORE_BUF_ANALOG_IN,
    MICROBIT_MBITMORE_BUF_ANALOG_IN,
    MICROBIT_MBITMORE_BUF_ANALOG_IN,
    MICROBIT_MBITMORE_BUF_NOTIFY,
};

MicroBitMbitMoreService *MicroBitMbitMoreService::instance = NULL;

void MicroBitMbitMoreService::createShared(BLEDevice &_ble)
{
    if (!instance)
        instance = new MicroBitMbitMoreService(_ble);
}

MicroBitMbitMoreService *MicroBitMbitMoreService::sharedInstance()
{
    return instance;
}

MicroBitMbitMoreService::MicroBitMbitMoreService(BLEDevice &_ble)
{
    (void)_ble;

    memclr(buffers, sizeof(buffers));
    for (int i = 0; i < mmCount; i++)
    {
        writeHandlers[i] = NULL;
        readAuthHandlers[i] = NULL;
    }

    // Register custom base UUID and create the service.
    RegisterBaseUUID(baseUUID);
    CreateService(serviceUUID);

    // COMMAND — central writes commands here; cached for read-back.
    CreateCharacteristic(mmCommand, charUUID[mmCommand],
                         buffers[mmCommand],
                         charBufSize[mmCommand], charBufSize[mmCommand],
                         microbit_propWRITE | microbit_propWRITE_WITHOUT | microbit_propREAD);

    // STATE — pin levels / sensor flags, polled by central.
    CreateCharacteristic(mmState, charUUID[mmState],
                         buffers[mmState],
                         charBufSize[mmState], charBufSize[mmState],
                         microbit_propREAD);

    // MOTION — accelerometer / orientation, polled by central.
    CreateCharacteristic(mmMotion, charUUID[mmMotion],
                         buffers[mmMotion],
                         charBufSize[mmMotion], charBufSize[mmMotion],
                         microbit_propREAD);

    // PIN_EVENT — async pin events (driver notifies).
    CreateCharacteristic(mmPinEvent, charUUID[mmPinEvent],
                         buffers[mmPinEvent],
                         charBufSize[mmPinEvent], charBufSize[mmPinEvent],
                         microbit_propREAD | microbit_propNOTIFY);

    // ACTION_EVENT — async button / gesture events (driver notifies).
    CreateCharacteristic(mmActionEvent, charUUID[mmActionEvent],
                         buffers[mmActionEvent],
                         charBufSize[mmActionEvent], charBufSize[mmActionEvent],
                         microbit_propREAD | microbit_propNOTIFY);

    // ANALOG_IN_P0..P3 — read-on-demand via authenticated read.
    for (int p = 0; p < 4; p++)
    {
        const int ch = mmAnalogInP0 + p;
        CreateCharacteristic(ch, charUUID[ch],
                             buffers[ch],
                             charBufSize[ch], charBufSize[ch],
                             microbit_propREAD | microbit_propREADAUTH);
    }

    // DATA — labelled-data channel (driver notifies on send).
    CreateCharacteristic(mmData, charUUID[mmData],
                         buffers[mmData],
                         charBufSize[mmData], charBufSize[mmData],
                         microbit_propREAD | microbit_propNOTIFY);
}

int MicroBitMbitMoreService::setCharValue(CharId ch, const uint8_t *data, size_t len)
{
    if (ch < 0 || ch >= mmCount || !data) return DEVICE_INVALID_PARAMETER;
    const size_t cap = charBufSize[ch];
    const size_t n = len > cap ? cap : len;
    memcpy(buffers[ch], data, n);
    return setChrValue(ch, buffers[ch], (uint16_t)n) ? DEVICE_OK : DEVICE_NOT_SUPPORTED;
}

int MicroBitMbitMoreService::notifyChar(CharId ch, const uint8_t *data, size_t len)
{
    if (ch < 0 || ch >= mmCount || !data) return DEVICE_INVALID_PARAMETER;
    const size_t cap = charBufSize[ch];
    const size_t n = len > cap ? cap : len;
    memcpy(buffers[ch], data, n);
    return notifyChrValue(ch, buffers[ch], (uint16_t)n) ? DEVICE_OK : DEVICE_NOT_SUPPORTED;
}

int MicroBitMbitMoreService::registerWriteHandler(CharId ch, WriteHandler handler)
{
    if (ch < 0 || ch >= mmCount) return DEVICE_INVALID_PARAMETER;
    writeHandlers[ch] = handler;
    return DEVICE_OK;
}

int MicroBitMbitMoreService::registerReadAuthHandler(CharId ch, ReadAuthHandler handler)
{
    if (ch < 0 || ch >= mmCount) return DEVICE_INVALID_PARAMETER;
    readAuthHandlers[ch] = handler;
    return DEVICE_OK;
}

void MicroBitMbitMoreService::onDataWritten(const microbit_ble_evt_write_t *params)
{
    // Map handle → channel index and dispatch.
    for (int i = 0; i < mmCount; i++)
    {
        if (params->handle != valueHandle(i)) continue;
        const size_t cap = charBufSize[i];
        const size_t n = params->len > cap ? cap : params->len;
        // Mirror the write into our buffer so subsequent reads see the
        // value the central just pushed.
        memcpy(buffers[i], params->data, n);
        if (writeHandlers[i])
            writeHandlers[i](params->data, params->len);
        return;
    }
}

void MicroBitMbitMoreService::onDataRead(microbit_onDataRead_t *params)
{
    // For READAUTH characteristics: ask the registered handler for fresh
    // data each time the central reads. Other characteristics fall through
    // to the cached buffer the base class returns by default.
    for (int i = 0; i < mmCount; i++)
    {
        if (params->handle != valueHandle(i) || !readAuthHandlers[i]) continue;
        uint8_t scratch[MICROBIT_MBITMORE_BUF_MAX];
        size_t written = readAuthHandlers[i](scratch, sizeof(scratch));
        if (written > charBufSize[i]) written = charBufSize[i];
        memcpy(buffers[i], scratch, written);
        params->data = buffers[i];
        params->length = (uint16_t)written;
        return;
    }
}

#endif // CONFIG_ENABLED(DEVICE_BLE)
