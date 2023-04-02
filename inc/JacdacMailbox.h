/*
The MIT License (MIT)

Copyright (c) 2017 Lancaster University.

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

#ifndef JACDAC_MAILBOX_H
#define JACDAC_MAILBOX_H

#include "CodalConfig.h"
#include "CodalComponent.h"
#include "MicroBitCompat.h"
#include "MicroBitMemoryMap.h"
#include "jdprotocol.h"

#define JACDAC_MAILBOX_BUFFER_SIZE              256
#define JACDAC_MAILBOX_MAXIMUM_QUEUE_SIZE       10

/**
  * Class definition for a MicroBitMailbox.
  *
  * Provides a memory based mechanism through which data and commands can be transferred to 
  * to a host application on a USB attached machine via DapLink debug channels (such as DapJS)
  */

namespace codal
{
    class JacdacMailbox;

    struct JacdacMailboxBuffer {
        uint8_t magic[8];                                              // Validation identifier
        uint8_t irqn;                                                  // IRQ number raised by PC to signal new command/data has been written
        uint8_t padding[3];                                            // unused
        volatile uint8_t inputBuffer[JACDAC_MAILBOX_BUFFER_SIZE];      // the PC reads from here
        volatile uint8_t outputBuffer[JACDAC_MAILBOX_BUFFER_SIZE];     // the PC writes here
    };

    struct LinkedFrame {
        LinkedFrame *next;
        uint32_t timestamp_ms;
        jd_frame_t frame;
    };

    class JacdacMailboxHandler
    {
        JacdacMailbox *mailbox;

        /**
         * Constructor.
         * Create a JacdacMailboxHandler instance, and register it with the given mailbox.
         *
         * @param the Jacdac mailbox to use. If NULL, the singleton mailbox is chosen.
         */
        JacdacMailboxHandler(JacdacMailbox *m = NULL);

        /**
         * Process a frame that has been received by the mailbox 
         * This method will be invokedfor on each packet received. 
         * NOTE: This may (likely) be called in interrupt context, so decoupling via scheduler is recommended for complex operations
         *
         * @param frame The data that has been received
         */
        virtual void process(const jd_frame_t *frame);
    };

    class JacdacMailbox
    {
    public:
        static JacdacMailbox    *instance;             // Reference to the singleton of this class
        JacdacMailboxBuffer*    exchangeBuffer;        // pointer to heap allocated memory in which the mailbox memory is stored.
        LinkedFrame *volatile   outputQueue;           // queue of frames waiting to be sent to the PC by the device.

        /**
         * Constructor.
         * Create a JacdacMailbox instance.
         *
         * @param memoryRegion Pointer to a RAM data structure where the address of the mailbox in memory can be registered.
         */
        JacdacMailbox(MicroBitNoInitMemoryRegion *memoryRegion = NULL);

        /**
         * Add a protocol service handler. 
         * All data handlers are invoked on each packet received. 
         * NOTE: This may (likely) be called in nterrupt context, so decoupling via scheduler is recommended for complex operations
         *
         * @param handler A class implementing the JacdacMailboxHandler to add
         * @return DEVICE_OK on success
         */
        int addHandler(JacdacMailboxHandler *handler);

        /**
         * Send a given buffer to an attached PC via the mailbox. 
         * If the mailbox is empty, it is scheduled for immediate transmission. if not, it is queued awaiting collection.
         * @param frame the jacdac frame to send
         * @return DEVICE_OK, or DEVICE_INSUFFICIENT_RESOURCES if the outputQueue is full.
         */
        int sendFrame(const jd_frame_t *frame);

        /**
         * Attempts to flush the input and output buffers.
         * A succesfully received buffer is delivered to all registered handers.
         * Schedule the next packet to be sent from the outputQueue is applicable.
         */
        void processQueues();
    };
}


#endif