#pragma once

#include <string>

#include "base.h"
#include "aes67-patch.h"
#include "ptp-status.h"

// AES67: connection to the AES67 daemon (aes67d) that controls the AES67 core in the FPGA.
//
// - publishes the streams the daemon discovers on the network as Mixerparameters
//   (AES67_DISCOVERED_*), for the routing page
// - keeps the receive streams of the core in line with the patch (see aes67-patch.h):
//   a stream is subscribed as soon as one of its channels is routed somewhere and
//   dropped when the last one is gone
// - keeps the transmit streams of the core in line with AES67_TX_*: a stream is sent
//   as soon as it has channels
//
// - publishes the state of the core and of ptp4l (AES67_CORE_ADDRESS, AES67_PTP_*), for the
//   setup page. ptp4l is asked directly (see ptp-status.h), the daemon knows nothing about it.
//
// The daemon talks newline-delimited JSON over a Unix socket and pushes events
// (discovered streams, configured streams) after "subscribe_events".

namespace OMC
{

class Aes67 : public X32Base
{
    using enum MP_ID;

    public:

        Aes67(X32BaseParameter* basepar);

        // call while the list of changed Mixerparameters is valid (10ms tick)
        void Sync();
        void Tick100ms();

    private:

        // One receive stream slot of the core.
        struct Slot
        {
            bool used = false;
            std::string address;
            uint port = 0;
            uint channels = 0;
            uint samples = 0;
            uint delay = 0;
            uint8_t input[AES67_INPUTS_MAX] = {0};
            std::string name;

            bool SameFlow(const Slot& other) const;
            bool operator==(const Slot& other) const;
        };

        // One transmit stream slot of the core.
        struct TxSlot
        {
            bool used = false;
            std::string address;
            uint channels = 0;
            uint samples = 0;
            uint8_t output[AES67_OUTPUTS_MAX] = {0};    // AES67 output (zero based) each channel is taken from
            std::string name;

            bool operator==(const TxSlot& other) const;
        };

        std::string socketPath;
        int fd = -1;
        std::string rxBuffer;
        uint64_t nextId = 1;
        uint64_t helloId = 0;

        uint waitTicks = 0;         // 100ms ticks to wait before the next connect / the next try after an error
        bool connectErrorLogged = false;
        bool haveSlots = false;     // the daemon has told us its receive streams
        bool imported = false;      // streams configured in the daemon were taken over into an empty patch
        bool reconcile = false;     // patch or daemon state has changed
        Slot slots[AES67_RX_SLOTS_MAX]; // receive streams as the daemon has them
        TxSlot txSlots[AES67_TX_SLOTS_MAX]; // transmit streams as the daemon has them

        uint64_t rxConfigId = 0;    // requests that ask the core for its build configuration
        uint64_t txConfigId = 0;
        bool haveRxConfig = false;  // the core has told its receive / transmit streams and channels
        bool haveTxConfig = false;
        uint64_t addressId = 0;     // request that asks for the address of the core
        uint addressTicks = 0;      // 100ms ticks until the address is asked for again
        String ownAddress = "";     // IP address of the core ("" = not known yet)

        PtpStatus ptp;              // ptp4l
        uint ptpTicks = 0;          // 100ms ticks until ptp4l is asked again

        void Connect();
        void Disconnect(const char* reason);
        void Receive();
        void HandleLine(const std::string& line);
        void HandleDiscovered(const std::string& json);
        void HandleConfig(const std::string& json);
        bool SendLine(const std::string& line);
        bool SendRequest(const char* method);
        bool SendReadRegister(const char* name);
        void SetCore(MP_ID mp, uint value);
        void SetInt(MP_ID mp, int64_t value);
        void PublishPtp();
        void ClearCore();
        bool SendSetRxStream(uint id, const Slot& slot);
        bool SendStopRxStream(uint id);
        bool SendSetTxStream(uint id, const TxSlot& slot);
        bool SendStopTxStream(uint id);

        void ClearDiscovered(uint first);
        void SetString(MP_ID mp, const String& value, uint index);
        void ImportSlots();
        void ImportTxSlots();
        void Reconcile();
        void ReconcileTx();
        void ClearTxActive();
};

}
