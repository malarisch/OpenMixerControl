#pragma once

#include <stdint.h>
#include <string>

// PtpStatus: what ptp4l (LinuxPTP) knows about the clock it disciplines.
//
// ptp4l answers PTP management messages on a Unix datagram socket, the same way "pmc -u"
// asks it. Three data sets are read: the state of the port, offset and path delay, and the
// grandmaster. The frequency correction is not part of them: it is read from the PTP
// hardware clock ptp4l steers (the wallclock of the AES67 core).
//
// Nothing blocks: Poll() sends the questions and takes the answers that have arrived since
// the last call, so the values are one poll interval old.

namespace OMC
{

class PtpStatus
{
    public:

        // port states of IEEE 1588
        enum class PortState : uint8_t
        {
            UNKNOWN = 0,
            INITIALIZING = 1,
            FAULTY = 2,
            DISABLED = 3,
            LISTENING = 4,
            PRE_LEADER = 5,
            LEADER = 6,         // "MASTER": this clock is the best one on the network
            PASSIVE = 7,
            UNCALIBRATED = 8,
            FOLLOWER = 9        // "SLAVE": this clock follows the leader
        };

        struct Info
        {
            bool online = false;            // ptp4l answers
            PortState state = PortState::UNKNOWN;
            uint8_t leader[8] = {0};        // clock identity of the grandmaster
            int64_t offsetNs = 0;           // offset from the leader
            int64_t pathDelayNs = 0;        // mean path delay
            bool haveFrequency = false;     // the hardware clock could be read
            int32_t frequencyPpb = 0;       // frequency correction as ptp4l logs it ("freq")
        };

        // socketPath: socket of ptp4l, nullptr = environment variable PTP4L_SOCKET or the default
        PtpStatus(const char* socketPath = nullptr);
        ~PtpStatus();

        // call about once per second
        void Poll();
        const Info& Get() const { return info; }

        static const char* StateName(PortState state);
        static std::string IdentityToString(const uint8_t identity[8]);

    private:

        std::string serverPath;
        std::string ownPath;
        int fd = -1;
        int clockFd = -1;
        uint16_t sequence = 0;
        uint8_t domain = 0;
        unsigned int silentPolls = 0;       // polls without an answer
        Info info;

        bool Open();
        void Close();
        void Receive();
        void HandleMessage(const uint8_t* data, size_t length);
        bool SendGet(uint16_t managementId);
        void ReadFrequency();
        static uint8_t ReadDomain();
};

}
