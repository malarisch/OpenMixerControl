#ifndef _GNU_SOURCE
#define _GNU_SOURCE     // clock_adjtime()
#endif

#include "ptp-status.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/timex.h>
#include <sys/un.h>

#include "../lib_ext/doctest/doctest/doctest.h"

#define PTP4L_SOCKET            "/var/run/ptp4lro"      // read-only management socket of ptp4l (uds_ro_address)
#define PTP4L_CONF              "/etc/aes67/ptp4l.conf" // the PTP domain is taken from here
#define PTP_CLOCK_NAME          "aes67_wallclock"       // PTP hardware clock of the AES67 core (aes67_eth driver)
#define PTP_SILENT_POLLS        3                       // polls without an answer until ptp4l counts as gone

// PTP management message (IEEE 1588-2008, 15.4.1): header, management fields, one TLV
#define PTP_MSG_MANAGEMENT      0x0d
#define PTP_HEADER_LENGTH       34
#define PTP_TLV_OFFSET          48      // header + targetPortIdentity (10) + hops (2) + action (1) + reserved (1)
#define PTP_DATA_OFFSET         54      // TLV: type (2), length (2), managementId (2), data
#define PTP_ACTION_GET          0
#define PTP_ACTION_RESPONSE     2
#define PTP_TLV_MANAGEMENT      0x0001

#define PTP_MID_CURRENT_DATA_SET    0x2001
#define PTP_MID_PARENT_DATA_SET     0x2002
#define PTP_MID_PORT_DATA_SET       0x2004

// a file descriptor of a PTP hardware clock as clock id (see clock_gettime(2), "dynamic clocks")
#define PTP_FD_TO_CLOCKID(fd)   ((clockid_t)((((unsigned int)~(fd)) << 3) | 3))

namespace OMC
{

static uint16_t ReadU16(const uint8_t* p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

static int64_t ReadS64(const uint8_t* p)
{
    uint64_t value = 0;
    for (int i = 0; i < 8; i++)
    {
        value = (value << 8) | p[i];
    }
    return (int64_t)value;
}

// TimeInterval: nanoseconds multiplied by 2^16
static int64_t ReadIntervalNs(const uint8_t* p)
{
    return ReadS64(p) / 65536;
}

PtpStatus::PtpStatus(const char* socketPath)
{
    if (socketPath == nullptr)
    {
        socketPath = getenv("PTP4L_SOCKET");
    }
    serverPath = (socketPath != nullptr) ? socketPath : PTP4L_SOCKET;
}

PtpStatus::~PtpStatus()
{
    Close();
    if (clockFd >= 0)
    {
        close(clockFd);
    }
}

const char* PtpStatus::StateName(PortState state)
{
    switch (state)
    {
        case PortState::INITIALIZING:   return "Initializing";
        case PortState::FAULTY:         return "Faulty";
        case PortState::DISABLED:       return "Disabled";
        case PortState::LISTENING:      return "Listening";
        case PortState::PRE_LEADER:     return "Pre-Leader";
        case PortState::LEADER:         return "Leader";
        case PortState::PASSIVE:        return "Passive";
        case PortState::UNCALIBRATED:   return "Uncalibrated";
        case PortState::FOLLOWER:       return "Follower";
        default:                        return "Unknown";
    }
}

// the notation of LinuxPTP: 001564.fffe.0af954
std::string PtpStatus::IdentityToString(const uint8_t identity[8])
{
    char text[24];
    snprintf(text, sizeof(text), "%02x%02x%02x.%02x%02x.%02x%02x%02x",
        identity[0], identity[1], identity[2], identity[3], identity[4], identity[5], identity[6], identity[7]);
    return text;
}

// ptp4l drops management messages of another domain
uint8_t PtpStatus::ReadDomain()
{
    uint8_t result = 0;
    FILE* file = fopen(PTP4L_CONF, "r");
    if (file != nullptr)
    {
        char line[128];
        while (fgets(line, sizeof(line), file) != nullptr)
        {
            unsigned int value = 0;
            if (sscanf(line, " domainNumber %u", &value) == 1)
            {
                result = (uint8_t)value;
            }
        }
        fclose(file);
    }
    return result;
}

bool PtpStatus::Open()
{
    if (fd >= 0)
    {
        return true;
    }

    // without ptp4l there is nothing to ask: do not even make a socket
    if (access(serverPath.c_str(), F_OK) != 0)
    {
        return false;
    }

    int s = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (s < 0)
    {
        return false;
    }

    // ptp4l answers to the address the question came from: the socket needs a name
    char path[sizeof(((struct sockaddr_un*)0)->sun_path)];
    const char* directory = (access("/var/run", W_OK) == 0) ? "/var/run" : "/tmp";
    snprintf(path, sizeof(path), "%s/omc-ptp.%d", directory, (int)getpid());
    unlink(path);

    struct sockaddr_un address;
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    strncpy(address.sun_path, path, sizeof(address.sun_path) - 1);
    if (bind(s, (struct sockaddr*)&address, sizeof(address)) != 0)
    {
        close(s);
        return false;
    }

    fd = s;
    ownPath = path;
    domain = ReadDomain();
    return true;
}

void PtpStatus::Close()
{
    if (fd >= 0)
    {
        close(fd);
        fd = -1;
        unlink(ownPath.c_str());
    }
}

bool PtpStatus::SendGet(uint16_t managementId)
{
    uint8_t message[PTP_DATA_OFFSET];
    memset(message, 0, sizeof(message));

    message[0] = PTP_MSG_MANAGEMENT;
    message[1] = 2;                                 // versionPTP
    message[2] = 0;                                 // messageLength
    message[3] = sizeof(message);
    message[4] = domain;
    pid_t pid = getpid();                           // sourcePortIdentity: something of our own, like pmc
    message[26] = (uint8_t)(pid >> 8);
    message[27] = (uint8_t)pid;
    message[29] = 1;                                // port number
    sequence++;
    message[30] = (uint8_t)(sequence >> 8);
    message[31] = (uint8_t)sequence;
    message[32] = 4;                                // controlField: management
    message[33] = 0x7f;                             // logMessageInterval
    memset(&message[34], 0xff, 10);                 // targetPortIdentity: every clock, every port
    message[46] = PTP_ACTION_GET;
    message[48] = (uint8_t)(PTP_TLV_MANAGEMENT >> 8);
    message[49] = (uint8_t)PTP_TLV_MANAGEMENT;
    message[51] = 2;                                // TLV length: only the managementId (a GET has no data)
    message[52] = (uint8_t)(managementId >> 8);
    message[53] = (uint8_t)managementId;

    struct sockaddr_un address;
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    strncpy(address.sun_path, serverPath.c_str(), sizeof(address.sun_path) - 1);

    return sendto(fd, message, sizeof(message), 0, (struct sockaddr*)&address, sizeof(address)) == (ssize_t)sizeof(message);
}

void PtpStatus::HandleMessage(const uint8_t* data, size_t length)
{
    if ((length < PTP_DATA_OFFSET) || ((data[0] & 0x0f) != PTP_MSG_MANAGEMENT) || ((data[46] & 0x0f) != PTP_ACTION_RESPONSE)
        || (ReadU16(&data[48]) != PTP_TLV_MANAGEMENT))
    {
        return;
    }

    // TLV length counts the managementId and the data
    size_t tlvLength = ReadU16(&data[50]);
    if ((tlvLength < 2) || (PTP_TLV_OFFSET + 4 + tlvLength > length))
    {
        return;
    }
    size_t dataLength = tlvLength - 2;
    const uint8_t* body = &data[PTP_DATA_OFFSET];

    switch (ReadU16(&data[52]))
    {
        case PTP_MID_CURRENT_DATA_SET:
            // stepsRemoved (2), offsetFromMaster (8), meanPathDelay (8)
            if (dataLength >= 18)
            {
                info.offsetNs = ReadIntervalNs(&body[2]);
                info.pathDelayNs = ReadIntervalNs(&body[10]);
                silentPolls = 0;
                info.online = true;
            }
            break;
        case PTP_MID_PARENT_DATA_SET:
            // parentPortIdentity (10), parentStats (1), reserved (1), variance (2), phase change rate (4),
            // grandmasterPriority1 (1), grandmasterClockQuality (4), grandmasterPriority2 (1), grandmasterIdentity (8)
            if (dataLength >= 32)
            {
                memcpy(info.leader, &body[24], 8);
            }
            break;
        case PTP_MID_PORT_DATA_SET:
            // portIdentity (10), portState (1), ...
            if (dataLength >= 11)
            {
                info.state = (body[10] <= (uint8_t)PortState::FOLLOWER) ? (PortState)body[10] : PortState::UNKNOWN;
            }
            break;
        default:
            break;
    }
}

void PtpStatus::Receive()
{
    uint8_t buffer[512];
    while (fd >= 0)
    {
        ssize_t length = recv(fd, buffer, sizeof(buffer), 0);
        if (length <= 0)
        {
            break;
        }
        HandleMessage(buffer, (size_t)length);
    }
}

// The frequency ptp4l has set on the hardware clock. ptp4l logs the output of its servo as
// "freq" and sets the clock to the negative of it: the sign is turned back here.
void PtpStatus::ReadFrequency()
{
    info.haveFrequency = false;

    if (clockFd < 0)
    {
        DIR* directory = opendir("/sys/class/ptp");
        if (directory == nullptr)
        {
            return;
        }
        struct dirent* entry;
        while ((clockFd < 0) && ((entry = readdir(directory)) != nullptr))
        {
            char path[300];
            char name[64] = "";
            snprintf(path, sizeof(path), "/sys/class/ptp/%s/clock_name", entry->d_name);
            FILE* file = fopen(path, "r");
            if (file == nullptr)
            {
                continue;
            }
            if ((fgets(name, sizeof(name), file) != nullptr) && (strncmp(name, PTP_CLOCK_NAME, strlen(PTP_CLOCK_NAME)) == 0))
            {
                snprintf(path, sizeof(path), "/dev/%s", entry->d_name);
                clockFd = open(path, O_RDWR | O_CLOEXEC);   // older kernels refuse clock_adjtime() on a read-only clock
            }
            fclose(file);
        }
        closedir(directory);
        if (clockFd < 0)
        {
            return;
        }
    }

    struct timex tx;
    memset(&tx, 0, sizeof(tx));     // modes = 0: only read
    if (clock_adjtime(PTP_FD_TO_CLOCKID(clockFd), &tx) < 0)
    {
        // the driver was unloaded: look for the clock again next time
        close(clockFd);
        clockFd = -1;
        return;
    }

    // tx.freq: parts per million with 16 fractional bits
    info.frequencyPpb = (int32_t)(-((int64_t)tx.freq * 1000) / 65536);
    info.haveFrequency = true;
}

void PtpStatus::Poll()
{
    // the answers to the questions of the last poll
    Receive();

    if (silentPolls >= PTP_SILENT_POLLS)
    {
        // ptp4l is gone (or was started again: its socket is a new one)
        info = Info();
        Close();
        silentPolls = 0;
    }

    if (Open())
    {
        silentPolls++;
        if (!SendGet(PTP_MID_PORT_DATA_SET) || !SendGet(PTP_MID_PARENT_DATA_SET) || !SendGet(PTP_MID_CURRENT_DATA_SET))
        {
            silentPolls = PTP_SILENT_POLLS;
        }
    }

    if (info.online)
    {
        ReadFrequency();
    }
    else
    {
        info.haveFrequency = false;
    }
}

// ###########################################################################
// #
// #      Tests
// #
// ###########################################################################

// a management response as ptp4l sends it
static size_t TestResponse(uint8_t* message, uint16_t managementId, const uint8_t* data, size_t dataLength)
{
    memset(message, 0, PTP_DATA_OFFSET);
    message[0] = PTP_MSG_MANAGEMENT;
    message[1] = 2;
    message[46] = PTP_ACTION_RESPONSE;
    message[49] = PTP_TLV_MANAGEMENT;
    message[51] = (uint8_t)(2 + dataLength);
    message[52] = (uint8_t)(managementId >> 8);
    message[53] = (uint8_t)managementId;
    memcpy(&message[PTP_DATA_OFFSET], data, dataLength);
    return PTP_DATA_OFFSET + dataLength;
}

static void TestWriteInterval(uint8_t* p, int64_t ns)
{
    uint64_t value = (uint64_t)(ns * 65536);
    for (int i = 7; i >= 0; i--)
    {
        p[i] = (uint8_t)value;
        value >>= 8;
    }
}

TEST_CASE("PTP status")
{
    SUBCASE("names")
    {
        const uint8_t identity[8] = {0x00, 0x15, 0x64, 0xff, 0xfe, 0x0a, 0xf9, 0x54};
        CHECK(PtpStatus::IdentityToString(identity) == "001564.fffe.0af954");
        CHECK(std::string(PtpStatus::StateName(PtpStatus::PortState::FOLLOWER)) == "Follower");
    }

    SUBCASE("no ptp4l")
    {
        PtpStatus status("/tmp/omc-test-no-such-ptp4l");
        status.Poll();
        CHECK(!status.Get().online);
    }

    SUBCASE("ptp4l answers")
    {
        // a ptp4l that is only a socket
        char serverPath[64];
        snprintf(serverPath, sizeof(serverPath), "/tmp/omc-test-ptp4l.%d", (int)getpid());
        unlink(serverPath);
        int server = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
        REQUIRE(server >= 0);
        struct sockaddr_un address;
        memset(&address, 0, sizeof(address));
        address.sun_family = AF_UNIX;
        strncpy(address.sun_path, serverPath, sizeof(address.sun_path) - 1);
        REQUIRE(bind(server, (struct sockaddr*)&address, sizeof(address)) == 0);

        PtpStatus status(serverPath);
        status.Poll();
        CHECK(!status.Get().online);

        // three questions, all of them GETs without data to every port
        uint16_t asked[3] = {0};
        struct sockaddr_un client;
        socklen_t clientLength = sizeof(client);
        for (int i = 0; i < 3; i++)
        {
            uint8_t request[128];
            clientLength = sizeof(client);
            ssize_t length = recvfrom(server, request, sizeof(request), 0, (struct sockaddr*)&client, &clientLength);
            REQUIRE(length == PTP_DATA_OFFSET);
            CHECK(request[0] == PTP_MSG_MANAGEMENT);
            CHECK(request[1] == 2);
            CHECK(ReadU16(&request[2]) == PTP_DATA_OFFSET);
            CHECK(request[34] == 0xff);
            CHECK(request[43] == 0xff);
            CHECK(request[46] == PTP_ACTION_GET);
            CHECK(ReadU16(&request[48]) == PTP_TLV_MANAGEMENT);
            CHECK(ReadU16(&request[50]) == 2);
            asked[i] = ReadU16(&request[52]);
        }
        CHECK(asked[0] == PTP_MID_PORT_DATA_SET);
        CHECK(asked[1] == PTP_MID_PARENT_DATA_SET);
        CHECK(asked[2] == PTP_MID_CURRENT_DATA_SET);

        uint8_t message[128];
        uint8_t data[64];

        memset(data, 0, sizeof(data));
        data[10] = 9;   // SLAVE
        size_t length = TestResponse(message, PTP_MID_PORT_DATA_SET, data, 26);
        REQUIRE(sendto(server, message, length, 0, (struct sockaddr*)&client, clientLength) == (ssize_t)length);

        memset(data, 0, sizeof(data));
        const uint8_t leader[8] = {0x00, 0x1d, 0xc1, 0xff, 0xfe, 0x12, 0x34, 0x56};
        memcpy(&data[24], leader, 8);
        length = TestResponse(message, PTP_MID_PARENT_DATA_SET, data, 32);
        REQUIRE(sendto(server, message, length, 0, (struct sockaddr*)&client, clientLength) == (ssize_t)length);

        memset(data, 0, sizeof(data));
        TestWriteInterval(&data[2], -137);
        TestWriteInterval(&data[10], 4711);
        length = TestResponse(message, PTP_MID_CURRENT_DATA_SET, data, 18);
        REQUIRE(sendto(server, message, length, 0, (struct sockaddr*)&client, clientLength) == (ssize_t)length);

        // something that is not an answer is ignored
        memset(message, 0xff, sizeof(message));
        sendto(server, message, 20, 0, (struct sockaddr*)&client, clientLength);

        status.Poll();
        CHECK(status.Get().online);
        CHECK(status.Get().state == PtpStatus::PortState::FOLLOWER);
        CHECK(PtpStatus::IdentityToString(status.Get().leader) == "001dc1.fffe.123456");
        CHECK(status.Get().offsetNs == -137);
        CHECK(status.Get().pathDelayNs == 4711);

        // ptp4l stops answering: after a few polls it counts as gone
        for (int i = 0; i < PTP_SILENT_POLLS + 1; i++)
        {
            status.Poll();
        }
        CHECK(!status.Get().online);
        CHECK(status.Get().state == PtpStatus::PortState::UNKNOWN);

        close(server);
        unlink(serverPath);
    }
}

}
