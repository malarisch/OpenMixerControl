/*
    ____                  __   ______ ___  
   / __ \                 \ \ / /___ \__ \ 
  | |  | |_ __   ___ _ __  \ V /  __) | ) |
  | |__| | |_) |  __/ | | |/ . \ ___) / /_ 
   \____/| .__/ \___|_| |_/_/ \_\____/____|
         | |                               
         |_|                               
  
  OpenX32 - The OpenSource Operating System for the Behringer X32 Audio Mixing Console
  Copyright 2025-2026 OpenMixerProject
  https://github.com/OpenMixerProject/OpenX32
  
  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  version 3 as published by the Free Software Foundation.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
*/

#include "aes67.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <optional>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <vector>

#include "../lib_ext/glaze/include/glaze/glaze.hpp"
#include "../lib_ext/doctest/doctest/doctest.h"

#define AES67D_SOCKET           "/run/aes67d.sock"
#define AES67D_PROTO_VERSION    9
#define AES67D_RETRY_TICKS      20  // 2 seconds between two connection attempts
#define AES67D_ERROR_TICKS      50  // 5 seconds before the streams are set again after the daemon refused one
#define AES67D_ADDRESS_TICKS    50  // 5 seconds between two questions for the address of the core
#define AES67_PTP_TICKS         10  // 1 second between two questions to ptp4l
#define AES67D_RX_BUFFER_MAX    (256 * 1024)

// Build configuration of the receive and of the transmit path of the core:
// bits 0..7 = stream slots, bits 8..15 = channels, bits 16..31 = buffer (samples)
#define AES67D_REG_RX_CONFIG    "aes67_csr_system_cfg_rx"
#define AES67D_REG_TX_CONFIG    "aes67_csr_system_cfg_tx"

// The JSON messages of aes67d (crate aes67-proto), reduced to what is used here.
// Numbers are uint32_t on purpose: glaze writes 8 bit types as characters.
namespace
{
    struct JsonRxStream
    {
        uint32_t id = 0;
        std::string dst_ip;
        uint32_t dst_port = 0;
        std::vector<uint32_t> ch_map;
        std::optional<uint32_t> channels;
        uint32_t output_delay = 0;
        uint32_t samples_per_channel = 0;
        std::optional<std::string> name;
    };

    struct JsonTxStream
    {
        uint32_t id = 0;
        std::string dst_ip;
        std::optional<uint32_t> channels;
        uint32_t samples_per_packet = 0;
        std::vector<uint32_t> ch_ids;
        uint32_t ssrc = 0;
        std::optional<std::string> name;
    };

    struct JsonHello
    {
        std::string client;
        uint32_t proto_version = 0;
    };

    struct JsonStreamId
    {
        uint32_t id = 0;
    };

    template <typename T>
    struct JsonCall
    {
        std::string method;
        T params;
    };

    struct JsonRegisterName
    {
        std::string name;
    };

    struct JsonCallNoParams
    {
        std::string method;
    };

    template <typename T>
    struct JsonRequest
    {
        uint64_t id = 0;
        T request;
    };

    struct JsonDiscovered
    {
        std::string session_name;
        std::string dst_ip;
        uint32_t dst_port = 0;
        uint32_t channels = 0;
        uint32_t sample_rate = 0;
        std::string encoding;
        float ptime_ms = 0;
    };

    struct JsonConfig
    {
        std::vector<JsonTxStream> tx_streams;
        std::vector<JsonRxStream> rx_streams;
    };

    struct JsonError
    {
        std::string code;
        std::string message;
    };

    struct JsonOk
    {
        std::string kind;
        glz::raw_json data;
    };

    struct JsonResult
    {
        std::optional<JsonOk> Ok;
        std::optional<JsonError> Err;
    };

    // a line from the daemon: a response has "id" and "result", an event has "event" and "data"
    struct JsonLine
    {
        std::optional<uint64_t> id;
        std::optional<JsonResult> result;
        std::optional<std::string> event;
        glz::raw_json data;
    };

    template <typename T>
    bool ReadJson(T& value, const std::string& json)
    {
        return !glz::read<glz::opts{.error_on_unknown_keys = false}>(value, json);
    }
}

namespace OMC
{

bool Aes67::Slot::SameFlow(const Slot& other) const
{
    return (port == other.port) && (address == other.address);
}

bool Aes67::Slot::operator==(const Slot& other) const
{
    if (used != other.used)
    {
        return false;
    }
    if (!used)
    {
        return true;
    }
    if (!SameFlow(other) || (channels != other.channels) || (samples != other.samples) || (delay != other.delay) || (name != other.name))
    {
        return false;
    }
    for (uint channel = 0; (channel < channels) && (channel < AES67_INPUTS_MAX); channel++)
    {
        if (input[channel] != other.input[channel])
        {
            return false;
        }
    }

    return true;
}

bool Aes67::TxSlot::operator==(const TxSlot& other) const
{
    if (used != other.used)
    {
        return false;
    }
    if (!used)
    {
        return true;
    }
    if ((address != other.address) || (channels != other.channels) || (samples != other.samples) || (name != other.name))
    {
        return false;
    }
    for (uint channel = 0; (channel < channels) && (channel < AES67_OUTPUTS_MAX); channel++)
    {
        if (output[channel] != other.output[channel])
        {
            return false;
        }
    }

    return true;
}

Aes67::Aes67(X32BaseParameter* basepar): X32Base(basepar)
{
    // another socket for tests on a PC
    const char* path = getenv("AES67D_SOCKET");
    socketPath = (path != nullptr) ? path : AES67D_SOCKET;
}

void Aes67::Sync()
{
    if (config->HasParametersChanged({ROUTING_FPGA, AES67_INPUT_STREAM, AES67_INPUT_NAME, AES67_INPUT_CHANNEL, AES67_RX_DELAY,
            AES67_TX_CHANNELS, AES67_TX_ADDRESS, AES67_TX_NAME}))
    {
        reconcile = true;
    }
}

void Aes67::Tick100ms()
{
    if (config->IsModelAnyWing())
    {
        return;
    }

    // ptp4l runs beside the daemon: its state is shown whether the daemon is there or not
    if (ptpTicks > 0)
    {
        ptpTicks--;
    }
    else
    {
        ptpTicks = AES67_PTP_TICKS - 1;
        ptp.Poll();
        PublishPtp();
    }

    if (waitTicks > 0)
    {
        waitTicks--;
        return;
    }

    if (fd < 0)
    {
        Connect();
        return;
    }

    Receive();

    // the automatic addresses of the transmit streams follow the address of the core (DHCP)
    if (fd >= 0)
    {
        if (addressTicks > 0)
        {
            addressTicks--;
        }
        else
        {
            addressTicks = AES67D_ADDRESS_TICKS;
            addressId = nextId;
            SendRequest("get_ip");
        }
    }

    // nothing is set up before the core has told what it has
    if ((fd >= 0) && haveSlots && haveRxConfig && haveTxConfig)
    {
        if (!imported)
        {
            imported = true;
            ImportSlots();
            ImportTxSlots();
            reconcile = true;
        }

        if (reconcile)
        {
            reconcile = false;
            Reconcile();
            if (fd >= 0)
            {
                ReconcileTx();
            }
        }
    }
}

// ###########################################################################
// #
// #      Connection
// #
// ###########################################################################

void Aes67::Connect()
{
    struct sockaddr_un address;
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    strncpy(address.sun_path, socketPath.c_str(), sizeof(address.sun_path) - 1);

    int handle = socket(AF_UNIX, SOCK_STREAM, 0);
    if (handle < 0)
    {
        waitTicks = AES67D_RETRY_TICKS;
        return;
    }

    // the daemon is not there as long as the AES67 stack is not started: no error, just try again
    if (connect(handle, (struct sockaddr*)&address, sizeof(address)) != 0)
    {
        if (!connectErrorLogged)
        {
            helper->DEBUG_FPGA(DEBUGLEVEL_NORMAL, "AES67: no daemon at %s (%s), waiting for it", socketPath.c_str(), strerror(errno));
            connectErrorLogged = true;
        }
        close(handle);
        waitTicks = AES67D_RETRY_TICKS;
        return;
    }

    fcntl(handle, F_SETFL, fcntl(handle, F_GETFL, 0) | O_NONBLOCK);

    fd = handle;
    rxBuffer.clear();
    haveSlots = false;
    haveRxConfig = false;
    haveTxConfig = false;
    connectErrorLogged = false;
    addressTicks = 0;

    JsonRequest<JsonCall<JsonHello>> hello;
    hello.id = helloId = nextId++;
    hello.request.method = "hello";
    hello.request.params.client = "omc";
    hello.request.params.proto_version = AES67D_PROTO_VERSION;

    // after this the daemon sends the discovered and the configured streams, and again whenever they change
    if (!SendLine(glz::write_json(hello).value_or("")) || !SendRequest("subscribe_events"))
    {
        return;
    }

    // what the loaded core has: streams, channels, buffer
    rxConfigId = nextId;
    if (!SendReadRegister(AES67D_REG_RX_CONFIG))
    {
        return;
    }
    txConfigId = nextId;
    if (!SendReadRegister(AES67D_REG_TX_CONFIG))
    {
        return;
    }

    helper->Log("AES67: connected to daemon at %s\n", socketPath.c_str());
    config->Set(AES67_ONLINE, 1.0f);
}

void Aes67::Disconnect(const char* reason)
{
    if (fd >= 0)
    {
        close(fd);
        fd = -1;
        helper->Error("AES67: connection to daemon closed (%s)\n", reason);
    }

    haveSlots = false;
    waitTicks = AES67D_RETRY_TICKS;

    if (config->GetBool(AES67_ONLINE))
    {
        config->Set(AES67_ONLINE, 0.0f);
    }
    ClearDiscovered(0);
    ClearTxActive();
    SetString(AES67_CORE_ADDRESS, "", 0);

    // another bitstream may be loaded until the daemon is back
    haveRxConfig = false;
    haveTxConfig = false;
    ClearCore();
}

void Aes67::Receive()
{
    char buffer[4096];

    while (fd >= 0)
    {
        ssize_t count = recv(fd, buffer, sizeof(buffer), 0);
        if (count > 0)
        {
            rxBuffer.append(buffer, count);
            if (rxBuffer.size() > AES67D_RX_BUFFER_MAX)
            {
                Disconnect("line too long");
            }
        }
        else if (count == 0)
        {
            Disconnect("daemon has gone");
        }
        else if ((errno == EAGAIN) || (errno == EWOULDBLOCK))
        {
            break;
        }
        else if (errno != EINTR)
        {
            Disconnect(strerror(errno));
        }
    }

    size_t start = 0;
    size_t end;
    while ((fd >= 0) && ((end = rxBuffer.find('\n', start)) != std::string::npos))
    {
        HandleLine(rxBuffer.substr(start, end - start));
        start = end + 1;
    }
    if (fd >= 0)
    {
        rxBuffer.erase(0, start);
    }
}

bool Aes67::SendLine(const std::string& line)
{
    if ((fd < 0) || line.empty())
    {
        return false;
    }

    helper->DEBUG_FPGA(DEBUGLEVEL_VERBOSE, "AES67 -> %s", line.c_str());

    std::string data = line + "\n";
    size_t sent = 0;
    while (sent < data.size())
    {
        // requests are small and the daemon reads all the time, so the socket buffer is never full
        ssize_t count = send(fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
        if (count > 0)
        {
            sent += count;
        }
        else if (errno != EINTR)
        {
            Disconnect(strerror(errno));
            return false;
        }
    }

    return true;
}

bool Aes67::SendRequest(const char* method)
{
    JsonRequest<JsonCallNoParams> request;
    request.id = nextId++;
    request.request.method = method;

    return SendLine(glz::write_json(request).value_or(""));
}

bool Aes67::SendReadRegister(const char* name)
{
    JsonRequest<JsonCall<JsonRegisterName>> request;
    request.id = nextId++;
    request.request.method = "read_register";
    request.request.params.name = name;

    return SendLine(glz::write_json(request).value_or(""));
}

bool Aes67::SendSetRxStream(uint id, const Slot& slot)
{
    JsonRequest<JsonCall<JsonRxStream>> request;
    request.id = nextId++;
    request.request.method = "set_rx_stream";

    JsonRxStream& params = request.request.params;
    params.id = id;
    params.dst_ip = slot.address;
    params.dst_port = slot.port;
    for (uint channel = 0; channel < slot.channels; channel++)
    {
        params.ch_map.push_back(slot.input[channel]);
    }
    params.channels = slot.channels;
    params.output_delay = slot.delay;
    params.samples_per_channel = slot.samples;
    if (!slot.name.empty())
    {
        params.name = slot.name;
    }

    return SendLine(glz::write_json(request).value_or(""));
}

bool Aes67::SendStopRxStream(uint id)
{
    JsonRequest<JsonCall<JsonStreamId>> request;
    request.id = nextId++;
    request.request.method = "stop_rx_stream";
    request.request.params.id = id;

    return SendLine(glz::write_json(request).value_or(""));
}

bool Aes67::SendSetTxStream(uint id, const TxSlot& slot)
{
    JsonRequest<JsonCall<JsonTxStream>> request;
    request.id = nextId++;
    request.request.method = "set_tx_stream";

    JsonTxStream& params = request.request.params;
    params.id = id;
    params.dst_ip = slot.address;
    params.channels = slot.channels;
    params.samples_per_packet = slot.samples;
    for (uint channel = 0; channel < slot.channels; channel++)
    {
        params.ch_ids.push_back(slot.output[channel]);
    }
    params.name = slot.name;

    return SendLine(glz::write_json(request).value_or(""));
}

bool Aes67::SendStopTxStream(uint id)
{
    JsonRequest<JsonCall<JsonStreamId>> request;
    request.id = nextId++;
    request.request.method = "stop_tx_stream";
    request.request.params.id = id;

    return SendLine(glz::write_json(request).value_or(""));
}

// ###########################################################################
// #
// #      Messages from the daemon
// #
// ###########################################################################

void Aes67::HandleLine(const std::string& text)
{
    helper->DEBUG_FPGA(DEBUGLEVEL_VERBOSE, "AES67 <- %s", text.c_str());

    JsonLine line;
    if (!ReadJson(line, text))
    {
        helper->Error("AES67: can not read message of the daemon: %s\n", text.c_str());
        return;
    }

    if (line.event)
    {
        if (*line.event == "discovered")
        {
            HandleDiscovered(line.data.str);
        }
        else if (*line.event == "config")
        {
            HandleConfig(line.data.str);
        }
        return;
    }

    if (!line.result)
    {
        return;
    }

    if (line.result->Err)
    {
        if (line.id && (*line.id == addressId))
        {
            // the address of the core can not be read at the moment: asked again soon
            return;
        }
        if (line.id && ((*line.id == rxConfigId) || (*line.id == txConfigId)))
        {
            // a core without these registers: nothing is known about it, so nothing is received or sent
            bool rx = (*line.id == rxConfigId);
            helper->Error("AES67: the core does not tell its %s configuration (%s), no %s streams\n",
                rx ? "receive" : "transmit", line.result->Err->message.c_str(), rx ? "receive" : "transmit");
            if (rx)
            {
                haveRxConfig = true;
                SetCore(AES67_RX_STREAMS, 0);
                SetCore(AES67_RX_INPUTS, 0);
                SetCore(AES67_RX_BUFFER, 0);
            }
            else
            {
                haveTxConfig = true;
                SetCore(AES67_TX_STREAMS, 0);
                SetCore(AES67_TX_OUTPUTS, 0);
            }
            return;
        }

        helper->Error("AES67: daemon reports %s: %s\n", line.result->Err->code.c_str(), line.result->Err->message.c_str());

        if (line.id && (*line.id == helloId))
        {
            // most likely another protocol version: this will not get better by trying again soon
            Disconnect("daemon refused the connection");
            waitTicks = 30 * 10;
            return;
        }

        // A stream was refused. What the daemon really has is unknown now: ask, and set the streams again later.
        haveSlots = false;
        waitTicks = AES67D_ERROR_TICKS;
        SendRequest("get_config");
    }
    else if (line.result->Ok && (line.result->Ok->kind == "config"))
    {
        HandleConfig(line.result->Ok->data.str);
        reconcile = true;
    }
    else if (line.result->Ok && (line.result->Ok->kind == "register_value") && line.id && ((*line.id == rxConfigId) || (*line.id == txConfigId)))
    {
        uint64_t value = 0;
        if (ReadJson(value, line.result->Ok->data.str))
        {
            uint streams = value & 0xff;
            uint channels = (value >> 8) & 0xff;
            uint buffer = (value >> 16) & 0xffff;

            if (*line.id == rxConfigId)
            {
                helper->Log("AES67: the core receives %d streams, %d channels, buffer %d samples\n", streams, channels, buffer);
                if ((streams > AES67_RX_SLOTS_MAX) || (channels > AES67_INPUTS_MAX))
                {
                    helper->Error("AES67: only %d receive streams and %d receive channels are used\n", AES67_RX_SLOTS_MAX, AES67_INPUTS_MAX);
                }
                SetCore(AES67_RX_STREAMS, (streams < AES67_RX_SLOTS_MAX) ? streams : AES67_RX_SLOTS_MAX);
                SetCore(AES67_RX_INPUTS, (channels < AES67_INPUTS_MAX) ? channels : AES67_INPUTS_MAX);
                SetCore(AES67_RX_BUFFER, buffer);
                haveRxConfig = true;
            }
            else
            {
                helper->Log("AES67: the core sends %d streams, %d channels\n", streams, channels);
                if ((streams > AES67_TX_SLOTS_MAX) || (channels > AES67_OUTPUTS_MAX))
                {
                    helper->Error("AES67: only %d transmit streams and %d transmit channels are used\n", AES67_TX_SLOTS_MAX, AES67_OUTPUTS_MAX);
                }
                SetCore(AES67_TX_STREAMS, (streams < AES67_TX_SLOTS_MAX) ? streams : AES67_TX_SLOTS_MAX);
                SetCore(AES67_TX_OUTPUTS, (channels < AES67_OUTPUTS_MAX) ? channels : AES67_OUTPUTS_MAX);
                haveTxConfig = true;
            }
            reconcile = true;
        }
    }
    else if (line.result->Ok && (line.result->Ok->kind == "ip"))
    {
        std::string address;
        bool valid = ReadJson(address, line.result->Ok->data.str);
        if (valid)
        {
            // for the setup page (cleared while the daemon is away, so it is set on every answer)
            SetString(AES67_CORE_ADDRESS, String(address.c_str()), 0);
        }
        if (valid && (ownAddress != String(address.c_str())))
        {
            ownAddress = String(address.c_str());
            helper->Log("AES67: address of the core is %s\n", address.c_str());
            reconcile = true;
        }
    }
}

void Aes67::SetCore(MP_ID mp, uint value)
{
    if (config->GetUint(mp) != value)
    {
        config->Set(mp, (float)value);
    }
}

void Aes67::SetInt(MP_ID mp, int64_t value)
{
    if (value > AES67_PTP_NS_LIMIT)
    {
        value = AES67_PTP_NS_LIMIT;
    }
    else if (value < -AES67_PTP_NS_LIMIT)
    {
        value = -AES67_PTP_NS_LIMIT;
    }
    if (config->GetInt(mp) != (int)value)
    {
        config->Set(mp, (float)value);
    }
}

void Aes67::PublishPtp()
{
    const PtpStatus::Info& info = ptp.Get();

    SetString(AES67_PTP_STATE, info.online ? String(PtpStatus::StateName(info.state)) : String(""), 0);
    SetString(AES67_PTP_LEADER, info.online ? String(PtpStatus::IdentityToString(info.leader).c_str()) : String(""), 0);
    SetInt(AES67_PTP_OFFSET, info.online ? info.offsetNs : 0);
    SetInt(AES67_PTP_PATH_DELAY, info.online ? info.pathDelayNs : 0);
    SetInt(AES67_PTP_FREQUENCY, info.haveFrequency ? info.frequencyPpb : 0);
    if (config->GetBool(AES67_PTP_FREQUENCY_VALID) != info.haveFrequency)
    {
        config->Set(AES67_PTP_FREQUENCY_VALID, info.haveFrequency ? 1.0f : 0.0f);
    }
}

void Aes67::ClearCore()
{
    SetCore(AES67_RX_STREAMS, 0);
    SetCore(AES67_RX_INPUTS, 0);
    SetCore(AES67_RX_BUFFER, 0);
    SetCore(AES67_TX_STREAMS, 0);
    SetCore(AES67_TX_OUTPUTS, 0);
}

void Aes67::SetString(MP_ID mp, const String& value, uint index)
{
    if (config->GetString(mp, index) != value)
    {
        config->Set(mp, value, index);
    }
}

void Aes67::ClearDiscovered(uint first)
{
    for (uint i = first; i < AES67_DISCOVERED_MAX; i++)
    {
        SetString(AES67_DISCOVERED_STREAM, "", i);
        SetString(AES67_DISCOVERED_NAME, "", i);
    }
    if (config->GetUint(AES67_DISCOVERED_COUNT) != first)
    {
        config->Set(AES67_DISCOVERED_COUNT, (float)first);
    }
}

// the complete list of streams on the network
void Aes67::HandleDiscovered(const std::string& json)
{
    std::vector<JsonDiscovered> discovered;
    if (!ReadJson(discovered, json))
    {
        helper->Error("AES67: can not read the list of discovered streams\n");
        return;
    }

    uint count = 0;
    for (auto const& entry : discovered)
    {
        // the core receives L24 at the samplerate of the mixer with up to 8 channels
        Aes67Stream stream;
        stream.address = String(entry.dst_ip.c_str());
        stream.port = entry.dst_port;
        stream.channels = entry.channels;
        stream.samples = (uint)lroundf(entry.ptime_ms * (float)entry.sample_rate / 1000.0f);
        if ((entry.encoding != "L24") || (entry.sample_rate != 48000) || !stream.Valid())
        {
            helper->DEBUG_FPGA(DEBUGLEVEL_NORMAL, "AES67: stream %s (%s %d Hz, %d channels) can not be received",
                entry.session_name.c_str(), entry.encoding.c_str(), entry.sample_rate, entry.channels);
            continue;
        }
        if (count >= AES67_DISCOVERED_MAX)
        {
            break;
        }

        String name = String(entry.session_name.c_str());
        SetString(AES67_DISCOVERED_STREAM, stream.Descriptor(), count);
        SetString(AES67_DISCOVERED_NAME, name, count);
        count++;

        // a stream in the patch may have got a new name or format since the scene was stored
        for (uint input = 0; input < Aes67Patch::Inputs(config); input++)
        {
            if (Aes67Patch::InputStream(config, input).SameFlow(stream))
            {
                SetString(AES67_INPUT_STREAM, stream.Descriptor(), input);
                SetString(AES67_INPUT_NAME, name, input);
            }
        }
    }

    ClearDiscovered(count);
}

// the receive streams as the daemon has them (also changed by other clients like aes67cfg or the web interface)
void Aes67::HandleConfig(const std::string& json)
{
    JsonConfig daemonConfig;
    if (!ReadJson(daemonConfig, json))
    {
        helper->Error("AES67: can not read the stream configuration of the daemon\n");
        return;
    }

    Slot seen[AES67_RX_SLOTS_MAX];
    for (auto const& entry : daemonConfig.rx_streams)
    {
        if (entry.id >= AES67_RX_SLOTS_MAX)
        {
            continue;
        }

        Slot& slot = seen[entry.id];
        slot.used = true;
        slot.address = entry.dst_ip;
        slot.port = entry.dst_port;
        slot.channels = (entry.channels && (*entry.channels > 0)) ? *entry.channels : entry.ch_map.size();
        if (slot.channels > AES67_INPUTS_MAX)
        {
            slot.channels = AES67_INPUTS_MAX;
        }
        slot.samples = entry.samples_per_channel;
        slot.delay = entry.output_delay;
        for (uint channel = 0; (channel < slot.channels) && (channel < entry.ch_map.size()); channel++)
        {
            slot.input[channel] = entry.ch_map[channel];
        }
        slot.name = entry.name.value_or("");
    }

    TxSlot txSeen[AES67_TX_SLOTS_MAX];
    for (auto const& entry : daemonConfig.tx_streams)
    {
        if (entry.id >= AES67_TX_SLOTS_MAX)
        {
            continue;
        }

        TxSlot& slot = txSeen[entry.id];
        slot.used = true;
        slot.address = entry.dst_ip;
        slot.channels = (entry.channels && (*entry.channels > 0)) ? *entry.channels : entry.ch_ids.size();
        if (slot.channels > AES67_OUTPUTS_MAX)
        {
            slot.channels = AES67_OUTPUTS_MAX;
        }
        slot.samples = entry.samples_per_packet;
        for (uint channel = 0; (channel < slot.channels) && (channel < entry.ch_ids.size()); channel++)
        {
            slot.output[channel] = entry.ch_ids[channel];
        }
        slot.name = entry.name.value_or("");
    }

    bool changed = !haveSlots;
    for (uint id = 0; id < AES67_RX_SLOTS_MAX; id++)
    {
        if (!(seen[id] == slots[id]))
        {
            changed = true;
        }
        slots[id] = seen[id];
    }
    for (uint id = 0; id < AES67_TX_SLOTS_MAX; id++)
    {
        if (!(txSeen[id] == txSlots[id]))
        {
            changed = true;
        }
        txSlots[id] = txSeen[id];
    }
    haveSlots = true;

    if (changed)
    {
        reconcile = true;
    }
}

// Streams that were set up in the daemon by hand (aes67cfg, web interface) before the
// patch existed are taken over once, so that they do not get lost with the first start.
void Aes67::ImportSlots()
{
    for (uint input = 0; input < Aes67Patch::Inputs(config); input++)
    {
        if (config->GetString(AES67_INPUT_STREAM, input).length() > 0)
        {
            return;
        }
    }

    for (uint id = 0; id < Aes67Patch::RxStreams(config); id++)
    {
        const Slot& slot = slots[id];

        Aes67Stream stream;
        stream.address = String(slot.address.c_str());
        stream.port = slot.port;
        stream.channels = slot.channels;
        stream.samples = slot.samples;
        if (!slot.used || !stream.Valid())
        {
            continue;
        }

        for (uint channel = 0; channel < slot.channels; channel++)
        {
            uint input = slot.input[channel];
            if ((input < Aes67Patch::Inputs(config)) && (config->GetString(AES67_INPUT_STREAM, input).length() == 0))
            {
                helper->Log("AES67: taking over %s channel %d on AES67 input %d\n", stream.Descriptor().c_str(), channel + 1, input + 1);
                config->Set(AES67_INPUT_STREAM, stream.Descriptor(), input);
                config->Set(AES67_INPUT_NAME, String(slot.name.c_str()), input);
                config->Set(AES67_INPUT_CHANNEL, (float)channel, input);
            }
        }
    }
}

// ###########################################################################
// #
// #      Patch -> receive streams of the core
// #
// ###########################################################################

void Aes67::Reconcile()
{
    vector<Aes67PlanStream> plan = Aes67Patch::Plan(config);
    uint delay = config->GetUint(AES67_RX_DELAY);

    Slot wanted[AES67_RX_SLOTS_MAX];
    vector<Slot> unplaced;

    for (auto const& entry : plan)
    {
        if (!entry.ok)
        {
            helper->Error("AES67: no AES67 input left for the unused channels of %s, stream is not received\n", entry.stream.Descriptor().c_str());
            continue;
        }

        Slot slot;
        slot.used = true;
        slot.address = entry.stream.address.c_str();
        slot.port = entry.stream.port;
        slot.channels = entry.stream.channels;
        slot.samples = entry.stream.samples;
        uint delayMax = Aes67Patch::RxDelayMax(config, slot.samples);
        slot.delay = (delay < delayMax) ? delay : delayMax;
        slot.name = entry.name.c_str();
        for (uint channel = 0; channel < slot.channels; channel++)
        {
            slot.input[channel] = entry.input[channel];
        }

        // a stream that is already received stays in its slot: changing the places of its channels does not interrupt it
        bool placed = false;
        for (uint id = 0; id < Aes67Patch::RxStreams(config); id++)
        {
            if (slots[id].used && slots[id].SameFlow(slot) && !wanted[id].used)
            {
                wanted[id] = slot;
                placed = true;
                break;
            }
        }
        if (!placed)
        {
            unplaced.push_back(slot);
        }
    }

    for (auto const& slot : unplaced)
    {
        // preferably a slot that is empty right now, otherwise one that gets free
        int target = -1;
        for (uint id = 0; (id < Aes67Patch::RxStreams(config)) && (target < 0); id++)
        {
            if (!wanted[id].used && !slots[id].used)
            {
                target = id;
            }
        }
        for (uint id = 0; (id < Aes67Patch::RxStreams(config)) && (target < 0); id++)
        {
            if (!wanted[id].used)
            {
                target = id;
            }
        }
        if (target < 0)
        {
            helper->Error("AES67: no receive slot left for %s:%d\n", slot.address.c_str(), slot.port);
            continue;
        }
        wanted[target] = slot;
    }

    // first drop what is not needed any more, then move channels of running streams, then start new streams
    for (uint id = 0; id < Aes67Patch::RxStreams(config); id++)
    {
        if (slots[id].used && !wanted[id].used)
        {
            helper->Log("AES67: slot %d: stop %s:%d\n", id, slots[id].address.c_str(), slots[id].port);
            if (!SendStopRxStream(id))
            {
                return;
            }
        }
    }
    for (uint pass = 0; pass < 2; pass++)
    {
        for (uint id = 0; id < Aes67Patch::RxStreams(config); id++)
        {
            bool isNew = !slots[id].used || !slots[id].SameFlow(wanted[id]);
            if (wanted[id].used && !(wanted[id] == slots[id]) && (isNew == (pass == 1)))
            {
                helper->Log("AES67: slot %d: receive %s:%d (%s)\n", id, wanted[id].address.c_str(), wanted[id].port, wanted[id].name.c_str());
                if (!SendSetRxStream(id, wanted[id]))
                {
                    return;
                }
            }
        }
    }

    // the daemon confirms with a "config" event; until then this is what it has
    for (uint id = 0; id < Aes67Patch::RxStreams(config); id++)
    {
        slots[id] = wanted[id];
    }
}

// ###########################################################################
// #
// #      AES67_TX_* -> transmit streams of the core
// #
// ###########################################################################

// Transmit streams that were set up in the daemon by hand are taken over once, as long
// as no transmit stream is set up here. Their channels are put in a row on the AES67
// outputs (see aes67-patch.h), the sources they have in the routing matrix move along.
void Aes67::ImportTxSlots()
{
    for (uint stream = 0; stream < Aes67Patch::TxStreams(config); stream++)
    {
        if (config->GetUint(AES67_TX_CHANNELS, stream) > 0)
        {
            return;
        }
    }

    const uint firstDestination = FPGA_OUTPUT_IDX_AES67 - 1;
    uint oldSources[AES67_OUTPUTS_MAX];
    uint newSources[AES67_OUTPUTS_MAX] = {0};
    for (uint output = 0; output < Aes67Patch::Outputs(config); output++)
    {
        oldSources[output] = config->GetUint(ROUTING_FPGA, firstDestination + output);
    }

    uint next = 0;
    for (uint id = 0; id < Aes67Patch::TxStreams(config); id++)
    {
        const TxSlot& slot = txSlots[id];
        if (!slot.used || (slot.channels == 0) || (next >= Aes67Patch::Outputs(config)))
        {
            continue;
        }

        uint channels = slot.channels;
        if (channels > Aes67Patch::Outputs(config) - next)
        {
            channels = Aes67Patch::Outputs(config) - next;
        }

        helper->Log("AES67: taking over transmit stream %d to %s with %d channels\n", id + 1, slot.address.c_str(), channels);
        config->Set(AES67_TX_CHANNELS, (float)channels, id);
        config->Set(AES67_TX_ADDRESS, String(slot.address.c_str()), id);
        config->Set(AES67_TX_NAME, String(slot.name.c_str()), id);

        for (uint channel = 0; channel < channels; channel++)
        {
            newSources[next++] = (slot.output[channel] < Aes67Patch::Outputs(config)) ? oldSources[slot.output[channel]] : 0;
        }
    }

    if (next == 0)
    {
        return;
    }
    for (uint output = 0; output < Aes67Patch::Outputs(config); output++)
    {
        if (newSources[output] != oldSources[output])
        {
            config->Set(ROUTING_FPGA, (float)newSources[output], firstDestination + output);
        }
    }
}

void Aes67::ClearTxActive()
{
    for (uint stream = 0; stream < AES67_TX_SLOTS_MAX; stream++)
    {
        SetString(AES67_TX_ACTIVE_ADDRESS, "", stream);
    }
}

void Aes67::ReconcileTx()
{
    // only the streams the core has: it refuses the others
    uint streams = Aes67Patch::TxStreams(config);
    for (uint id = 0; id < streams; id++)
    {
        TxSlot wanted;

        uint channels = Aes67Patch::TxChannels(config, id);
        if (channels > 0)
        {
            String address = config->GetString(AES67_TX_ADDRESS, id);
            if (!Aes67Patch::IsMulticastAddress(address))
            {
                if (address.length() > 0)
                {
                    helper->Error("AES67: %s is no multicast address, transmit stream %d gets an automatic one\n", address.c_str(), id + 1);
                }
                address = Aes67Patch::TxAutoAddress(ownAddress, id);
                if (address.length() == 0)
                {
                    // the address of the core is not known yet: this comes again as soon as it is
                    SetString(AES67_TX_ACTIVE_ADDRESS, "", id);
                    continue;
                }
            }

            String name = config->GetString(AES67_TX_NAME, id);
            if (name.length() == 0)
            {
                name = String("OpenX32 TX ") + String(id + 1);
            }

            uint first = Aes67Patch::TxFirstOutput(config, id);
            wanted.used = true;
            wanted.address = address.c_str();
            wanted.channels = channels;
            wanted.samples = AES67_TX_SAMPLES;
            wanted.name = name.c_str();
            for (uint channel = 0; channel < channels; channel++)
            {
                wanted.output[channel] = first + channel;
            }
        }

        SetString(AES67_TX_ACTIVE_ADDRESS, String(wanted.address.c_str()), id);

        if (wanted == txSlots[id])
        {
            continue;
        }

        if (wanted.used)
        {
            helper->Log("AES67: transmit stream %d: %d channels to %s (%s)\n", id + 1, wanted.channels, wanted.address.c_str(), wanted.name.c_str());
            if (!SendSetTxStream(id, wanted))
            {
                return;
            }
        }
        else
        {
            helper->Log("AES67: transmit stream %d: stop\n", id + 1);
            if (!SendStopTxStream(id))
            {
                return;
            }
        }

        // the daemon confirms with a "config" event; until then this is what it has
        txSlots[id] = wanted;
    }
}

// ###########################################################################
// #
// #      Tests
// #
// ###########################################################################

TEST_CASE("AES67 patch")
{
    Helper* testHelper = new Helper();
    Config* c = new Config("X32C", testHelper, false);

    // the core of the X32 bitstream; the transmit streams are set where they are tested
    c->Set(MP_ID::AES67_RX_STREAMS, 8.0f);
    c->Set(MP_ID::AES67_RX_INPUTS, 8.0f);
    c->Set(MP_ID::AES67_RX_BUFFER, 256.0f);
    c->Set(MP_ID::AES67_TX_OUTPUTS, 8.0f);

    const uint first = FPGA_INPUT_IDX_AES67;
    const uint destination = 72; // DSP input 1

    Aes67Stream stereo = Aes67Stream::Parse("239.1.0.9:5004/2/48");
    Aes67Stream eight = Aes67Stream::Parse("239.69.0.77:5004/8/48");

    // what the routing page does
    auto route = [&](const Aes67Stream& stream, uint channel, uint dest) -> uint
    {
        uint source = Aes67Patch::PatchChannel(c, stream, "Name", channel, dest);
        if (source != 0)
        {
            c->Set(MP_ID::ROUTING_FPGA, (float)source, dest);
        }
        return source;
    };

    SUBCASE("descriptor")
    {
        CHECK(stereo.Valid());
        CHECK(stereo.channels == 2);
        CHECK(stereo.samples == 48);
        CHECK(stereo.port == 5004);
        CHECK(stereo.Descriptor() == String("239.1.0.9:5004/2/48"));
        CHECK(!Aes67Stream::Parse("").Valid());
        CHECK(!Aes67Stream::Parse("239.1.0.9:5004/33/48").Valid());
    }

    SUBCASE("nothing routed, nothing received")
    {
        c->Set(MP_ID::AES67_INPUT_STREAM, stereo.Descriptor(), 0);
        CHECK(Aes67Patch::Plan(c).size() == 0);
    }

    SUBCASE("one channel: the other one is parked on an unused input")
    {
        CHECK(route(stereo, 0, destination) == first);

        vector<Aes67PlanStream> plan = Aes67Patch::Plan(c);
        REQUIRE(plan.size() == 1);
        CHECK(plan[0].ok);
        CHECK(plan[0].wanted[0]);
        CHECK(!plan[0].wanted[1]);
        CHECK(plan[0].input[0] == 0);
        CHECK(plan[0].input[1] == Aes67Patch::Inputs(c) - 1);
        CHECK(Aes67Patch::RoutingLabel(c, destination) == String("Name 1"));
    }

    SUBCASE("nothing is parked on an input a destination listens to")
    {
        // a destination on the last AES67 input, which has no stream
        c->Set(MP_ID::ROUTING_FPGA, (float)(first + Aes67Patch::Inputs(c) - 1), destination + 5);
        CHECK(route(stereo, 0, destination) == first);

        vector<Aes67PlanStream> plan = Aes67Patch::Plan(c);
        REQUIRE(plan.size() == 1);
        CHECK(plan[0].input[1] == Aes67Patch::Inputs(c) - 2);
    }

    SUBCASE("second channel gets its own input, a channel in use is shared")
    {
        CHECK(route(stereo, 0, destination) == first);
        CHECK(route(stereo, 1, destination + 1) == first + 1);
        CHECK(route(stereo, 1, destination + 2) == first + 1);

        vector<Aes67PlanStream> plan = Aes67Patch::Plan(c);
        REQUIRE(plan.size() == 1);
        CHECK(plan[0].input[0] == 0);
        CHECK(plan[0].input[1] == 1);
    }

    SUBCASE("stepping the channel of one destination keeps its input")
    {
        CHECK(route(eight, 0, destination) == first);
        CHECK(route(eight, 1, destination) == first);
        CHECK(route(eight, 2, destination) == first);
        CHECK(c->GetUint(MP_ID::AES67_INPUT_CHANNEL, 0) == 2);

        vector<Aes67PlanStream> plan = Aes67Patch::Plan(c);
        REQUIRE(plan.size() == 1);
        CHECK(plan[0].wanted[2]);
        CHECK(plan[0].input[2] == 0);
    }

    SUBCASE("an input another destination listens to is not taken away")
    {
        CHECK(route(stereo, 0, destination) == first);
        CHECK(route(stereo, 0, destination + 1) == first);
        // destination + 1 moves on, destination keeps channel 1
        CHECK(route(stereo, 1, destination + 1) == first + 1);
        CHECK(c->GetUint(MP_ID::AES67_INPUT_CHANNEL, 0) == 0);
    }

    SUBCASE("all inputs with one stream")
    {
        for (uint channel = 0; channel < 8; channel++)
        {
            CHECK(route(eight, channel, destination + channel) == first + channel);
        }
        vector<Aes67PlanStream> plan = Aes67Patch::Plan(c);
        REQUIRE(plan.size() == 1);
        CHECK(plan[0].ok);

        // no input left for another stream
        CHECK(route(stereo, 0, destination + 8) == 0);
        CHECK(c->GetUint(MP_ID::ROUTING_FPGA, destination + 8) == 0);
    }

    SUBCASE("the last input stays free as long as a stream needs a parking place")
    {
        // channels 1..7 of the eight channel stream: channel 8 needs the parking place
        for (uint channel = 0; channel < 7; channel++)
        {
            CHECK(route(eight, channel, destination + channel) == first + channel);
        }
        CHECK(route(stereo, 0, destination + 8) == 0);
        CHECK(route(stereo, 1, destination + 8) == 0);
        // the patch of the refused input is untouched
        CHECK(c->GetString(MP_ID::AES67_INPUT_STREAM, 7).length() == 0);

        // the missing channel of the stream itself fits
        CHECK(route(eight, 7, destination + 8) == first + 7);
    }

    SUBCASE("without parking place a channel is hidden behind a later one of its stream")
    {
        // six channels of the eight channel stream (not the first two) and the stereo stream
        for (uint channel = 2; channel < 8; channel++)
        {
            CHECK(route(eight, channel, destination + channel) != 0);
        }
        CHECK(route(stereo, 0, destination + 10) != 0);
        CHECK(route(stereo, 1, destination + 11) != 0);

        vector<Aes67PlanStream> plan = Aes67Patch::Plan(c);
        REQUIRE(plan.size() == 2);
        for (auto const& entry : plan)
        {
            CHECK(entry.ok);
            if (entry.stream.SameFlow(eight))
            {
                CHECK(entry.input[0] == entry.input[2]);
                CHECK(entry.input[1] == entry.input[2]);
            }
        }
    }

    SUBCASE("transmit streams share the AES67 outputs in a row")
    {
        const uint out = FPGA_OUTPUT_IDX_AES67 - 1;

        // as long as the core has not told its transmit streams there are none
        CHECK(Aes67Patch::TxStreams(c) == 0);
        CHECK(Aes67Patch::SetTxChannels(c, 0, 2) == 0);

        c->Set(MP_ID::AES67_TX_STREAMS, 4.0f);
        CHECK(Aes67Patch::TxChannels(c, 0) == 0);
        CHECK(Aes67Patch::SetTxChannels(c, 0, 2) == 2);
        CHECK(Aes67Patch::SetTxChannels(c, 1, 2) == 2);
        CHECK(Aes67Patch::TxDestination(c, 0) == out);
        CHECK(Aes67Patch::TxDestination(c, 1) == out + 2);
        CHECK(Aes67Patch::TxChannelsMax(c, 1) == 6);

        c->Set(MP_ID::ROUTING_FPGA, 10.0f, out + 0);
        c->Set(MP_ID::ROUTING_FPGA, 11.0f, out + 1);
        c->Set(MP_ID::ROUTING_FPGA, 20.0f, out + 2);
        c->Set(MP_ID::ROUTING_FPGA, 21.0f, out + 3);

        // the first stream grows: the second one moves and keeps its sources, the new channels have none
        CHECK(Aes67Patch::SetTxChannels(c, 0, 4) == 4);
        CHECK(Aes67Patch::TxDestination(c, 1) == out + 4);
        CHECK(c->GetUint(MP_ID::ROUTING_FPGA, out + 0) == 10);
        CHECK(c->GetUint(MP_ID::ROUTING_FPGA, out + 1) == 11);
        CHECK(c->GetUint(MP_ID::ROUTING_FPGA, out + 2) == 0);
        CHECK(c->GetUint(MP_ID::ROUTING_FPGA, out + 3) == 0);
        CHECK(c->GetUint(MP_ID::ROUTING_FPGA, out + 4) == 20);
        CHECK(c->GetUint(MP_ID::ROUTING_FPGA, out + 5) == 21);

        // not more than there are outputs
        CHECK(Aes67Patch::SetTxChannels(c, 2, 8) == 2);
        CHECK(Aes67Patch::TxChannelsMax(c, 3) == 0);
        CHECK(Aes67Patch::SetTxChannels(c, 3, 1) == 0);

        // the first stream is switched off: the others move up, the freed outputs are silent
        c->Set(MP_ID::ROUTING_FPGA, 30.0f, out + 6);
        CHECK(Aes67Patch::SetTxChannels(c, 0, 0) == 0);
        CHECK(Aes67Patch::TxDestination(c, 1) == out);
        CHECK(Aes67Patch::TxDestination(c, 2) == out + 2);
        CHECK(c->GetUint(MP_ID::ROUTING_FPGA, out + 0) == 20);
        CHECK(c->GetUint(MP_ID::ROUTING_FPGA, out + 1) == 21);
        CHECK(c->GetUint(MP_ID::ROUTING_FPGA, out + 2) == 30);
        CHECK(c->GetUint(MP_ID::ROUTING_FPGA, out + 3) == 0);
        for (uint output = 4; output < Aes67Patch::Outputs(c); output++)
        {
            CHECK(c->GetUint(MP_ID::ROUTING_FPGA, out + output) == 0);
        }
    }

    SUBCASE("a scene with more transmit streams than the core has")
    {
        c->Set(MP_ID::AES67_TX_STREAMS, 2.0f);
        c->Set(MP_ID::AES67_TX_CHANNELS, 2.0f, 0);
        c->Set(MP_ID::AES67_TX_CHANNELS, 2.0f, 1);
        c->Set(MP_ID::AES67_TX_CHANNELS, 2.0f, 2);
        CHECK(Aes67Patch::TxChannels(c, 1) == 2);
        CHECK(Aes67Patch::TxChannels(c, 2) == 0);
        CHECK(Aes67Patch::TxChannelsMax(c, 0) == 6);
        CHECK(Aes67Patch::SetTxChannels(c, 2, 2) == 0);
    }

    SUBCASE("a scene with more transmit channels than outputs is cut")
    {
        c->Set(MP_ID::AES67_TX_STREAMS, 4.0f);
        c->Set(MP_ID::AES67_TX_CHANNELS, 6.0f, 0);
        c->Set(MP_ID::AES67_TX_CHANNELS, 6.0f, 1);
        c->Set(MP_ID::AES67_TX_CHANNELS, 2.0f, 2);
        CHECK(Aes67Patch::TxChannels(c, 0) == 6);
        CHECK(Aes67Patch::TxChannels(c, 1) == 2);
        CHECK(Aes67Patch::TxChannels(c, 2) == 0);
        CHECK(Aes67Patch::TxDestination(c, 1) == FPGA_OUTPUT_IDX_AES67 - 1 + 6);
    }

    SUBCASE("without the configuration of the core there is nothing to patch")
    {
        c->Set(MP_ID::AES67_RX_INPUTS, 0.0f);
        c->Set(MP_ID::AES67_RX_STREAMS, 0.0f);
        CHECK(!Aes67Patch::IsAes67Input(c, first));
        CHECK(route(stereo, 0, destination) == 0);
        CHECK(Aes67Patch::Plan(c).size() == 0);
    }

    SUBCASE("a core with fewer channels than a stream has cannot receive it")
    {
        c->Set(MP_ID::AES67_RX_INPUTS, 4.0f);
        CHECK(route(eight, 0, destination) == 0);
        CHECK(route(stereo, 1, destination) == first);
        CHECK(!Aes67Patch::IsAes67Input(c, first + 4));
    }

    SUBCASE("the receive delay follows the buffer of the core")
    {
        CHECK(Aes67Patch::RxDelayMax(c, 48) == 80);
        c->Set(MP_ID::AES67_RX_BUFFER, 64.0f);
        CHECK(Aes67Patch::RxDelayMax(c, 48) == 0);
    }

    SUBCASE("transmit addresses")
    {
        CHECK(Aes67Patch::TxAutoAddress("192.168.178.134", 0) == String("239.69.134.1"));
        CHECK(Aes67Patch::TxAutoAddress("192.168.178.134", 3) == String("239.69.134.4"));
        CHECK(Aes67Patch::TxAutoAddress("", 0) == String(""));
        CHECK(Aes67Patch::TxAutoAddress("0.0.0.0", 0) == String(""));
        CHECK(Aes67Patch::IsMulticastAddress("239.69.1.3"));
        CHECK(!Aes67Patch::IsMulticastAddress("192.168.1.3"));
        CHECK(!Aes67Patch::IsMulticastAddress(""));
    }

    SUBCASE("unrouting frees the input")
    {
        CHECK(route(stereo, 0, destination) == first);
        c->Set(MP_ID::ROUTING_FPGA, (float)FPGA_INPUT_IDX_XLR, destination);
        CHECK(Aes67Patch::Plan(c).size() == 0);
        // inputs that never had a stream are used first, then the freed one
        for (uint input = 1; input < Aes67Patch::Inputs(c); input++)
        {
            CHECK(route(eight, input, destination + input) == first + input);
        }
        CHECK(route(eight, 0, destination + 20) == first);
    }
}

}
