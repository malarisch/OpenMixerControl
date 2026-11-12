#pragma once

#include <stdio.h>
#include <vector>

#include "defines.h"
#include "config.h"

using namespace std;

// AES67 patch: which stream channel plays on which AES67 input of the FPGA.
//
// The AES67 core delivers a number of receive channels (Inputs()), which are ordinary
// sources of the FPGA routing matrix (FPGA_INPUT_IDX_AES67 ...). A stream of the
// network has no fixed place in that matrix, so there is one level in between:
//
//   ROUTING_FPGA[destination]  = AES67 input          (as for any other source)
//   AES67_INPUT_STREAM[input]  = stream descriptor    (the patch, stored in the scene)
//   AES67_INPUT_CHANNEL[input] = channel of that stream
//
// The routing page (client) writes these Mixerparameters, the server (Aes67)
// subscribes exactly the streams that are patched to an input AND routed to at
// least one destination. Everything here works on Config only, so that both
// sides come to the same result.

namespace OMC
{

// A stream as it is identified in the patch: where it is sent to and how it is packed.
struct Aes67Stream
{
    String address;     // destination (multicast) address
    uint port = 0;      // destination UDP port
    uint channels = 0;  // channels in the stream
    uint samples = 0;   // samples per channel in one packet

    bool Valid() const
    {
        return (address.length() > 0) && (port > 0) && (port < 65536) && (samples > 0) && (samples < 256) &&
               (channels > 0) && (channels <= AES67_INPUTS_MAX);
    }

    // Two descriptors mean the same stream if they go to the same address and port.
    bool SameFlow(const Aes67Stream& other) const
    {
        return (port == other.port) && (address == other.address);
    }

    // "239.1.0.9:5004/2/48" = address:port/channels/samples per packet
    String Descriptor() const
    {
        return address + String(":") + String(port) + String("/") + String(channels) + String("/") + String(samples);
    }

    static Aes67Stream Parse(const String& descriptor)
    {
        Aes67Stream stream;
        char ip[16] = {0};
        uint port = 0;
        uint channels = 0;
        uint samples = 0;

        if (sscanf(descriptor.c_str(), "%15[0-9.]:%u/%u/%u", ip, &port, &channels, &samples) == 4)
        {
            stream.address = String(ip);
            stream.port = port;
            stream.channels = channels;
            stream.samples = samples;
        }

        return stream;
    }
};

// One stream the core has to receive, with the place of each of its channels.
struct Aes67PlanStream
{
    Aes67Stream stream;
    String name;
    bool wanted[AES67_INPUTS_MAX] = {false};   // channel is patched and routed
    uint8_t input[AES67_INPUTS_MAX] = {0};     // AES67 input (zero based) each channel is written to
    bool ok = true;                                     // false: no place left for the channels nobody wants
};

class Aes67Patch
{
    public:

        // ---------------------------------------------------------------------------
        // Configuration of the loaded core. The server (Aes67) reads it from the core and
        // publishes it as Mixerparameters; everything is 0 as long as it is not known
        // (no connection to the daemon). The *_MAX values only limit the tables here.

        // Receive channels of the core = AES67 inputs of the routing matrix.
        // A stream cannot have more channels than that: every channel is written to an input.
        static uint Inputs(Config* config)
        {
            return Limited(config->GetUint(MP_ID::AES67_RX_INPUTS), AES67_INPUTS_MAX);
        }

        // Receive streams the core handles at the same time.
        static uint RxStreams(Config* config)
        {
            return Limited(config->GetUint(MP_ID::AES67_RX_STREAMS), AES67_RX_SLOTS_MAX);
        }

        // Transmit channels of the core = AES67 outputs of the routing matrix.
        static uint Outputs(Config* config)
        {
            return Limited(config->GetUint(MP_ID::AES67_TX_OUTPUTS), AES67_OUTPUTS_MAX);
        }

        // Transmit streams the core handles at the same time. Streams beyond that do not
        // exist: they have no channels.
        static uint TxStreams(Config* config)
        {
            return Limited(config->GetUint(MP_ID::AES67_TX_STREAMS), AES67_TX_SLOTS_MAX);
        }

        // Longest receive delay (samples on top of the packet) for a stream with the given
        // packet size: the core must not get more than half of its buffer ahead of the playout.
        static uint RxDelayMax(Config* config, uint samplesPerPacket)
        {
            uint half = config->GetUint(MP_ID::AES67_RX_BUFFER) / 2;
            return (half > samplesPerPacket) ? (half - samplesPerPacket) : 0;
        }

        static bool IsAes67Input(Config* config, uint fpgaInput)
        {
            return (fpgaInput >= FPGA_INPUT_IDX_AES67) && (fpgaInput < FPGA_INPUT_IDX_AES67 + Inputs(config));
        }

        static Aes67Stream InputStream(Config* config, uint input)
        {
            return Aes67Stream::Parse(config->GetString(MP_ID::AES67_INPUT_STREAM, input));
        }

        // Is any destination of the routing matrix routed to this AES67 input?
        static bool IsRouted(Config* config, uint input)
        {
            bool routed[AES67_INPUTS_MAX];
            GetRoutedInputs(config, routed, -1, 0);
            return (input < Inputs(config)) && routed[input];
        }

        // Name of an AES67 input for the tight labels of the routing pages: "Stagebox 3"
        static String InputLabel(Config* config, uint input)
        {
            Aes67Stream stream = InputStream(config, input);
            if (!stream.Valid())
            {
                return String("AES67 In ") + String(input + 1);
            }

            return ShortName(config->GetString(MP_ID::AES67_INPUT_NAME, input), stream, 9) + String(" ") +
                   String(config->GetUint(MP_ID::AES67_INPUT_CHANNEL, input) + 1);
        }

        // Source of a destination of the FPGA routing matrix as text. Use this instead of
        // GetFormatedValue() of ROUTING_FPGA to get the stream names of the AES67 inputs.
        static String RoutingLabel(Config* config, uint destination)
        {
            uint source = config->GetUint(MP_ID::ROUTING_FPGA, destination);
            if (IsAes67Input(config, source))
            {
                return InputLabel(config, source - FPGA_INPUT_IDX_AES67);
            }

            return config->GetParameter(MP_ID::ROUTING_FPGA)->GetFormatedValue(destination);
        }

        // Session name cut to the given length; streams without a name show their address.
        static String ShortName(const String& name, const Aes67Stream& stream, uint maxLength)
        {
            String result = name;
            if (result.length() == 0)
            {
                result = stream.address;
            }
            if (result.length() > maxLength)
            {
                result = result.substring(0, maxLength);
                result.trim();
            }

            return result;
        }

        // The streams to receive and where their channels go.
        //
        // The core writes every channel of a received stream to some AES67 input; it cannot
        // skip one. Channels nobody asked for are therefore parked on an input that no
        // destination is routed to. If there is none, they are parked on the input of a later channel of the same
        // stream (the core writes the channels of a sample in ascending order, so the later
        // one wins). A stream for which neither is possible is marked as not ok.
        //
        // overrideDestination/overrideSource: plan as if this destination was routed to that source.
        static vector<Aes67PlanStream> Plan(Config* config, int overrideDestination = -1, uint overrideSource = 0)
        {
            vector<Aes67PlanStream> plan;
            bool routed[AES67_INPUTS_MAX];
            bool claimed[AES67_INPUTS_MAX] = {false};

            GetRoutedInputs(config, routed, overrideDestination, overrideSource);

            for (uint input = 0; input < Inputs(config); input++)
            {
                if (!routed[input])
                {
                    continue;
                }

                Aes67Stream stream = InputStream(config, input);
                uint channel = config->GetUint(MP_ID::AES67_INPUT_CHANNEL, input);
                if (!stream.Valid() || (channel >= stream.channels) || (stream.channels > Inputs(config)))
                {
                    continue;
                }

                Aes67PlanStream* entry = nullptr;
                for (auto& candidate : plan)
                {
                    if (candidate.stream.SameFlow(stream))
                    {
                        entry = &candidate;
                        break;
                    }
                }
                if (entry == nullptr)
                {
                    Aes67PlanStream newEntry;
                    newEntry.stream = stream;
                    newEntry.name = config->GetString(MP_ID::AES67_INPUT_NAME, input);
                    plan.push_back(newEntry);
                    entry = &plan.back();
                }

                // a stream channel can only be written to one input: the first input wins
                if ((channel < entry->stream.channels) && !entry->wanted[channel])
                {
                    entry->wanted[channel] = true;
                    entry->input[channel] = input;
                    claimed[input] = true;
                }
            }

            // parking place for unwanted channels: the last input no destination listens to
            // (a destination may be routed to an input without stream: it must stay silent)
            int parking = -1;
            for (int input = Inputs(config) - 1; input >= 0; input--)
            {
                if (!claimed[input] && !routed[input])
                {
                    parking = input;
                    break;
                }
            }

            for (auto& entry : plan)
            {
                int later = -1; // input of the next wanted channel after the current one
                for (int channel = entry.stream.channels - 1; channel >= 0; channel--)
                {
                    if (entry.wanted[channel])
                    {
                        later = entry.input[channel];
                    }
                    else if (parking >= 0)
                    {
                        entry.input[channel] = parking;
                    }
                    else if (later >= 0)
                    {
                        entry.input[channel] = later;
                    }
                    else
                    {
                        entry.ok = false;
                    }
                }
            }

            return plan;
        }

        // Patch a channel of a stream to an AES67 input, for the use by one destination of
        // the routing matrix. An input that already plays this channel is shared, otherwise
        // an input is taken that no other destination is routed to.
        //
        // Returns the source to write to ROUTING_FPGA[destination], or 0 if no input is free.
        static uint PatchChannel(Config* config, const Aes67Stream& stream, const String& name, uint channel, uint destination)
        {
            if (!stream.Valid() || (channel >= stream.channels) || (stream.channels > Inputs(config)))
            {
                return 0;
            }

            // already patched?
            for (uint input = 0; input < Inputs(config); input++)
            {
                if (InputStream(config, input).SameFlow(stream) && (config->GetUint(MP_ID::AES67_INPUT_CHANNEL, input) == channel))
                {
                    return CheckPlan(config, stream, destination, input) ? (FPGA_INPUT_IDX_AES67 + input) : 0;
                }
            }

            // inputs other destinations are routed to are taken
            bool taken[AES67_INPUTS_MAX];
            GetRoutedInputs(config, taken, destination, 0);

            // preferably keep the input this destination uses at the moment, then an unpatched one
            int candidates[AES67_INPUTS_MAX + 1];
            uint candidateCount = 0;
            uint current = config->GetUint(MP_ID::ROUTING_FPGA, destination);
            if (IsAes67Input(config, current) && !taken[current - FPGA_INPUT_IDX_AES67])
            {
                candidates[candidateCount++] = current - FPGA_INPUT_IDX_AES67;
            }
            for (uint pass = 0; pass < 2; pass++)
            {
                for (uint input = 0; input < Inputs(config); input++)
                {
                    bool patched = InputStream(config, input).Valid();
                    if (!taken[input] && (patched == (pass == 1)) && ((candidateCount == 0) || (candidates[0] != (int)input)))
                    {
                        candidates[candidateCount++] = input;
                    }
                }
            }

            for (uint i = 0; i < candidateCount; i++)
            {
                uint input = candidates[i];

                String oldStream = config->GetString(MP_ID::AES67_INPUT_STREAM, input);
                String oldName = config->GetString(MP_ID::AES67_INPUT_NAME, input);
                uint oldChannel = config->GetUint(MP_ID::AES67_INPUT_CHANNEL, input);

                config->Set(MP_ID::AES67_INPUT_STREAM, stream.Descriptor(), input);
                config->Set(MP_ID::AES67_INPUT_NAME, name, input);
                config->Set(MP_ID::AES67_INPUT_CHANNEL, (float)channel, input);

                if (CheckPlan(config, stream, destination, input))
                {
                    return FPGA_INPUT_IDX_AES67 + input;
                }

                config->Set(MP_ID::AES67_INPUT_STREAM, oldStream, input);
                config->Set(MP_ID::AES67_INPUT_NAME, oldName, input);
                config->Set(MP_ID::AES67_INPUT_CHANNEL, (float)oldChannel, input);
            }

            return 0;
        }

        // ---------------------------------------------------------------------------
        // Transmit streams
        //
        // The core sends a number of channels (Outputs()), which are ordinary destinations of the FPGA
        // routing matrix (FPGA_OUTPUT_IDX_AES67 ...). The transmit streams share them in the
        // order of their numbers: stream 1 takes the first AES67_TX_CHANNELS[0] outputs,
        // stream 2 the following ones and so on. What does not fit any more is not sent.

        // First AES67 output (zero based) of a transmit stream.
        static uint TxFirstOutput(Config* config, uint stream)
        {
            uint first = 0;
            uint streams = TxStreams(config);
            for (uint i = 0; (i < stream) && (i < streams); i++)
            {
                first += config->GetUint(MP_ID::AES67_TX_CHANNELS, i);
            }
            return (first < Outputs(config)) ? first : Outputs(config);
        }

        // Channels of a transmit stream that really fit on the AES67 outputs.
        static uint TxChannels(Config* config, uint stream)
        {
            if (stream >= TxStreams(config))
            {
                return 0;
            }
            uint channels = config->GetUint(MP_ID::AES67_TX_CHANNELS, stream);
            uint left = Outputs(config) - TxFirstOutput(config, stream);
            return (channels < left) ? channels : left;
        }

        // Destination of the FPGA routing matrix (index of ROUTING_FPGA) of the first channel of a transmit stream.
        static uint TxDestination(Config* config, uint stream)
        {
            return (FPGA_OUTPUT_IDX_AES67 - 1) + TxFirstOutput(config, stream);
        }

        // Channels a transmit stream can have at most: what the other streams leave.
        static uint TxChannelsMax(Config* config, uint stream)
        {
            uint others = 0;
            for (uint i = 0; i < TxStreams(config); i++)
            {
                if (i != stream)
                {
                    others += TxChannels(config, i);
                }
            }
            return (others < Outputs(config)) ? (Outputs(config) - others) : 0;
        }

        // Set the number of channels of a transmit stream. The following streams move on the
        // AES67 outputs; their sources move with them. New channels start without a source.
        //
        // Returns the number of channels the stream has now.
        static uint SetTxChannels(Config* config, uint stream, uint channels)
        {
            if (stream >= TxStreams(config))
            {
                return 0;
            }
            uint max = TxChannelsMax(config, stream);
            if (channels > max)
            {
                channels = max;
            }

            // sources of the stream channels as they are now
            uint counts[AES67_TX_SLOTS_MAX];
            uint sources[AES67_TX_SLOTS_MAX][AES67_OUTPUTS_MAX];
            for (uint i = 0; i < TxStreams(config); i++)
            {
                counts[i] = TxChannels(config, i);
                for (uint channel = 0; channel < Outputs(config); channel++)
                {
                    sources[i][channel] = (channel < counts[i]) ? config->GetUint(MP_ID::ROUTING_FPGA, TxDestination(config, i) + channel) : 0;
                }
            }

            counts[stream] = channels;

            uint output = 0;
            for (uint i = 0; i < TxStreams(config); i++)
            {
                if (config->GetUint(MP_ID::AES67_TX_CHANNELS, i) != counts[i])
                {
                    config->Set(MP_ID::AES67_TX_CHANNELS, (float)counts[i], i);
                }
                for (uint channel = 0; channel < counts[i]; channel++)
                {
                    SetRouting(config, (FPGA_OUTPUT_IDX_AES67 - 1) + output, sources[i][channel]);
                    output++;
                }
            }
            // outputs without stream are sent nowhere
            for (; output < Outputs(config); output++)
            {
                SetRouting(config, (FPGA_OUTPUT_IDX_AES67 - 1) + output, 0);
            }

            return channels;
        }

        // Address a transmit stream is sent to if none is set: 239.69.<last number of the
        // own address>.<stream number>, so that two consoles in a network do not collide.
        static String TxAutoAddress(const String& ownAddress, uint stream)
        {
            uint a = 0, b = 0, c = 0, d = 0;
            if ((sscanf(ownAddress.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) != 4) || (a == 0) || (a > 255) || (b > 255) || (c > 255) || (d > 255))
            {
                return String("");
            }
            return String("239.69.") + String(d) + String(".") + String(stream + 1);
        }

        static bool IsMulticastAddress(const String& address)
        {
            uint a = 0, b = 0, c = 0, d = 0;
            return (sscanf(address.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) && (a >= 224) && (a <= 239) && (b <= 255) && (c <= 255) && (d <= 255);
        }

    private:

        static uint Limited(uint value, uint max)
        {
            return (value < max) ? value : max;
        }

        static void SetRouting(Config* config, uint destination, uint source)
        {
            if (config->GetUint(MP_ID::ROUTING_FPGA, destination) != source)
            {
                config->Set(MP_ID::ROUTING_FPGA, (float)source, destination);
            }
        }

        // Which AES67 inputs does at least one destination listen to?
        static void GetRoutedInputs(Config* config, bool routed[AES67_INPUTS_MAX], int overrideDestination, uint overrideSource)
        {
            for (uint input = 0; input < AES67_INPUTS_MAX; input++)
            {
                routed[input] = false;
            }

            uint destinations = config->GetParameter(MP_ID::ROUTING_FPGA)->GetInstances();
            for (uint destination = 0; destination < destinations; destination++)
            {
                uint source = ((int)destination == overrideDestination) ? overrideSource : config->GetUint(MP_ID::ROUTING_FPGA, destination);
                if (IsAes67Input(config, source))
                {
                    routed[source - FPGA_INPUT_IDX_AES67] = true;
                }
            }
        }

        // Can every stream still be received if the destination is routed to this input?
        static bool CheckPlan(Config* config, const Aes67Stream& stream, uint destination, uint input)
        {
            vector<Aes67PlanStream> plan = Plan(config, destination, FPGA_INPUT_IDX_AES67 + input);
            if (plan.size() > RxStreams(config))
            {
                return false;
            }
            for (auto const& entry : plan)
            {
                if (!entry.ok)
                {
                    return false;
                }
            }

            return true;
        }
};

}
