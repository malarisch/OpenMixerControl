#pragma once

#include "page.h"

using namespace std;

namespace OMC
{

class PageRoutingFpga: public Page
{
    using enum MP_ID;

    private:

        uint selection_target = 0;
        uint selection_index = 0;
        int selection_source = 0;
        int selection_source_item = 0;

        uint current_targetblocksize = 0;
        uint current_targetblockoffset = 0;

        // The destination roller: the fixed blocks of the FPGA, followed by the AES67 transmit streams.
        static const uint fixed_targets = 6;

        // AES67 transmit stream (zero based) that is selected as destination block, or -1
        int SelectedTxStream()
        {
            return (selection_target >= fixed_targets) ? (int)(selection_target - fixed_targets) : -1;
        }

        // Encoder 4 sets the number of channels of the selected transmit stream.
        void BindTxEncoder()
        {
            int stream = SelectedTxStream();
            if (stream < 0)
            {
                config->SurfaceUnbind(SurfaceElementId::DISPLAY_ENCODER_4);
                return;
            }

            uint channels = Aes67Patch::TxChannels(config, stream);
            String label = String("TX ") + String(stream + 1) + String(" Channels\n");
            label += (channels > 0) ? (String(channels) + String("/") + String(Aes67Patch::TxChannelsMax(config, stream))) : String("Off");
            if (channels > 0)
            {
                // where the stream is sent to, as soon as the AES67 daemon has started it
                String address = config->GetString(AES67_TX_ACTIVE_ADDRESS, stream);
                label += String("\n") + ((address.length() > 0) ? address : String(config->GetBool(AES67_ONLINE) ? "starting" : "AES67 offline"));
            }
            config->SurfaceBindCustom(SurfaceElementId::DISPLAY_ENCODER_4, label);
        }

        // The source roller: the fixed blocks of the FPGA, followed by the AES67 streams
        // (discovered on the network or patched in the scene).
        static const uint fixed_sources = 5;
        struct Aes67Source
        {
            Aes67Stream stream;
            String name;
        };
        vector<Aes67Source> aes67_sources;
        String source_options = "";
        bool aes67_full = false; // the last try to route a stream found no free AES67 input

        // AES67 input (zero based) the selected destination is routed to, or -1
        int SelectedAes67Input()
        {
            uint source = config->GetUint(ROUTING_FPGA, current_targetblockoffset + selection_index);
            if (Aes67Patch::IsAes67Input(config, source) && Aes67Patch::InputStream(config, source - FPGA_INPUT_IDX_AES67).Valid())
            {
                return source - FPGA_INPUT_IDX_AES67;
            }
            return -1;
        }

        // Route a channel of a stream to the selected destination.
        void RouteAes67(const Aes67Stream& stream, const String& name, uint channel)
        {
            uint destination = current_targetblockoffset + selection_index;
            uint source = Aes67Patch::PatchChannel(config, stream, name, channel, destination);

            aes67_full = (source == 0);
            if (!aes67_full && (config->GetUint(ROUTING_FPGA, destination) != source))
            {
                config->Set(ROUTING_FPGA, source, destination);
            }
        }

        void BindSourceEncoder()
        {
            String label = String(LV_SYMBOL_REFRESH) + String("\nSource");
            if (aes67_full)
            {
                label += String("\nAES67 full");
            }
            config->SurfaceBindCustom(SurfaceElementId::DISPLAY_ENCODER_5, label);
        }

        void LoadSources()
        {
            aes67_sources.clear();

            uint count = config->GetUint(AES67_DISCOVERED_COUNT);
            for (uint i = 0; (i < count) && (i < AES67_DISCOVERED_MAX); i++)
            {
                Aes67Source source;
                source.stream = Aes67Stream::Parse(config->GetString(AES67_DISCOVERED_STREAM, i));
                source.name = config->GetString(AES67_DISCOVERED_NAME, i);
                if (source.stream.Valid())
                {
                    aes67_sources.push_back(source);
                }
            }

            // streams in use that are not on the network (any more, or not yet) stay selectable;
            // a stream that is gone and not routed anywhere disappears from the list
            for (uint input = 0; input < Aes67Patch::Inputs(config); input++)
            {
                Aes67Source source;
                source.stream = Aes67Patch::InputStream(config, input);
                source.name = config->GetString(AES67_INPUT_NAME, input);
                if (source.stream.Valid() && Aes67Patch::IsRouted(config, input) && (FindSource(source.stream) < 0))
                {
                    aes67_sources.push_back(source);
                }
            }

            String options = "XLR\nCARD\nAUX\nDSP\nAES50 A";
            for (auto const& source : aes67_sources)
            {
                options += String("\n") + Aes67Patch::ShortName(source.name, source.stream, 14);
            }
            if (options != source_options)
            {
                source_options = options;
                lv_roller_set_options(objects.routing_fpga_source, options.c_str(), LV_ROLLER_MODE_NORMAL);
            }
        }

        int FindSource(const Aes67Stream& stream)
        {
            for (uint i = 0; i < aes67_sources.size(); i++)
            {
                if (aes67_sources[i].stream.SameFlow(stream))
                {
                    return i;
                }
            }
            return -1;
        }

    public:

        PageRoutingFpga(PageBaseParameter* pagebasepar) : Page(pagebasepar)
        {
            prevPage = X32_PAGE::ROUTING;
            nextPage = X32_PAGE::ROUTING_DSP1;
            tabLayer0 = objects.maintab;
            tabIndex0 = 2;
            tabLayer1 = objects.routingtab;
            tabIndex1 = 1;
        }

        void OnShow() override
        {
            config->SurfaceBindCustom(SurfaceElementId::DISPLAY_ENCODER_1, String(LV_SYMBOL_SHUFFLE) + String("\nSection"));
            config->SurfaceBindCustom(SurfaceElementId::DISPLAY_ENCODER_2, String(LV_SYMBOL_REFRESH) + String("\nSelect"));
            config->SurfaceBindCustom(SurfaceElementId::DISPLAY_ENCODER_3, String(LV_SYMBOL_UP) + String("\nSelect\n") + LV_SYMBOL_DOWN);
            aes67_full = false;

            LoadTargets();
            LoadMatrix();
        }

        // The destination roller offers the transmit streams the loaded AES67 core has.
        void LoadTargets()
        {
            uint streams = Aes67Patch::TxStreams(config);

            String targets = "Channels\nAUX Out\nXLR Out\nA50 Out\nCARD Out\nUltranet";
            for (uint stream = 0; stream < streams; stream++)
            {
                targets += String("\nAES67 TX") + String(stream + 1);
            }
            if (selection_target > fixed_targets + streams - 1)
            {
                selection_target = fixed_targets + streams - 1;
                selection_index = 0;
            }
            lv_roller_set_options(objects.routing_fpga_target, targets.c_str(), LV_ROLLER_MODE_NORMAL);
            lv_roller_set_selected(objects.routing_fpga_target, selection_target, LV_ANIM_OFF);
        }

        void OnChange(bool force)
        {
            if (config->HasParametersChanged({AES67_TX_STREAMS, AES67_TX_OUTPUTS, AES67_RX_STREAMS, AES67_RX_INPUTS}))
            {
                LoadTargets();
                LoadMatrix();
            }

            if (config->HasParametersChanged({ROUTING_FPGA,
                    AES67_DISCOVERED_COUNT, AES67_DISCOVERED_STREAM, AES67_DISCOVERED_NAME,
                    AES67_INPUT_STREAM, AES67_INPUT_NAME, AES67_INPUT_CHANNEL,
                    AES67_TX_CHANNELS, AES67_TX_ACTIVE_ADDRESS, AES67_ONLINE}))
            {
                LoadMatrix();
            }
        }

        void OnChangeCustomEncoder(SurfaceElementId surface_element_id, int amount) override
        {
            // a transmit stream without channels has nothing to select or to route
            if ((current_targetblocksize == 0) &&
                (surface_element_id != SurfaceElementId::DISPLAY_ENCODER_1) && (surface_element_id != SurfaceElementId::DISPLAY_ENCODER_4))
            {
                return;
            }

            switch (surface_element_id)
            {
                case SurfaceElementId::DISPLAY_ENCODER_1:
                    selection_target = helper->CheckBoundaries(selection_target, amount, 0, fixed_targets + Aes67Patch::TxStreams(config) - 1);
                    lv_roller_set_selected(objects.routing_fpga_target, selection_target, LV_ANIM_OFF);
                    LoadMatrix();
                    break;
                case SurfaceElementId::DISPLAY_ENCODER_2:
                    selection_index = helper->CheckBoundaries(selection_index, amount, 0, current_targetblocksize-1);
                    LoadMatrix();
                    break;
                case SurfaceElementId::DISPLAY_ENCODER_3:
                    if (amount < 0)
                    {
                        selection_index = helper->CheckBoundaries(selection_index, -8, 0, current_targetblocksize-1);
                    }
                    else
                    {
                        selection_index = helper->CheckBoundaries(selection_index, 8, 0, current_targetblocksize-1);
                    }
                    LoadMatrix();
                    break;
                case SurfaceElementId::DISPLAY_ENCODER_4:
                    {
                        int stream = SelectedTxStream();
                        if (stream >= 0)
                        {
                            int channels = Aes67Patch::TxChannels(config, stream);
                            int newChannels = helper->CheckBoundaries(channels, amount, 0, Aes67Patch::TxChannelsMax(config, stream));
                            if (newChannels != channels)
                            {
                                Aes67Patch::SetTxChannels(config, stream, newChannels);
                                LoadMatrix();
                            }
                        }
                    }
                    break;
                case SurfaceElementId::DISPLAY_ENCODER_5:
                    {
                        selection_source = helper->CheckBoundaries(selection_source, amount, 0, fixed_sources - 1 + aes67_sources.size());
                        lv_roller_set_selected(objects.routing_fpga_source, selection_source, LV_ANIM_OFF);

                        if (selection_source >= (int)fixed_sources)
                        {
                            // AES67 stream: start with its first channel, the next encoder selects the channel
                            Aes67Source& source = aes67_sources[selection_source - fixed_sources];
                            RouteAes67(source.stream, source.name, 0);
                            LoadMatrix();
                            break;
                        }
                        aes67_full = false;

                        uint item_index = 0;

                        switch(selection_source)
                        {
                            case 0: // XLR
                                item_index = FPGA_INPUT_IDX_XLR;
                                break;
                            case 1: // CARD
                                item_index = FPGA_INPUT_IDX_CARD;
                                break;
                            case 2: // AUX
                                item_index = FPGA_INPUT_IDX_AUX;
                                break;
                            case 3: // DSP
                                item_index = FPGA_INPUT_IDX_DSP_RETURN;
                                break;
                            case 4: // AES50 A
                                item_index = FPGA_INPUT_IDX_AES50A;
                                break;
                            default:
                                break;
                        }
                        
                        config->Set(ROUTING_FPGA, item_index, current_targetblockoffset + selection_index);
                    }
                    break;
                case SurfaceElementId::DISPLAY_ENCODER_6:
                    {
                        // only bound as custom encoder while the destination plays an AES67 stream: select the channel
                        int input = SelectedAes67Input();
                        if (input >= 0)
                        {
                            Aes67Stream stream = Aes67Patch::InputStream(config, input);
                            String name = config->GetString(AES67_INPUT_NAME, input);
                            int channel = config->GetUint(AES67_INPUT_CHANNEL, input);
                            int newChannel = helper->CheckBoundaries(channel, amount, 0, stream.channels - 1);
                            if (newChannel != channel)
                            {
                                RouteAes67(stream, name, newChannel);
                                LoadMatrix();
                            }
                        }
                    }
                    break;
                default:
                    break;
            }
        }

        void LoadMatrix()
        {
            uint targetblockoffset = 0;
            uint targetblocksize = 0;

            switch (selection_target)
            {
                case 0: // CHANNELS
                    targetblockoffset = 72;
                    targetblocksize = 40;
                    break;
                case 1: // AUX Out
                    targetblockoffset = 65;
                    targetblocksize = 6;
                    break;
                case 2: // XLR Out
                    targetblockoffset = 0;
                    targetblocksize = 16;
                    break;
                case 3: // AES50 Out
                    targetblockoffset = 112;
                    targetblocksize = 48;
                    break;
                case 4: // Card Out
                    targetblockoffset = 32;
                    targetblocksize = 32;
                    break;
                case 5: // Ultranet
                    targetblockoffset = 16;
                    targetblocksize = 16;
                    break;
                default: // AES67 transmit stream
                    targetblockoffset = Aes67Patch::TxDestination(config, SelectedTxStream());
                    targetblocksize = Aes67Patch::TxChannels(config, SelectedTxStream());
                    break;
            }

            current_targetblockoffset = targetblockoffset;
            current_targetblocksize = targetblocksize;
            if (selection_index >= targetblocksize)
            {
                selection_index = (targetblocksize > 0) ? (targetblocksize - 1) : 0;
            }

            BindTxEncoder();
            if (targetblocksize == 0)
            {
                // nothing to route: the transmit stream is off
                config->SurfaceUnbind(SurfaceElementId::DISPLAY_ENCODER_5);
                config->SurfaceUnbind(SurfaceElementId::DISPLAY_ENCODER_6);
            }

            LoadSources();

            for (uint i = 0; i < lv_obj_get_child_count(objects.routing_matrix); i++)
            {
                lv_obj_t* routing_tile = lv_obj_get_child(objects.routing_matrix, i);

                lv_obj_t* routing_tile_lbl_header = lv_obj_get_child(routing_tile, 0);
                lv_obj_t* routing_tile_lbl = lv_obj_get_child(routing_tile, 1);
                lv_obj_t* routing_tile_lbl_footer = lv_obj_get_child(routing_tile, 2);

                String header = "";
                String label = "";
                String footer = "";

                if (i < targetblocksize)
                {
                    switch (selection_target)
                    {
                        case 0: // CHANNELS
                            header = config->GetString(CHANNEL_NAME_INTERN, i);
                            label = Aes67Patch::RoutingLabel(config, i + targetblockoffset);
                            footer = config->GetString(CHANNEL_NAME, i);
                            break;
                        default:
                            header = String(i+1);
                            label = Aes67Patch::RoutingLabel(config, i + targetblockoffset);
                            break;
                    }
                }

                lv_label_set_text(routing_tile_lbl_header, header.c_str());
                lv_label_set_text(routing_tile_lbl, label.c_str());
                lv_label_set_text(routing_tile_lbl_footer, footer.c_str());

                if ((i == selection_index) && (i < targetblocksize))
                {
                    lv_obj_set_state(routing_tile_lbl_header, LV_STATE_FOCUSED, true);

                    int aes67_input = SelectedAes67Input();
                    if (aes67_input >= 0)
                    {
                        // AES67 stream: the encoder steps through the channels of the stream
                        Aes67Stream stream = Aes67Patch::InputStream(config, aes67_input);
                        config->SurfaceBindCustom(SurfaceElementId::DISPLAY_ENCODER_6,
                            Aes67Patch::ShortName(config->GetString(AES67_INPUT_NAME, aes67_input), stream, 12) +
                            String("\nChannel ") + String(config->GetUint(AES67_INPUT_CHANNEL, aes67_input) + 1) +
                            String("/") + String(stream.channels));
                    }
                    else
                    {
                        config->SurfaceBind(SurfaceElementId::DISPLAY_ENCODER_6, MixerparameterAction::CHANGE, ROUTING_FPGA, i + targetblockoffset);
                    }
                    BindSourceEncoder();

                    // Update source block
                    switch (config->GetUint(ROUTING_FPGA, i + targetblockoffset))
                    {
                        case FPGA_INPUT_IDX_XLR ... FPGA_INPUT_IDX_XLR + 31:
                            selection_source = 0;
                            break;
                        case FPGA_INPUT_IDX_CARD ... FPGA_INPUT_IDX_CARD + 31:
                            selection_source = 1;
                            break;
                        case FPGA_INPUT_IDX_AUX ... FPGA_INPUT_IDX_AUX + 7:
                            selection_source = 2;
                            break;
                        case FPGA_INPUT_IDX_DSP_RETURN ... FPGA_INPUT_IDX_DSP_RETURN + 39:
                            selection_source = 3;
                            break;
                        case FPGA_INPUT_IDX_AES50A ... FPGA_INPUT_IDX_AES67 - 1:
                            selection_source = 4;
                            break;
                        case FPGA_INPUT_IDX_AES67 ... FPGA_INPUT_IDX_AES67 + AES67_INPUTS_MAX - 1:
                            {
                                // the stream of the AES67 input; an input without stream belongs to the AES50 A block
                                int source = (aes67_input >= 0) ? FindSource(Aes67Patch::InputStream(config, aes67_input)) : -1;
                                selection_source = (source >= 0) ? (int)(fixed_sources + source) : 4;
                            }
                            break;
                        default:
                            break;
                    }                    
                    lv_roller_set_selected(objects.routing_fpga_source, selection_source, LV_ANIM_OFF);
                }
                else
                {
                    lv_obj_set_state(routing_tile_lbl_header, LV_STATE_FOCUSED, false);
                }
            }

            SyncEncoderWidgets(true);
        }
};

}