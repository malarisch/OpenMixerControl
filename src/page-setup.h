#pragma once
#include "page.h"
using namespace std;

namespace OMC
{

class PageSetup: public Page
{
    using enum MP_ID;

    lv_obj_t* aes67Text = nullptr;  // state of the AES67 core and of ptp4l

    public:
        PageSetup(PageBaseParameter* pagebasepar) : Page(pagebasepar)
        {
            nextPage = X32_PAGE::SETUP_CARD;
            tabLayer0 = objects.maintab;
            tabIndex0 = 3;
            tabLayer1 = objects.setuptab;
            tabIndex1 = 0;
        }

        void OnInit() override
        {
            // String versioninfo;
            // if (mixer->adda->HasXlrIn0()) {
            //     versioninfo += "IN0: " + mixer->adda->GetXlrIn0().substring(2, mixer->adda->GetXlrIn0().length() - 1) + "\n";
            // }
            // if (mixer->adda->HasXlrIn1()) {
            //     versioninfo += "IN1: " + mixer->adda->GetXlrIn1().substring(2, mixer->adda->GetXlrIn1().length() - 1) + "\n";
            // }
            // if (mixer->adda->HasXlrIn2()) {
            //     versioninfo += "IN2: " + mixer->adda->GetXlrIn2().substring(2, mixer->adda->GetXlrIn2().length() - 1) + "\n";
            // }
            // if (mixer->adda->HasXlrIn3()) {
            //     versioninfo += "IN3: " + mixer->adda->GetXlrIn3().substring(2, mixer->adda->GetXlrIn3().length() - 1) + "\n";
            // }
            // if (mixer->adda->HasXlrOut0()) {
            //     versioninfo += "OUT0: " + mixer->adda->GetXlrOut0().substring(2, mixer->adda->GetXlrOut0().length() - 1) + "\n";
            // }
            // if (mixer->adda->HasXlrOut1()) {
            //     versioninfo += "OUT1: " + mixer->adda->GetXlrOut1().substring(2, mixer->adda->GetXlrOut1().length() - 1) + "\n";
            // }
            // if (mixer->adda->HasExpansion()) {
            //     versioninfo += "CARD: " + mixer->adda->GetExpansion().substring(2, mixer->adda->GetExpansion().length() - 1) + "\n";
            // }
            // lv_label_set_text_fmt(objects.setup_versioninfotext, "%s", versioninfo.c_str());

            // AES67: a third column beside "IP ADDRESS". It is made here and not in the EEZ project,
            // header and text look like the ones of the columns beside it.
            if (!config->IsModelAnyWing())
            {
                lv_obj_t* parent = lv_obj_get_parent(objects.setup_ipaddresstext);

                const lv_style_selector_t style = (lv_style_selector_t)LV_PART_MAIN | (lv_style_selector_t)LV_STATE_DEFAULT;

                lv_obj_t* header = lv_label_create(parent);
                lv_obj_set_pos(header, 522, 9);
                lv_obj_set_size(header, 218, LV_SIZE_CONTENT);
                lv_obj_set_style_bg_color(header, lv_color_hex(0x264056), style);
                lv_obj_set_style_bg_opa(header, 64, style);
                lv_obj_set_style_text_align(header, LV_TEXT_ALIGN_CENTER, style);
                lv_obj_set_style_pad_top(header, 5, style);
                lv_obj_set_style_pad_bottom(header, 5, style);
                lv_label_set_text_static(header, "AES67");

                aes67Text = lv_label_create(parent);
                lv_obj_set_pos(aes67Text, 522, 48);
                lv_obj_set_size(aes67Text, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
                lv_label_set_text_static(aes67Text, "...");
            }
        }

        void OnChange(bool force_update) override
        {
            if ((aes67Text != nullptr) && (force_update || config->HasParametersChanged({AES67_ONLINE, AES67_CORE_ADDRESS,
                    AES67_PTP_STATE, AES67_PTP_LEADER, AES67_PTP_OFFSET, AES67_PTP_PATH_DELAY, AES67_PTP_FREQUENCY, AES67_PTP_FREQUENCY_VALID})))
            {
                String core = config->GetString(AES67_CORE_ADDRESS);
                String text = String("Core: ") + (config->GetBool(AES67_ONLINE) ? "online" : "offline") + "\n"
                    + "Core IP: " + ((core.length() > 0) ? core : String("-")) + "\n\n";

                String ptpState = config->GetString(AES67_PTP_STATE);
                if (ptpState.length() == 0)
                {
                    text += "PTP: not running";
                }
                else
                {
                    text += String("PTP: ") + ptpState + "\n"
                        + "Leader: " + config->GetString(AES67_PTP_LEADER) + "\n"
                        + "Offset: " + String(config->GetInt(AES67_PTP_OFFSET)) + " ns\n"
                        + "Path delay: " + String(config->GetInt(AES67_PTP_PATH_DELAY)) + " ns\n"
                        + "Freq. corr.: " + (config->GetBool(AES67_PTP_FREQUENCY_VALID) ? String(config->GetInt(AES67_PTP_FREQUENCY)) + " ppb" : String("-"));
                }
                lv_label_set_text(aes67Text, text.c_str());
            }
        }

        void OnShow() override 
        {
            config->SurfaceBind(SurfaceElementId::DISPLAY_ENCODER_2, MixerparameterAction::CHANGE_SELECTED_CHANNEL, CHANNEL_COLOR);
            config->SurfaceBind(SurfaceElementId::DISPLAY_ENCODER_BUTTON_2, MixerparameterAction::TOGGLE_SELECTED_CHANNEL, CHANNEL_COLOR_INVERTED);

            config->SurfaceBind(SurfaceElementId::DISPLAY_ENCODER_4, MixerparameterAction::CHANGE, LED_BRIGHTNESS);
            config->SurfaceBind(SurfaceElementId::DISPLAY_ENCODER_5, MixerparameterAction::CHANGE, CHANNEL_LCD_MODE);
            config->SurfaceBind(SurfaceElementId::DISPLAY_ENCODER_6, MixerparameterAction::CHANGE, LCD_CONTRAST);

        }
};

}