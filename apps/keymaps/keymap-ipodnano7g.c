/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

/* Button Code Definitions for the iPod nano 7G (N31) */

#include "config.h"
#include "action.h"
#include "button.h"
#include "settings.h"

/*
 * The N31 has five physical keys and no clickwheel:
 *
 *   Vol+ / Vol-   SoC GPIO 40/41
 *   Home          PMIC reg 7 bit 4
 *   Sleep         PMIC reg 7 bit 5
 *   Play          PMIC reg 8 bit 1, the side key between the volume keys
 *
 * Real navigation is the capacitive touchscreen, which is Phase 6 work. Until
 * that lands the volume keys double as up/down so the UI is usable, with
 * Play as select and Home as back. Once touch works this mapping should be
 * revisited -- volume keys doing double duty is a bring-up compromise, not
 * the intended feel of the device.
 */

/* {Action Code,    Button code,    Prereq button code } */

static const struct button_mapping button_context_standard[] = {
    {ACTION_STD_PREV,           BUTTON_VOL_UP,                      BUTTON_NONE},
    {ACTION_STD_PREVREPEAT,     BUTTON_VOL_UP|BUTTON_REPEAT,        BUTTON_NONE},
    {ACTION_STD_NEXT,           BUTTON_VOL_DOWN,                    BUTTON_NONE},
    {ACTION_STD_NEXTREPEAT,     BUTTON_VOL_DOWN|BUTTON_REPEAT,      BUTTON_NONE},
    {ACTION_STD_OK,             BUTTON_PLAY|BUTTON_REL,             BUTTON_PLAY},
    {ACTION_STD_CONTEXT,        BUTTON_PLAY|BUTTON_REPEAT,          BUTTON_PLAY},
    {ACTION_STD_CANCEL,         BUTTON_HOME|BUTTON_REL,             BUTTON_HOME},
    {ACTION_STD_MENU,           BUTTON_HOME|BUTTON_REPEAT,          BUTTON_HOME},
    LAST_ITEM_IN_LIST
}; /* button_context_standard */

static const struct button_mapping button_context_wps[] = {
    {ACTION_WPS_PLAY,           BUTTON_PLAY|BUTTON_REL,             BUTTON_PLAY},
    {ACTION_WPS_STOP,           BUTTON_PLAY|BUTTON_REPEAT,          BUTTON_NONE},
    {ACTION_WPS_VOLUP,          BUTTON_VOL_UP,                      BUTTON_NONE},
    {ACTION_WPS_VOLUP,          BUTTON_VOL_UP|BUTTON_REPEAT,        BUTTON_NONE},
    {ACTION_WPS_VOLDOWN,        BUTTON_VOL_DOWN,                    BUTTON_NONE},
    {ACTION_WPS_VOLDOWN,        BUTTON_VOL_DOWN|BUTTON_REPEAT,      BUTTON_NONE},
    {ACTION_WPS_BROWSE,         BUTTON_HOME|BUTTON_REL,             BUTTON_HOME},
    {ACTION_WPS_CONTEXT,        BUTTON_HOME|BUTTON_REPEAT,          BUTTON_HOME},
    {ACTION_STD_KEYLOCK,        BUTTON_POWER|BUTTON_REL,            BUTTON_POWER},
    LAST_ITEM_IN_LIST
}; /* button_context_wps */

static const struct button_mapping button_context_list[] = {
    LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_STD)
}; /* button_context_list */

static const struct button_mapping button_context_settings[] = {
    {ACTION_SETTINGS_INC,       BUTTON_VOL_UP,                      BUTTON_NONE},
    {ACTION_SETTINGS_INCREPEAT, BUTTON_VOL_UP|BUTTON_REPEAT,        BUTTON_NONE},
    {ACTION_SETTINGS_DEC,       BUTTON_VOL_DOWN,                    BUTTON_NONE},
    {ACTION_SETTINGS_DECREPEAT, BUTTON_VOL_DOWN|BUTTON_REPEAT,      BUTTON_NONE},
    LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_STD)
}; /* button_context_settings */

static const struct button_mapping button_context_yesno[] = {
    {ACTION_YESNO_ACCEPT,       BUTTON_PLAY,                        BUTTON_NONE},
    {ACTION_STD_CANCEL,         BUTTON_HOME,                        BUTTON_NONE},
    LAST_ITEM_IN_LIST__NEXTLIST(CONTEXT_STD)
}; /* button_context_yesno */

const struct button_mapping* target_get_context_mapping(int context)
{
    switch (context & ~CONTEXT_LOCKED)
    {
        default:
        case CONTEXT_STD:
            return button_context_standard;
        case CONTEXT_WPS:
            return button_context_wps;
        case CONTEXT_SETTINGS:
        case CONTEXT_SETTINGS_EQ:
        case CONTEXT_SETTINGS_TIME:
        case CONTEXT_SETTINGS_COLOURCHOOSER:
            return button_context_settings;
        case CONTEXT_TREE:
        case CONTEXT_CUSTOM|CONTEXT_TREE:
        case CONTEXT_MAINMENU:
        case CONTEXT_BOOKMARKSCREEN:
        case CONTEXT_LIST:
            return button_context_list;
        case CONTEXT_YESNOSCREEN:
            return button_context_yesno;
    }
}
