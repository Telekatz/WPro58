#include <stdint.h>
#include <string.h>

#include "state_settings_OSD_adjust.h"

//#include "receiver.h"
//#include "channels.h"

#include "settings.h"
#include "settings_internal.h"
#include "settings_eeprom.h"
#include "buttons.h"
//#include "timer.h"

#include "ui.h"

void StateMachine::SettingsOSDAdjustStateHandler::onEnter() {
	//EepromSettings.getCallSign(callsign);
    firstLine = EepromSettings.OSDFirstLine;
    if(firstLine < FIRST_LINE_MIN)
        firstLine = FIRST_LINE_MIN;
    if(firstLine > FIRST_LINE_MAX)
        firstLine = FIRST_LINE_MAX;

    firstCol = EepromSettings.OSDFirstCol;
    if(firstCol < FIRST_COL_MIN)
        firstCol = FIRST_COL_MIN;
    if(firstCol > FIRST_COL_MAX)
        firstCol = FIRST_COL_MAX;

    cursor = 0;

    OSD::testScreen();
}

void StateMachine::SettingsOSDAdjustStateHandler::onUpdate() {

}

void StateMachine::SettingsOSDAdjustStateHandler::onButtonChange(
    Button button,
    Buttons::PressType pressType
) {
	if (button == Button::MODE){
		if(pressType == Buttons::PressType::SHORT){
		    cursor = 1 - cursor;
			Ui::needUpdate();
        } else {
            if (EepromSettings.OSDFirstLine != firstLine){
                EepromSettings.OSDFirstLine = firstLine;
                EepromSettings.save();
            }
            if (EepromSettings.OSDFirstCol != firstCol){
                EepromSettings.OSDFirstCol = firstCol;
                EepromSettings.save();
            }
            OSD::clear();
			StateMachine::switchState(StateMachine::State::SETTINGS_OSD);
		}
	} else {

        switch(button){
            case Button::UP:
                if(cursor) {
                    if (firstCol > FIRST_COL_MIN){
                        firstCol--;
                        OSD::reinit(firstLine, firstCol);
                    }
                } else {
                    if (firstLine > FIRST_LINE_MIN){
                        firstLine--;
                        OSD::reinit(firstLine, firstCol);
                    }
                }
                break;
            case Button::DOWN:
                if(cursor) {
                    if (firstCol < FIRST_COL_MAX){
                        firstCol++;
                        OSD::reinit(firstLine, firstCol);
                    }
                } else {
                    if (firstLine < FIRST_LINE_MAX){
                        firstLine++;
                        OSD::reinit(firstLine, firstCol);
                    }
                }
                break;
        }

        Ui::needUpdate();
	}

}


void StateMachine::SettingsOSDAdjustStateHandler::onInitialDraw() {
    Ui::needUpdate(); // Lazy. :(
}

void StateMachine::SettingsOSDAdjustStateHandler::onUpdateDraw() {
	const int TEXTSIZE = 2;
    Ui::clear();



    Ui::display.setTextSize(1);
    Ui::display.setTextColor(WHITE);
	Ui::display.setCursor(5, 5);
	if (cursor) {
        Ui::display.print(("adjust horizontal"));
        Ui::display.setCursor(5, 20);
        Ui::display.print(firstCol);
	} else {
	    Ui::display.print(("adjust vertical"));
	    Ui::display.setCursor(5, 20);
	    Ui::display.print(firstLine);
	}


    Ui::display.setTextSize(1);
    Ui::display.setTextColor(WHITE);
    Ui::display.setCursor(0, Ui::display.height() - CHAR_HEIGHT - 1);
    Ui::display.print(("Long MODE to exit."));

    Ui::needDisplay();
}
