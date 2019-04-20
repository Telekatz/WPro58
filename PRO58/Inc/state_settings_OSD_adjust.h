#ifndef STATE_SETTINGS_OSD_ADJUST_H
#define STATE_SETTINGS_OSD_ADJUST_H


#include "state.h"
#include "settings.h"


namespace StateMachine {
    class SettingsOSDAdjustStateHandler : public StateMachine::StateHandler {
        private:
    			int firstLine;
    			int firstCol;
    			int cursor;
        public:
            void onEnter();
            void onUpdate();

            void onInitialDraw();
            void onUpdateDraw();

            void onButtonChange(Button button, Buttons::PressType pressType);
    };
}


#endif
