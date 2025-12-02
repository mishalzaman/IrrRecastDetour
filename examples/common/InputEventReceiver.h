#pragma once
#include <irrlicht.h>
#include <iostream>

using namespace irr;
using namespace core;
using namespace gui;

class InputEventListener : public IEventReceiver {
private:
    IGUIEnvironment* guienv;

    bool mouseClicked;
    position2di mousePos;
    bool KeyIsDown[KEY_KEY_CODES_COUNT];

    bool bIsRightMouseDown;
    position2di MouseDragPos;
    position2di LastMouseDragPos;

public:
    InputEventListener() :
        guienv(0),
        mouseClicked(false),
        bIsRightMouseDown(false)
    {
        for (u32 i = 0; i < KEY_KEY_CODES_COUNT; ++i)
            KeyIsDown[i] = false;
    }

    /**
     * @brief Set the GUI environment pointer
     */
    void setGUIEnvironment(IGUIEnvironment* env) {
        guienv = env;
    }

    virtual bool OnEvent(const SEvent& event) {
        //  STEP 1: Give the GUI environment general priority
        // ================================================================
        if (guienv && guienv->postEventFromUser(event)) {
            // GUI consumed the event
            return true;
        }

        // ================================================================
        //  STEP 2: If GUI didn't want it, process it for game logic
        // ================================================================

        // --- Handle Mouse Events for the Game ---
        if (event.EventType == EET_MOUSE_INPUT_EVENT) {

            // Update drag position on any mouse event
            MouseDragPos = position2di(event.MouseInput.X, event.MouseInput.Y);

            switch (event.MouseInput.Event) {
            case EMIE_LMOUSE_PRESSED_DOWN:
                mouseClicked = true;
                mousePos = MouseDragPos;
                return true;

            case EMIE_RMOUSE_PRESSED_DOWN:
                bIsRightMouseDown = true;
                LastMouseDragPos = MouseDragPos;
                return true;

            case EMIE_RMOUSE_LEFT_UP:
                bIsRightMouseDown = false;
                return true;

            case EMIE_MOUSE_MOVED:
                if (bIsRightMouseDown) {
                    return true;
                }
                break;

            default:
                break;
            }
        }

        // --- Handle Key Presses for the Game ---
        if (event.EventType == EET_KEY_INPUT_EVENT) {
            KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;
            return true;
        }

        return false;
    }

    // --- Existing helper functions ---

    bool wasMouseClicked() {
        bool clicked = mouseClicked;
        mouseClicked = false;
        return clicked;
    }

    position2di getMousePos() const {
        return mousePos;
    }

    virtual bool IsKeyDown(EKEY_CODE keyCode) const {
        return KeyIsDown[keyCode];
    }

    bool IsRightMouseDown() const {
        return bIsRightMouseDown;
    }

    position2di getMouseDragDelta() {
        if (!bIsRightMouseDown) {
            return position2di(0, 0);
        }

        position2di delta = MouseDragPos - LastMouseDragPos;
        LastMouseDragPos = MouseDragPos;
        return delta;
    }
};
