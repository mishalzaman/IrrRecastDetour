#pragma once
#include <irrlicht.h>
#include <iostream> // For debugging, if needed

// Namespaces
using namespace irr;
using namespace core;
using namespace gui; // Add GUI namespace

class InputEventListener : public IEventReceiver {
private:
    // This pointer will hold our GUI environment
    IGUIEnvironment* guienv;

    // Your existing members
    bool mouseClicked;
    position2di mousePos;
    bool KeyIsDown[KEY_KEY_CODES_COUNT];

public:
    InputEventListener() :
        guienv(0), // Initialize the GUI pointer to null
        mouseClicked(false)
    {
        for (u32 i = 0; i < KEY_KEY_CODES_COUNT; ++i)
            KeyIsDown[i] = false;
    }

    /**
     * @brief This new method lets us pass the GUI pointer from main.cpp
     */
    void setGUIEnvironment(IGUIEnvironment* env) {
        guienv = env;
    }

    virtual bool OnEvent(const SEvent& event) {

        // ================================================================
        //  STEP 1: Give the GUI first priority for all events
        // ================================================================
        if (guienv && guienv->postEventFromUser(event)) {
            // If postEventFromUser returns true, the GUI "consumed" the
            // event (e.g., a slider was clicked, a button was pressed).
            // We MUST return 'true' here to tell Irrlicht the event is
            // handled. We DO NOT set 'mouseClicked' for our game logic.
            return true;
        }

        // ================================================================
        //  STEP 2: If GUI didn't want it, process it for game logic
        // ================================================================

        // --- Handle Mouse Clicks for the Game ---
        if (event.EventType == EET_MOUSE_INPUT_EVENT) {
            if (event.MouseInput.Event == EMIE_LMOUSE_PRESSED_DOWN) {
                // The GUI didn't handle this click, so it's a "game world" click.
                mouseClicked = true;
                mousePos = position2di(event.MouseInput.X, event.MouseInput.Y);

                // We handled this event.
                return true;
            }
        }

        // --- Handle Key Presses for the Game ---
        if (event.EventType == EET_KEY_INPUT_EVENT) {
            KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;

            // We handled this event.
            return true;
        }

        // We did not handle this event
        return false;
    }

    // --- Your existing helper functions (unchanged) ---

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
};