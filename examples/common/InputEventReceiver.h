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

    // --- NEW MEMBERS FOR CAMERA CONTROL ---
    bool bIsRightMouseDown;
    position2di MouseDragPos;
    position2di LastMouseDragPos;


public:
    InputEventListener() :
        guienv(0), // Initialize the GUI pointer to null
        mouseClicked(false),
        bIsRightMouseDown(false) // Initialize new member
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

        // --- Handle Mouse Events for the Game ---
        if (event.EventType == EET_MOUSE_INPUT_EVENT) {

            // Update drag position on any mouse event
            MouseDragPos = position2di(event.MouseInput.X, event.MouseInput.Y);

            switch (event.MouseInput.Event) {
                // Left click for player movement
            case EMIE_LMOUSE_PRESSED_DOWN:
                mouseClicked = true;
                mousePos = MouseDragPos;
                return true;

                // Right click for camera rotation
            case EMIE_RMOUSE_PRESSED_DOWN:
                bIsRightMouseDown = true;
                LastMouseDragPos = MouseDragPos; // Start drag
                return true;

            case EMIE_RMOUSE_LEFT_UP:
                bIsRightMouseDown = false;
                return true;

            case EMIE_MOUSE_MOVED:
                if (bIsRightMouseDown) {
                    // We are dragging, consume the event
                    return true;
                }
                break; // Otherwise, let it fall through

            default:
                break; // Other mouse events
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

    // --- NEW HELPER FUNCTIONS FOR CAMERA ---

    /**
     * @brief Is the right mouse button currently held down?
     */
    bool IsRightMouseDown() const {
        return bIsRightMouseDown;
    }

    /**
     * @brief Get the mouse drag delta since the last call and reset it.
     */
    position2di getMouseDragDelta() {
        if (!bIsRightMouseDown) {
            return position2di(0, 0);
        }

        position2di delta = MouseDragPos - LastMouseDragPos;
        LastMouseDragPos = MouseDragPos; // Update last position
        return delta;
    }
};