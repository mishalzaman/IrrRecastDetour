#include "InputEventReceiver.h"
#include <iostream> 

InputEventReceiver::InputEventReceiver()
    : _wheelDelta(0.0f),
    _scaledMousePos(0, 0),
    _guienv(nullptr),
    _scaleX(1.0f),
    _scaleY(1.0f)
{
    // Initialize other pollable states here
}

void InputEventReceiver::Init(gui::IGUIEnvironment* env, core::dimension2du windowSize, core::dimension2du renderSize)
{
    _guienv = env;
    _windowSize = windowSize;
    _renderSize = renderSize;

    // Calculate the scaling ratios
    if (_windowSize.Width > 0)
        _scaleX = (f32)_renderSize.Width / _windowSize.Width;
    else
        _scaleX = 1.0f;

    if (_windowSize.Height > 0)
        _scaleY = (f32)_renderSize.Height / _windowSize.Height;
    else
        _scaleY = 1.0f;
}

bool InputEventReceiver::OnEvent(const SEvent& event)
{
    // --- Update pollable state first ---
    // (This ensures getWheelDelta() works in your Update() loop)
    if (event.EventType == EET_MOUSE_INPUT_EVENT)
    {
        if (event.MouseInput.Event == EMIE_MOUSE_WHEEL)
        {
            _wheelDelta += event.MouseInput.Wheel;
        }
    }
    // ... (update other pollable states like key presses here) ...


    // --- Handle GUI event scaling ---
    if (_guienv)
    {
        // --- MOUSE EVENTS ---
        if (event.EventType == EET_MOUSE_INPUT_EVENT)
        {
            // Create a new, modifiable event
            SEvent scaledEvent = event;

            // Scale mouse coordinates
            scaledEvent.MouseInput.X = (s32)(event.MouseInput.X * _scaleX);
            scaledEvent.MouseInput.Y = (s32)(event.MouseInput.Y * _scaleY);

            // Store scaled position for polling
            _scaledMousePos.X = scaledEvent.MouseInput.X;
            _scaledMousePos.Y = scaledEvent.MouseInput.Y;

            // Send the new, scaled event to the GUI
            _guienv->postEventFromUser(scaledEvent);

            // Return true to "consume" the original event
            // This stops Irrlicht from *also* sending the unscaled
            // event to the GUI.
            return true;
        }

        // --- KEYBOARD EVENTS ---
        if (event.EventType == EET_KEY_INPUT_EVENT)
        {
            // Keyboard events also need to be sent *manually* to the GUI
            // (so you can type in the box)
            _guienv->postEventFromUser(event);

            // Also update your internal key states here for polling
            // ...

            // Consume the event so it isn't sent twice
            return true;
        }
    }

    // For all other events, or if guienv is null
    return false;
}

// Get the delta and reset it to zero
f32 InputEventReceiver::getWheelDelta()
{
    f32 delta = _wheelDelta;
    _wheelDelta = 0.0f;
    return delta;
}