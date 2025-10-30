#pragma once
#include <irrlicht.h>

using namespace irr;

class InputEventReceiver : public IEventReceiver
{
public:
    // Constructor
    InputEventReceiver();

    // Call this *after* device creation
    void Init(gui::IGUIEnvironment* env, core::dimension2du windowSize, core::dimension2du renderSize);

    virtual bool OnEvent(const SEvent& event) override;

    // --- Your poller functions ---
    f32 getWheelDelta(); // Modified this slightly

    // --- New poller for scaled mouse ---
    const core::position2di& getScaledMousePosition() const { return _scaledMousePos; }

    // (Add IsKeyDown, etc. if you need them)

private:
    // Internal state for polling
    f32 _wheelDelta;
    core::position2di _scaledMousePos;
    // (key states, etc.)

    // GUI scaling helpers
    gui::IGUIEnvironment* _guienv;
    core::dimension2du _windowSize;
    core::dimension2du _renderSize;
    f32 _scaleX;
    f32 _scaleY;
};