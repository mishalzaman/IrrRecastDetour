#pragma once

#include <irrlicht.h>
#include <map>

using namespace irr;
using namespace core;
using namespace video;
using namespace gui;

class NavMeshGUI
{
public:
    NavMeshGUI(IGUIEnvironment* guienv);
    ~NavMeshGUI();

    void Load(u32 windowWidth, u32 windowHeight);
    bool OnEvent(const SEvent& event);

    // Getter for slider values
    f32 getSliderValue(const stringc& name) const;

private:
    struct SliderControl
    {
        IGUIStaticText* label;
        IGUIScrollBar* slider;
        IGUIStaticText* valueDisplay;
        f32 minValue;
        f32 maxValue;
        s32 id;
    };

    void _creatPanel(u32 windowWidth, u32 windowHeight);
    void _addSlider(const stringc& name, const wchar_t* label,
        f32 minValue, f32 maxValue, f32 defaultValue, s32& yPos);
    void _updateSliderValueDisplay(const stringc& name);

    IGUIEnvironment* _guienv;
    IGUIStaticText* _mainPanel;

    std::map<stringc, SliderControl> _sliders;
    std::map<s32, stringc> _idToName; // Map slider ID to name

    s32 _nextSliderID;

    // Layout constants
    static const s32 MARGIN = 10;
    static const s32 LABEL_WIDTH = 80;
    static const s32 SLIDER_WIDTH = 200;
    static const s32 VALUE_WIDTH = 50;
    static const s32 ROW_HEIGHT = 20;
    static const s32 ROW_SPACING = 10;
};