#pragma once
#include <irrlicht.h>
#include <string>
#include <map>
#include <functional>

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

    f32 getSliderValue(const stringc& name) const;

    void setBuildCallback(std::function<void()> callback);
    void setShowNavmeshCallback(std::function<void(bool)> callback);

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
    void _addSlider(const stringc& name, const wchar_t* labelText,
        f32 minValue, f32 maxValue, f32 defaultValue, s32& yPos);
    void _updateSliderValueDisplay(const stringc& name);
    void _onBuildButtonPressed();
    void _onShowNavmeshToggled();

    IGUIEnvironment* _guienv;
    IGUIStaticText* _mainPanel;
    IGUIButton* _buildButton;
    IGUICheckBox* _showNavmeshCheckbox;

    std::map<stringc, SliderControl> _sliders;
    std::map<s32, stringc> _idToName;

    std::function<void()> _buildCallback;
    std::function<void(bool)> _showNavmeshCallback;

    s32 _nextSliderID;

    // Layout constants
    static const s32 MARGIN = 10;
    static const s32 LABEL_WIDTH = 130;
    static const s32 SLIDER_WIDTH = 140;
    static const s32 VALUE_WIDTH = 60;
    static const s32 ROW_HEIGHT = 20;
    static const s32 ROW_SPACING = 10;
    static const s32 BUILD_BUTTON_ID = 9999;
    static const s32 SHOW_NAVMESH_CHECKBOX_ID = 9998;
};