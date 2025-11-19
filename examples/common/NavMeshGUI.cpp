#include "NavMeshGUI.h"
#include <stdio.h>

NavMeshGUI::NavMeshGUI(IGUIEnvironment* guienv) :
    _guienv(guienv),
    _mainPanel(nullptr),
    _buildButton(nullptr),
    _nextSliderID(1000)
{
}

NavMeshGUI::~NavMeshGUI()
{
    if (_mainPanel)
        _mainPanel->drop();
}

void NavMeshGUI::Load(u32 windowWidth, u32 windowHeight)
{
    _creatPanel(windowWidth, windowHeight);

    // Starting Y position for sliders
    s32 yPos = 20;

    // Add sliders - easy to add more
    _addSlider("CellSize", L"CellSize:", 0.1f, 1.0f, 0.5f, yPos);
    // Add more sliders here:
    // _addSlider("CellHeight", L"CellHeight:", 0.1f, 2.0f, 0.8f, yPos);
    // _addSlider("AgentRadius", L"AgentRadius:", 0.1f, 5.0f, 1.0f, yPos);

    // Add some spacing before the button
    yPos += ROW_SPACING;

    // Add Build button
    _buildButton = _guienv->addButton(
        rect<s32>(MARGIN, yPos, MARGIN + 150, yPos + 30),
        _mainPanel,
        BUILD_BUTTON_ID,
        L"Build NavMesh"
    );
}

void NavMeshGUI::_creatPanel(u32 windowWidth, u32 windowHeight)
{
    const s32 panelWidth = 364;
    const s32 panelX = windowWidth - panelWidth;

    // Create the main panel background (using IGUIStaticText for styling)
    _mainPanel = _guienv->addStaticText(
        L"",
        rect<s32>(panelX, 0, windowWidth, windowHeight),
        true,  // border
        false, // wordWrap
        0,     // parent
        -1,    // id
        true   // fillBackground
    );

    // Style the panel
    _mainPanel->setBackgroundColor(SColor(200, 40, 45, 55));
    _mainPanel->setDrawBorder(true);
    _mainPanel->setOverrideColor(SColor(255, 200, 200, 200));
    _mainPanel->setNotClipped(false);
    _mainPanel->grab();
}

void NavMeshGUI::_addSlider(const stringc& name, const wchar_t* labelText,
    f32 minValue, f32 maxValue, f32 defaultValue, s32& yPos)
{
    SliderControl control;
    control.minValue = minValue;
    control.maxValue = maxValue;
    control.id = _nextSliderID++;

    s32 currentX = MARGIN;

    // Create label
    control.label = _guienv->addStaticText(
        labelText,
        rect<s32>(currentX, yPos, currentX + LABEL_WIDTH, yPos + ROW_HEIGHT),
        false,
        false,
        _mainPanel
    );
    control.label->setOverrideColor(SColor(255, 200, 200, 200));

    currentX += LABEL_WIDTH + 5;

    // Create slider
    control.slider = _guienv->addScrollBar(
        true, // horizontal
        rect<s32>(currentX, yPos, currentX + SLIDER_WIDTH, yPos + ROW_HEIGHT),
        _mainPanel,
        control.id
    );

    // Map value range to 0-1000 for better precision
    control.slider->setMin(0);
    control.slider->setMax(1000);

    // Set default position
    f32 normalizedDefault = (defaultValue - minValue) / (maxValue - minValue);
    control.slider->setPos((s32)(normalizedDefault * 1000));
    control.slider->setSmallStep(1);
    control.slider->setLargeStep(50);

    currentX += SLIDER_WIDTH + 5;

    // Create value display
    control.valueDisplay = _guienv->addStaticText(
        L"",
        rect<s32>(currentX, yPos, currentX + VALUE_WIDTH, yPos + ROW_HEIGHT),
        false,
        false,
        _mainPanel
    );
    control.valueDisplay->setOverrideColor(SColor(255, 200, 200, 200));

    // Store the control
    _sliders[name] = control;
    _idToName[control.id] = name;

    // Update the display with initial value
    _updateSliderValueDisplay(name);

    // Move Y position down for next slider
    yPos += ROW_HEIGHT + ROW_SPACING;
}

void NavMeshGUI::_updateSliderValueDisplay(const stringc& name)
{
    auto it = _sliders.find(name);
    if (it != _sliders.end())
    {
        const SliderControl& control = it->second;
        f32 value = getSliderValue(name);

        wchar_t tmp[32];
        swprintf(tmp, 32, L"%.2f", value);
        control.valueDisplay->setText(tmp);
    }
}

f32 NavMeshGUI::getSliderValue(const stringc& name) const
{
    auto it = _sliders.find(name);
    if (it != _sliders.end())
    {
        const SliderControl& control = it->second;
        f32 normalized = control.slider->getPos() / 1000.0f;
        return control.minValue + normalized * (control.maxValue - control.minValue);
    }
    return 0.0f;
}

bool NavMeshGUI::OnEvent(const SEvent& event)
{
    if (event.EventType == EET_GUI_EVENT)
    {
        if (event.GUIEvent.EventType == EGET_SCROLL_BAR_CHANGED)
        {
            s32 id = event.GUIEvent.Caller->getID();
            auto it = _idToName.find(id);
            if (it != _idToName.end())
            {
                _updateSliderValueDisplay(it->second);
                return true;
            }
        }
        else if (event.GUIEvent.EventType == EGET_BUTTON_CLICKED)
        {
            s32 id = event.GUIEvent.Caller->getID();
            if (id == BUILD_BUTTON_ID)
            {
                _onBuildButtonPressed();
                return true;
            }
        }
    }
    return false;
}

void NavMeshGUI::setBuildCallback(std::function<void()> callback)
{
    _buildCallback = callback;
}

void NavMeshGUI::_onBuildButtonPressed()
{
    if (_buildCallback)
    {
        _buildCallback();
    }
}