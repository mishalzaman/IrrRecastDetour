#include "NavMeshGUI.h"
#include <stdio.h>

NavMeshGUI::NavMeshGUI(IGUIEnvironment* guienv) :
    _guienv(guienv),
    _mainPanel(nullptr),
    _buildButton(nullptr),
    _showNavmeshCheckbox(nullptr),
    _nextSliderID(1000)
{
    irr::gui::IGUISkin* skin = _guienv->getSkin();

    if (skin) {
        skin->setColor(
            irr::gui::EGDC_BUTTON_TEXT, // Or EGDC_STATIC_TEXT, depending on the skin/context
            irr::video::SColor(255, 255, 255, 255) // White (Alpha, Red, Green, Blue)
        );
    }
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

    // 1. Rasterization
    _addSlider("CellSize", L"Cell Size:", 0.05f, 1.0f, 0.15f, yPos);
    _addSlider("CellHeight", L"Cell Height:", 0.05f, 1.0f, 0.2f, yPos);

    // 2. Agent Properties
    _addSlider("AgentHeight", L"Agent Height:", 0.5f, 2.0f, 0.8f, yPos);
    _addSlider("AgentRadius", L"Agent Radius:", 0.1f, 5.0f, 0.4f, yPos);
    _addSlider("AgentMaxClimb", L"Max Climb:", 0.1f, 2.0f, 0.6f, yPos);
    _addSlider("AgentMaxSlope", L"Max Slope:", 0.0f, 90.0f, 45.0f, yPos);

    // 3. Region / Filtering
    _addSlider("RegionMinSize", L"Min Region:", 1.0f, 100.0f, 8.0f, yPos);
    _addSlider("RegionMergeSize", L"Merge Region:", 1.0f, 100.0f, 20.0f, yPos);

    // 4. Polygonization
    _addSlider("EdgeMaxLen", L"Max Edge Len:", 0.0f, 50.0f, 12.0f, yPos);
    _addSlider("EdgeMaxError", L"Max Edge Err:", 0.1f, 5.0f, 1.3f, yPos);
    _addSlider("VertsPerPoly", L"Verts Per Poly:", 3.0f, 6.0f, 6.0f, yPos);

    // 5. Detail Mesh
    _addSlider("DetailSampleDist", L"Det Sample Dist:", 0.0f, 16.0f, 6.0f, yPos);
    _addSlider("DetailSampleMaxError", L"Det Sample Err:", 0.0f, 5.0f, 1.0f, yPos);

    // Add some spacing before controls
    yPos += ROW_SPACING;

    // Add Show Navmesh checkbox
    _showNavmeshCheckbox = _guienv->addCheckBox(
        true, // initially unchecked
        rect<s32>(MARGIN, yPos, MARGIN + 200, yPos + 25),
        _mainPanel,
        SHOW_NAVMESH_CHECKBOX_ID,
        L"Show Navmesh"
    );

    yPos += 35; // spacing after checkbox

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
    control.slider->setPos((s32)(normalizedDefault * 1000 + 0.5f));
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
        else if (event.GUIEvent.EventType == EGET_CHECKBOX_CHANGED)
        {
            s32 id = event.GUIEvent.Caller->getID();
            if (id == SHOW_NAVMESH_CHECKBOX_ID)
            {
                _onShowNavmeshToggled();
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

void NavMeshGUI::setShowNavmeshCallback(std::function<void(bool)> callback)
{
    _showNavmeshCallback = callback;
}

void NavMeshGUI::_onBuildButtonPressed()
{
    if (_buildCallback)
    {
        _buildCallback();
    }
}

void NavMeshGUI::_onShowNavmeshToggled()
{
    if (_showNavmeshCallback && _showNavmeshCheckbox)
    {
        bool isChecked = _showNavmeshCheckbox->isChecked();
        _showNavmeshCallback(isChecked);
    }
}