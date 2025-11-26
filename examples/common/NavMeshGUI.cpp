#include "NavMeshGUI.h"
#include <stdio.h>

NavMeshGUI::NavMeshGUI(IGUIEnvironment* guienv) :
    _guienv(guienv),
    _mainPanel(nullptr),
    _buildButton(nullptr),
    _showNavmeshCheckbox(nullptr),
    _nextSliderID(1000),
    // Initialize layout vars to defaults (will be overwritten in Load)
    _margin(BASE_MARGIN),
    _labelWidth(BASE_LABEL_WIDTH),
    _sliderWidth(BASE_SLIDER_WIDTH),
    _valueWidth(BASE_VALUE_WIDTH),
    _rowHeight(BASE_ROW_HEIGHT),
    _rowSpacing(BASE_ROW_SPACING),
    _scaleFactor(1.0f)
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
    _scaleFactor = (f32)windowHeight / 720.0f;

    if (_scaleFactor < 0.5f) _scaleFactor = 0.5f;

    _margin = (s32)(BASE_MARGIN * _scaleFactor);
    _labelWidth = (s32)(BASE_LABEL_WIDTH * _scaleFactor);
    _sliderWidth = (s32)(BASE_SLIDER_WIDTH * _scaleFactor);
    _valueWidth = (s32)(BASE_VALUE_WIDTH * _scaleFactor);
    _rowHeight = (s32)(BASE_ROW_HEIGHT * _scaleFactor);
    _rowSpacing = (s32)(BASE_ROW_SPACING * _scaleFactor);

    s32 padding = (s32)(5 * _scaleFactor);
    s32 extraPadding = (s32)(14 * _scaleFactor);
    _panelWidth = _margin * 2 + _labelWidth + padding + _sliderWidth + padding + _valueWidth + extraPadding;

    _creatPanel(windowWidth, windowHeight);

    // Starting Y position for sliders
    s32 yPos = (s32)(20 * _scaleFactor);

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
    yPos += _rowSpacing;

    // Add Show Navmesh checkbox
    s32 checkboxHeight = (s32)(25 * _scaleFactor);
    s32 checkboxWidth = (s32)(200 * _scaleFactor);
    _showNavmeshCheckbox = _guienv->addCheckBox(
        true, // initially unchecked
        rect<s32>(_margin, yPos, _margin + checkboxWidth, yPos + checkboxHeight),
        _mainPanel,
        SHOW_NAVMESH_CHECKBOX_ID,
        L"Show Navmesh"
    );

    yPos += (s32)(35 * _scaleFactor); // spacing after checkbox

    // Add Build button
    s32 btnHeight = (s32)(30 * _scaleFactor);
    s32 btnWidth = (s32)(150 * _scaleFactor);
    _buildButton = _guienv->addButton(
        rect<s32>(_margin, yPos, _margin + btnWidth, yPos + btnHeight),
        _mainPanel,
        BUILD_BUTTON_ID,
        L"Build NavMesh"
    );
}

void NavMeshGUI::_creatPanel(u32 windowWidth, u32 windowHeight)
{
    const s32 panelX = windowWidth - _panelWidth;

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

    s32 currentX = _margin;
    s32 padding = (s32)(5 * _scaleFactor);

    // Create label
    control.label = _guienv->addStaticText(
        labelText,
        rect<s32>(currentX, yPos, currentX + _labelWidth, yPos + _rowHeight),
        false,
        false,
        _mainPanel
    );
    control.label->setOverrideColor(SColor(255, 200, 200, 200));

    currentX += _labelWidth + padding;

    // Create slider
    control.slider = _guienv->addScrollBar(
        true, // horizontal
        rect<s32>(currentX, yPos, currentX + _sliderWidth, yPos + _rowHeight),
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

    currentX += _sliderWidth + padding;

    // Create value display
    control.valueDisplay = _guienv->addStaticText(
        L"",
        rect<s32>(currentX, yPos, currentX + _valueWidth, yPos + _rowHeight),
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
    yPos += _rowHeight + _rowSpacing;
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