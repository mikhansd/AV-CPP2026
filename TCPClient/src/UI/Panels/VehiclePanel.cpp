#include "VehiclePanel.h"

#include <algorithm>
#include <stdio.h>

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "imgui.h"
#include "VehicleState.h"

void VehiclePanel::onUpdate()
{
    static bool headlightWasDown = false;
    static bool leftSigWasDown = false;
    static bool rightSigWasDown = false;

    ImGuiIO& io = ImGui::GetIO();
    ImGui::Begin("Vehicle Control");

    ImGui::Text("Vehicle State");
    ImGui::Separator();

    if (ImGui::BeginTable("LightsTable", 3, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_NoBordersInBody | ImGuiTableFlags_NoHostExtendX | ImGuiTableFlags_SizingFixedSame))
    {
        // Set up fixed-width columns (you can tune widths as needed)
        ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthFixed, 120); // Label
        ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthFixed, 50); // Status
        ImGui::TableSetupColumn(nullptr, ImGuiTableColumnFlags_WidthFixed, 100); // Toggle

        addLightRow("Headlights", "H", VehicleState::getInstance().lightStatus.Headlights);
        addLightRow("Left Turn", "Q", VehicleState::getInstance().lightStatus.leftSig);
        addLightRow("Right Turn", "E", VehicleState::getInstance().lightStatus.rightSig);
        addLightRow("Brake Lights", "", VehicleState::getInstance().lightStatus.brakeLights);

        ImGui::EndTable();
    }

    ImGui::Separator();

    ImGui::Text("Direction:");
    const char* directionStr = 
        VehicleState::getInstance().driveStatus.gear == GearID::Forward ? "Forward" :
        VehicleState::getInstance().driveStatus.gear == GearID::Reverse ? "Backward" :
        VehicleState::getInstance().driveStatus.braking ? "Braking" : "Neutral";
    ImGui::TextColored(ImVec4(0.7f, 0.7f, 1.0f, 1.0f), "%s", directionStr);

    ImGui::Separator();

    ImGui::Text("Max Speed Control");

    // Set fixed width for the slider
    ImGui::PushItemWidth(200);
    if (ImGui::SliderInt("Speed (%)", &maxSpeed, 0, 100)) {
        uiContext.controller->updateUISliderInt(SliderID::MaxSpeed, maxSpeed);
    }
    ImGui::PopItemWidth();

    ImGui::SameLine();

    // Set fixed width for the input box
    ImGui::PushItemWidth(85);

    // step = 1 means +/- buttons appear, step_fast = 5 means shift+click or right-click goes faster
    bool edited = ImGui::InputInt(
        "##SpeedInput",
        &maxSpeed,
        1, 5,                            // step, step_fast
        ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll
    );

    // Commit on press of enter key
    if (edited) {
        maxSpeed = std::clamp(maxSpeed, 0, 100);
        uiContext.controller->updateUISliderInt(SliderID::MaxSpeed, maxSpeed);
        lastSentSpeed = maxSpeed;
    }

    ImGui::PopItemWidth();

    ImGui::Text("Calibrate Heading Routine [1]"); 
    if (ImGui::Button("Begin##CalibrateHeading", ImVec2(100, 0))) {
        uiContext.controller->updateUIButton(ButtonID::CalibrateHeading, true);
    } else {
        uiContext.controller->updateUIButton(ButtonID::CalibrateHeading, false);
    }

    ImGui::End();
}

void VehiclePanel::addLightRow(const char *label, const char *keyHint, bool state)
{
    ImGui::TableNextRow();

    // Column 0: Label + key hint if provided
    ImGui::TableSetColumnIndex(0);
    if (keyHint && keyHint[0] != '\0')
        ImGui::Text("%s [%s]", label, keyHint);
    else
        ImGui::Text("%s", label);

    // Column 1: Colored ON/OFF
    ImGui::TableSetColumnIndex(1);
    ImVec4 color = state ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1);
    ImGui::TextColored(color, state ? "ON" : "OFF");
}
