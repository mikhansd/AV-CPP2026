#include "CameraFeedPanel.h"

#include <glad/glad.h>
#include <iostream>

#ifndef IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#define IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "Shared/Keys.h"

CameraFeedPanel::CameraFeedPanel(UIContext& uiContext)
    : UIPanel(uiContext), videoStream("", frameProcessor) 
{
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindTexture(GL_TEXTURE_2D, 0);

    frameProcessor.start();
}

CameraFeedPanel::~CameraFeedPanel()
{
    stopStream();
    frameProcessor.stop();
    glDeleteTextures(1, &texture);
}

void CameraFeedPanel::onUpdate()
{
    bool open = true;
    const bool showContents = ImGui::Begin("Camera Feed", &open);

    // --- Controls ---
    ImGui::Checkbox("Enable stream", &userEnabled);

    // Determine current host route
    currHost = uiContext.connectionType();   // <-- FIX: route was undefined before
    if (currHost != lastHost) {              // track switching Local <-> Remote
        lastHost = currHost;
        urlDirty = true;
    }

    if (currHost == ConnectionType::Local) {
        ImGui::SameLine();
        if (ImGui::Checkbox("Use remote relay (even on local)", &forceRemoteOnLocal)) {
            urlDirty = true;
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Prefer remote stream even when connected locally.");
    }

    ImGui::Separator();

    const bool connected = uiContext.isConnected();
    const bool visible   = showContents && !ImGui::IsWindowCollapsed();
    const bool wantStream = connected && userEnabled && visible;

    // --- Start/Stop gating ---
    if (wantStream && (!streaming || urlDirty)) {
        std::string url = buildRtspUrl();
        if (!url.empty() && canToggle()) {
            std::cout << "Setting stream URL to: " << url << std::endl;
            videoStream.restart(url);
            streaming = true;
            urlDirty = false;
            lastToggle = std::chrono::steady_clock::now();
        }
    } else if (!wantStream && streaming) {
        stopStream();
    }

    // --- Status line ---
    const char* routeText =
        (currHost == ConnectionType::Local)  ? "Local" :
        (currHost == ConnectionType::Remote) ? "Remote" : "Unknown";

    ImGui::Text("Connection: %s (%s)",
                connected ? "Connected" : "Not connected",
                routeText);

    ImGui::SameLine();
    ImGui::Text("| Stream: %s", streaming ? "Running" : "Stopped");

    std::string hosttext =
        (lastHost == ConnectionType::Local)  ? LOCAL_PI_IP :
        (lastHost == ConnectionType::Remote) ? VM_PUBLIC_IP :
                                               "None";

    if (lastHost != ConnectionType::None) {
        ImGui::SameLine();
        ImGui::Text("| Target: %s", hosttext.c_str());
    }

    // --- Rendering ---
    if (showContents) {
        Frame frame = frameProcessor.getFrame();
        ImVec2 contentSize = ImGui::GetContentRegionAvail();

        const float aspect =
            (frame.ready && frame.height > 0)
            ? float(frame.width) / float(frame.height)
            : (16.0f/9.0f);

        float w = contentSize.x;
        float h = w / aspect;
        if (h > contentSize.y) { h = contentSize.y; w = h * aspect; }
        float offsetX = (contentSize.x - w) * 0.5f;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offsetX);

        if (streaming) {
            if (frame.ready) {
                glBindTexture(GL_TEXTURE_2D, texture);
                glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

                if (frame.width != prevWidth || frame.height != prevHeight) {
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame.width, frame.height,
                                 0, GL_RGB, GL_UNSIGNED_BYTE, frame.pixels.data());
                    prevWidth = frame.width;
                    prevHeight = frame.height;
                } else {
                    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, frame.width, frame.height,
                                    GL_RGB, GL_UNSIGNED_BYTE, frame.pixels.data());
                }

                glBindTexture(GL_TEXTURE_2D, 0);
                ImGui::Image((ImTextureID)(intptr_t)texture, ImVec2(w, h));
            } else {
                ImGui::TextColored(ImVec4(1,0,0,1), "Waiting for frames...");
            }
        } else {
            if (!connected && userEnabled)
                ImGui::TextWrapped("Not connected. The stream will start when vehicle connects.");
            else
                ImGui::TextWrapped("Stream is stopped. Enable it above to start.");
        }
    }

    ImGui::End();
}

bool CameraFeedPanel::canToggle() const {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now() - lastToggle).count() > 500;
}

void CameraFeedPanel::startStream() {
    if (streaming || !canToggle()) return;
    videoStream.start();
    streaming = true;
    lastToggle = std::chrono::steady_clock::now();
}

void CameraFeedPanel::stopStream() {
    if (!streaming || !canToggle()) return;
    videoStream.stop();
    streaming = false;
    lastToggle = std::chrono::steady_clock::now();
}

std::string CameraFeedPanel::buildRtspUrl() const {
    std::string host;
    int port = 0;
    std::string path = "/stream";

    switch (currHost) {
        case ConnectionType::Local:
            if (forceRemoteOnLocal) {
                host = VM_PUBLIC_IP;
                port = REMOTE_VIDEO_PORT;
            } else {
                host = LOCAL_PI_IP;
                port = LOCAL_VIDEO_PORT;
            }
            break;

        case ConnectionType::Remote:
            host = VM_PUBLIC_IP;
            port = REMOTE_VIDEO_PORT;
            break;

        default:
            return {};
    }

    return "rtsp://" + host + ":" + std::to_string(port) + path;
}
