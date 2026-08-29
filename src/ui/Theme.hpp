#pragma once

#include <imgui.h>

namespace mechatron::Theme {

struct Palette {
    ImVec4 bg{0.059f, 0.067f, 0.082f, 1.0f};
    ImVec4 surface{0.102f, 0.114f, 0.137f, 1.0f};
    ImVec4 surfaceHover{0.137f, 0.157f, 0.212f, 1.0f};
    ImVec4 surfaceActive{0.165f, 0.188f, 0.235f, 1.0f};
    ImVec4 border{0.165f, 0.188f, 0.235f, 1.0f};
    ImVec4 borderStrong{0.204f, 0.227f, 0.275f, 1.0f};
    ImVec4 text{1.0f, 1.0f, 1.0f, 1.0f};
    ImVec4 textDim{0.784f, 0.816f, 0.863f, 1.0f};
    ImVec4 textDisabled{0.420f, 0.447f, 0.502f, 1.0f};
    ImVec4 primary{0.055f, 0.647f, 0.914f, 1.0f};
    ImVec4 primaryHover{0.220f, 0.741f, 0.973f, 1.0f};
    ImVec4 primaryActive{0.008f, 0.518f, 0.780f, 1.0f};
    ImVec4 success{0.133f, 0.773f, 0.369f, 1.0f};
    ImVec4 warning{0.961f, 0.620f, 0.043f, 1.0f};
    ImVec4 error{0.937f, 0.267f, 0.267f, 1.0f};
};

inline const Palette& DarkPalette() {
    static Palette p;
    return p;
}

inline const Palette& CurrentPalette() { return DarkPalette(); }

inline ImVec4 WithAlpha(ImVec4 c, float a) { c.w = a; return c; }

namespace Semantic {
    inline ImVec4 Wire() { return CurrentPalette().primary; }
    inline ImVec4 WireSelected() { return CurrentPalette().primaryHover; }
    inline ImVec4 Grid() { return WithAlpha(CurrentPalette().border, 0.35f); }
    inline ImVec4 GridCenter() { return WithAlpha(CurrentPalette().borderStrong, 0.45f); }
    inline ImVec4 JunctionDot() { return CurrentPalette().primary; }
    inline ImVec4 JunctionOutline() { return WithAlpha(CurrentPalette().text, 0.78f); }
    inline ImVec4 NodeBg() { return WithAlpha(CurrentPalette().surface, 0.95f); }
    inline ImVec4 NodeBgSelected() { return WithAlpha(CurrentPalette().surfaceHover, 0.95f); }
    inline ImVec4 NodeBorder() { return WithAlpha(CurrentPalette().borderStrong, 1.0f); }
    inline ImVec4 NodeBorderSelected() { return CurrentPalette().primary; }
    inline ImVec4 PinIdle() { return WithAlpha(CurrentPalette().primary, 0.95f); }
    inline ImVec4 PinHover() { return CurrentPalette().primaryHover; }
    inline ImVec4 PinOutline() { return WithAlpha(CurrentPalette().text, 0.78f); }
    inline ImVec4 PinLabel() { return CurrentPalette().textDim; }
    inline ImVec4 SchematicBody() { return WithAlpha(CurrentPalette().textDim, 0.86f); }
    inline ImVec4 Label() { return CurrentPalette().text; }
    inline ImVec4 ScopeBg() { return CurrentPalette().bg; }
    inline ImVec4 ScopeGrid() { return CurrentPalette().border; }
    inline ImVec4 ScopeBorder() { return CurrentPalette().borderStrong; }
    inline ImVec4 TimelineBg() { return CurrentPalette().surface; }
    inline ImVec4 Splitter() { return CurrentPalette().border; }
    inline ImVec4 SplitterHovered() { return CurrentPalette().primary; }
    inline ImVec4 Button() { return CurrentPalette().surfaceActive; }
    inline ImVec4 ButtonHovered() { return CurrentPalette().primaryHover; }
    inline ImVec4 ButtonActive() { return CurrentPalette().primaryActive; }
    inline ImVec4 ToolActive() { return CurrentPalette().primary; }
    inline ImVec4 ToolInactive() { return CurrentPalette().surfaceActive; }
}

inline ImU32 U32(ImVec4 c) { return ImGui::GetColorU32(c); }

inline void ApplyModernDark() {
    ImGuiStyle& style = ImGui::GetStyle();
    const Palette& p = CurrentPalette();

    style.WindowRounding = 8.0f;
    style.FrameRounding = 6.0f;
    style.GrabRounding = 6.0f;
    style.ScrollbarRounding = 8.0f;
    style.TabRounding = 6.0f;
    style.WindowPadding = ImVec2(10.0f, 10.0f);
    style.FramePadding = ImVec2(8.0f, 4.0f);
    style.ItemSpacing = ImVec2(8.0f, 6.0f);
    style.ItemInnerSpacing = ImVec2(6.0f, 6.0f);
    style.ScrollbarSize = 12.0f;
    style.WindowBorderSize = 0.0f;
    style.FrameBorderSize = 0.0f;
    style.PopupRounding = 6.0f;
    style.ChildRounding = 6.0f;

    ImVec4* c = style.Colors;
    c[ImGuiCol_Text] = p.text;
    c[ImGuiCol_TextDisabled] = p.textDisabled;
    c[ImGuiCol_WindowBg] = p.bg;
    c[ImGuiCol_ChildBg] = p.surface;
    c[ImGuiCol_PopupBg] = p.surface;
    c[ImGuiCol_Border] = WithAlpha(p.border, 0.55f);
    c[ImGuiCol_BorderShadow] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_FrameBg] = p.surfaceHover;
    c[ImGuiCol_FrameBgHovered] = WithAlpha(p.primary, 0.35f);
    c[ImGuiCol_FrameBgActive] = WithAlpha(p.primary, 0.55f);
    c[ImGuiCol_TitleBg] = p.surface;
    c[ImGuiCol_TitleBgActive] = p.surfaceHover;
    c[ImGuiCol_TitleBgCollapsed] = WithAlpha(p.surface, 0.85f);
    c[ImGuiCol_MenuBarBg] = p.surface;
    c[ImGuiCol_ScrollbarBg] = WithAlpha(p.bg, 1.0f);
    c[ImGuiCol_ScrollbarGrab] = p.borderStrong;
    c[ImGuiCol_ScrollbarGrabHovered] = p.textDisabled;
    c[ImGuiCol_ScrollbarGrabActive] = p.primary;
    c[ImGuiCol_CheckMark] = p.text;
    c[ImGuiCol_SliderGrab] = p.primary;
    c[ImGuiCol_SliderGrabActive] = p.primaryHover;
    c[ImGuiCol_Button] = p.surfaceActive;
    c[ImGuiCol_ButtonHovered] = p.primary;
    c[ImGuiCol_ButtonActive] = p.primaryActive;
    c[ImGuiCol_Header] = WithAlpha(p.primary, 0.28f);
    c[ImGuiCol_HeaderHovered] = WithAlpha(p.primary, 0.45f);
    c[ImGuiCol_HeaderActive] = WithAlpha(p.primary, 0.65f);
    c[ImGuiCol_Separator] = WithAlpha(p.border, 0.65f);
    c[ImGuiCol_SeparatorHovered] = p.primary;
    c[ImGuiCol_SeparatorActive] = p.primaryHover;
    c[ImGuiCol_ResizeGrip] = WithAlpha(p.borderStrong, 0.4f);
    c[ImGuiCol_ResizeGripHovered] = WithAlpha(p.primary, 0.6f);
    c[ImGuiCol_ResizeGripActive] = p.primary;
    c[ImGuiCol_Tab] = p.surface;
    c[ImGuiCol_TabHovered] = p.primary;
    c[ImGuiCol_TabActive] = p.surfaceHover;
    c[ImGuiCol_TabUnfocused] = p.surface;
    c[ImGuiCol_TabUnfocusedActive] = p.surfaceHover;
    c[ImGuiCol_PlotLines] = p.primary;
    c[ImGuiCol_PlotLinesHovered] = p.primaryHover;
    c[ImGuiCol_PlotHistogram] = p.primary;
    c[ImGuiCol_PlotHistogramHovered] = p.primaryHover;
    c[ImGuiCol_TableHeaderBg] = p.surfaceHover;
    c[ImGuiCol_TableBorderStrong] = p.borderStrong;
    c[ImGuiCol_TableBorderLight] = p.border;
    c[ImGuiCol_TableRowBg] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_TableRowBgAlt] = WithAlpha(p.surface, 0.45f);
    c[ImGuiCol_TextSelectedBg] = WithAlpha(p.primary, 0.35f);
    c[ImGuiCol_DragDropTarget] = p.primaryHover;
    c[ImGuiCol_NavHighlight] = p.primary;
    c[ImGuiCol_NavWindowingHighlight] = WithAlpha(p.primary, 0.7f);
    c[ImGuiCol_NavWindowingDimBg] = WithAlpha(p.bg, 0.72f);
    c[ImGuiCol_ModalWindowDimBg] = WithAlpha(p.bg, 0.72f);
}

}
