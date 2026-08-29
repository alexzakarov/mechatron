#pragma once

// ============================================================================
// I18n — Stage 9 central bilingual label registry.
//
// Mechatron's UI is bilingual (Turkish + English). Before Stage 9, labels
// were scattered per-file as either "TR (EN)" combined strings (e.g.
// "Böl (Subdivide)") or as separate _tr / _en function pairs. This header
// provides a single runtime-switchable API so panels can render labels in
// whichever language UIApplication::m_ui_language is set to.
//
// The registry is intentionally simple: it stores a global language enum
// (Language::Turkish / Language::English) and offers helpers to pick the
// right string from a TR/EN pair. The pie menus, sculpt/paint toolbars,
// top menu, and ModelEditor's operator specs all read this global.
//
// Design notes:
//   - Global state is OK here because the language preference is an
//     application-wide setting (mirrors Blender's USER enterprise pattern).
//   - We deliberately do NOT introduce a translation file format (JSON,
//     gettext .po). Mechatron's labels are baked into the operator specs
//     at compile time; the registry just routes them.
// ============================================================================

#include <cstdint>
#include <string>
#include <string_view>

namespace mechatron {
namespace i18n {

// Supported UI languages.
enum class Language : uint8_t {
    Turkish = 0,    // Default for Mechatron (TR-first convention)
    English = 1,
};

// Returns the active UI language. Defaults to Turkish.
Language current_language();

// Sets the active UI language. All subsequent label_for() / tooltip_for()
// calls return strings from the chosen language.
void set_language(Language lang);

// Convenience: set by string ("tr" / "en"). Unknown values keep the current
// language and return false.
bool set_language_by_code(std::string_view code);

// Convenience: current language as a short code ("tr" / "en").
std::string_view current_language_code();

// Pick the right label from a TR/EN pair. Falls back to whichever side is
// non-empty if the preferred one is blank; returns `fallback` if both are.
const char* label_for(const char* tr, const char* en,
                      const char* fallback = "");

// Same as label_for but returns a std::string so callers can compose.
std::string label_for_string(const char* tr, const char* en,
                             const char* fallback = "");

// Pick the right tooltip from a TR/EN pair. Same fallback rule as label_for.
const char* tooltip_for(const char* tr, const char* en,
                        const char* fallback = "");

// Combined display label in the form "TR (EN)" — used by specs that haven't
// been fully migrated to separate TR/EN fields yet. Returns the EN label
// alone if `tr` is empty, or the TR label alone if `en` is empty.
std::string bilingual_display(const char* tr, const char* en);

} // namespace i18n
} // namespace mechatron
