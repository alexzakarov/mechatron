// ============================================================================
// I18n.cpp — Stage 9 central bilingual label registry implementation.
// See I18n.hpp for the design rationale.
// ============================================================================

#include "I18n.hpp"

namespace mechatron {
namespace i18n {

namespace {

Language g_language = Language::Turkish;

} // namespace

Language current_language() {
    return g_language;
}

void set_language(Language lang) {
    g_language = lang;
}

bool set_language_by_code(std::string_view code) {
    if (code == "tr") { g_language = Language::Turkish; return true; }
    if (code == "en") { g_language = Language::English; return true; }
    return false;
}

std::string_view current_language_code() {
    return (g_language == Language::English) ? "en" : "tr";
}

const char* label_for(const char* tr, const char* en, const char* fallback) {
    const char* primary   = (g_language == Language::English) ? en : tr;
    const char* secondary = (g_language == Language::English) ? tr : en;

    if (primary && *primary)   return primary;
    if (secondary && *secondary) return secondary;
    return fallback ? fallback : "";
}

std::string label_for_string(const char* tr, const char* en, const char* fallback) {
    const char* out = label_for(tr, en, fallback);
    return out ? std::string(out) : std::string();
}

const char* tooltip_for(const char* tr, const char* en, const char* fallback) {
    // Same logic as label_for — kept as a separate function so callers can
    // distinguish "tooltip" intent in code review.
    return label_for(tr, en, fallback);
}

std::string bilingual_display(const char* tr, const char* en) {
    const bool has_tr = tr && *tr;
    const bool has_en = en && *en;
    if (has_tr && has_en) {
        return std::string(tr) + " (" + en + ")";
    }
    if (has_tr) return tr;
    if (has_en) return en;
    return {};
}

} // namespace i18n
} // namespace mechatron
