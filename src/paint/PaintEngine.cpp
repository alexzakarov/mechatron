// ============================================================================
// PaintEngine.cpp — Stage 5 implementation.
// See PaintEngine.hpp for the brush-type / blend-mode mapping tables.
// ============================================================================

#include "PaintEngine.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace mechatron {

namespace {

constexpr float kColorEps = 1e-6f;

// Linear interpolate two colors.
inline Vec4 lerp_color(const Vec4& a, const Vec4& b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return a.lerp(b, t);
}

// Convert a Vec4 color (linear RGB in [0,1]) to a luminance value [0,1].
inline float luminance(const Vec4& c) {
    return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z;
}

} // namespace

// ---------------------------------------------------------------------------
// Stroke lifecycle
// ---------------------------------------------------------------------------
void PaintEngine::begin_stroke(PaintSession& session,
                               EditableMesh& mesh,
                               const Vec3& start_point) {
    session.active = true;
    session.first_sample = true;
    session.last_point = start_point;
    session.current_point = start_point;
    session.avg_color_accum = Vec4{0, 0, 0, 0};
    session.avg_color_count = 0;
    session.avg_weight_accum = 0.0f;
    session.avg_weight_count = 0;

    const size_t n = mesh.vertex_count();
    if (session.mode == PaintMode::Vertex || session.mode == PaintMode::Texture) {
        session.color_cache.assign(n, Vec4{0, 0, 0, 0});
        for (size_t i = 0; i < n; ++i) {
            session.color_cache[i] = get_vertex_color(mesh, static_cast<uint32_t>(i));
        }
    } else {
        session.weight_cache.assign(n, 0.0f);
        for (size_t i = 0; i < n; ++i) {
            session.weight_cache[i] =
                get_vertex_weight(mesh, session.brush.vertex_group_index, static_cast<uint32_t>(i));
        }
    }
}

bool PaintEngine::stroke_to(PaintSession& session,
                            EditableMesh& mesh,
                            const SpatialHash& spatial,
                            const Vec3& new_point) {
    if (!session.active) return false;
    session.current_point = new_point;

    const float radius   = std::max(1e-3f, session.brush.radius);
    const float strength = std::clamp(session.brush.strength, 0.0f, 1.0f);
    const bool  invert   = session.brush.invert;
    const BrushFalloff falloff = session.brush.falloff;
    const PaintBrushType type = session.brush.type;
    const PaintMode mode = session.mode;

    std::vector<uint32_t> hit_indices;
    spatial.query_sphere(new_point, radius, hit_indices);
    if (hit_indices.empty()) {
        session.last_point = new_point;
        session.first_sample = false;
        return false;
    }

    const std::vector<Vec3>& verts = mesh.vertices();

    // Helper: weight the falloff with the brush strength (matches Blender's
    // behavior where strength scales the brush's effect).
    auto weight_for = [&](uint32_t idx) {
        const Vec3& v = verts[idx];
        const float dx = v.x - new_point.x;
        const float dy = v.y - new_point.y;
        const float dz = v.z - new_point.z;
        const float d = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (d > radius) return -1.0f;
        return SculptEngine::falloff_weight(falloff, radius, d) * strength;
    };

    bool any_changed = false;

    switch (type) {
        case PaintBrushType::Draw: {
            if (mode == PaintMode::Vertex || mode == PaintMode::Texture) {
                for (uint32_t idx : hit_indices) {
                    if (idx >= verts.size()) continue;
                    const float w = weight_for(idx);
                    if (w < 0.0f) continue;
                    const Vec4 base = (idx < session.color_cache.size())
                        ? session.color_cache[idx] : get_vertex_color(mesh, idx);
                    const float t = invert ? w : w; // Invert handled by blend mode below.
                    const Vec4 out = invert
                        ? blend_colors(PaintBlendMode::Subtract, base, session.brush.color, t)
                        : blend_colors(session.brush.blend, base, session.brush.color, t);
                    set_vertex_color(mesh, idx, out);
                    any_changed = true;
                }
            } else { // Weight mode
                for (uint32_t idx : hit_indices) {
                    if (idx >= verts.size()) continue;
                    const float w = weight_for(idx);
                    if (w < 0.0f) continue;
                    const float current = (idx < session.weight_cache.size())
                        ? session.weight_cache[idx]
                        : get_vertex_weight(mesh, session.brush.vertex_group_index, idx);
                    // Invert removes weight (erases).
                    const float target = invert ? 0.0f : session.brush.weight;
                    const float next = std::clamp(current + (target - current) * w, 0.0f, 1.0f);
                    set_vertex_weight(mesh, session.brush.vertex_group_index, idx, next);
                    any_changed = true;
                }
            }
            break;
        }
        case PaintBrushType::Set: {
            if (mode == PaintMode::Vertex || mode == PaintMode::Texture) {
                for (uint32_t idx : hit_indices) {
                    if (idx >= verts.size()) continue;
                    const float w = weight_for(idx);
                    if (w < 0.0f) continue;
                    set_vertex_color(mesh, idx, session.brush.color);
                    any_changed = true;
                }
            } else {
                for (uint32_t idx : hit_indices) {
                    if (idx >= verts.size()) continue;
                    const float w = weight_for(idx);
                    if (w < 0.0f) continue;
                    const float target = invert ? 0.0f : session.brush.weight;
                    set_vertex_weight(mesh, session.brush.vertex_group_index, idx, target);
                    any_changed = true;
                }
            }
            break;
        }
        case PaintBrushType::Blur: {
            // Average each affected vertex's value with the average of its
            // neighbors (cached at stroke begin). This is a per-sample blur;
            // repeated strokes accumulate the smoothing effect.
            if (mode == PaintMode::Vertex || mode == PaintMode::Texture) {
                for (uint32_t idx : hit_indices) {
                    if (idx >= verts.size()) continue;
                    const float w = weight_for(idx);
                    if (w < 0.0f) continue;
                    std::vector<EdgeKey> edges = mesh.get_vertex_edges(idx);
                    if (edges.empty()) continue;
                    Vec4 sum{0, 0, 0, 0};
                    int n = 0;
                    for (const EdgeKey& ek : edges) {
                        const uint32_t other = (ek.v0 == idx) ? ek.v1
                                            : (ek.v1 == idx ? ek.v0 : UINT32_MAX);
                        if (other >= verts.size()) continue;
                        sum = sum + get_vertex_color(mesh, other);
                        ++n;
                    }
                    if (n == 0) continue;
                    const Vec4 avg = sum * (1.0f / static_cast<float>(n));
                    const Vec4 current = get_vertex_color(mesh, idx);
                    set_vertex_color(mesh, idx, lerp_color(current, avg, w * 0.5f));
                    any_changed = true;
                }
            } else {
                for (uint32_t idx : hit_indices) {
                    if (idx >= verts.size()) continue;
                    const float w = weight_for(idx);
                    if (w < 0.0f) continue;
                    std::vector<EdgeKey> edges = mesh.get_vertex_edges(idx);
                    if (edges.empty()) continue;
                    float sum = 0.0f;
                    int n = 0;
                    for (const EdgeKey& ek : edges) {
                        const uint32_t other = (ek.v0 == idx) ? ek.v1
                                            : (ek.v1 == idx ? ek.v0 : UINT32_MAX);
                        if (other >= verts.size()) continue;
                        sum += get_vertex_weight(mesh, session.brush.vertex_group_index, other);
                        ++n;
                    }
                    if (n == 0) continue;
                    const float avg = sum / static_cast<float>(n);
                    const float current = get_vertex_weight(mesh, session.brush.vertex_group_index, idx);
                    const float next = current + (avg - current) * w * 0.5f;
                    set_vertex_weight(mesh, session.brush.vertex_group_index, idx, next);
                    any_changed = true;
                }
            }
            break;
        }
        case PaintBrushType::Average: {
            // Accumulate colors touched by this stroke and blend all touched
            // vertices toward the running average.
            if (mode == PaintMode::Vertex || mode == PaintMode::Texture) {
                Vec4 sum{0, 0, 0, 0};
                int count = 0;
                for (uint32_t idx : hit_indices) {
                    if (idx >= verts.size()) continue;
                    sum = sum + get_vertex_color(mesh, idx);
                    ++count;
                }
                if (count == 0) break;
                const Vec4 avg = sum * (1.0f / static_cast<float>(count));
                for (uint32_t idx : hit_indices) {
                    if (idx >= verts.size()) continue;
                    const float w = weight_for(idx);
                    if (w < 0.0f) continue;
                    const Vec4 current = get_vertex_color(mesh, idx);
                    set_vertex_color(mesh, idx, lerp_color(current, avg, w * 0.5f));
                    any_changed = true;
                }
            } else {
                float sum = 0.0f;
                int count = 0;
                for (uint32_t idx : hit_indices) {
                    if (idx >= verts.size()) continue;
                    sum += get_vertex_weight(mesh, session.brush.vertex_group_index, idx);
                    ++count;
                }
                if (count == 0) break;
                const float avg = sum / static_cast<float>(count);
                for (uint32_t idx : hit_indices) {
                    if (idx >= verts.size()) continue;
                    const float w = weight_for(idx);
                    if (w < 0.0f) continue;
                    const float current = get_vertex_weight(mesh, session.brush.vertex_group_index, idx);
                    const float next = current + (avg - current) * w * 0.5f;
                    set_vertex_weight(mesh, session.brush.vertex_group_index, idx, next);
                    any_changed = true;
                }
            }
            break;
        }
        case PaintBrushType::Smear: {
            // Drag each touched vertex's value toward the value of the
            // closest touched vertex on the *previous* stroke sample.
            const Vec3 smear_dir{
                new_point.x - session.last_point.x,
                new_point.y - session.last_point.y,
                new_point.z - session.last_point.z,
            };
            const float smear_len = smear_dir.length();
            if (smear_len < kColorEps) break;
            if (mode == PaintMode::Vertex || mode == PaintMode::Texture) {
                for (uint32_t idx : hit_indices) {
                    if (idx >= verts.size()) continue;
                    const float w = weight_for(idx);
                    if (w < 0.0f) continue;
                    // Sample a vertex just behind the stroke direction.
                    const Vec3 back{
                        verts[idx].x - smear_dir.x,
                        verts[idx].y - smear_dir.y,
                        verts[idx].z - smear_dir.z,
                    };
                    std::vector<uint32_t> probe;
                    spatial.query_sphere(back, radius * 0.3f, probe);
                    if (probe.empty()) continue;
                    // Closest probe vertex.
                    uint32_t best = probe[0];
                    float best_d2 = std::numeric_limits<float>::max();
                    for (uint32_t p : probe) {
                        if (p >= verts.size()) continue;
                        const float dx = verts[p].x - back.x;
                        const float dy = verts[p].y - back.y;
                        const float dz = verts[p].z - back.z;
                        const float d2 = dx * dx + dy * dy + dz * dz;
                        if (d2 < best_d2) { best_d2 = d2; best = p; }
                    }
                    const Vec4 dragged = get_vertex_color(mesh, best);
                    const Vec4 current = get_vertex_color(mesh, idx);
                    set_vertex_color(mesh, idx, lerp_color(current, dragged, w * 0.5f));
                    any_changed = true;
                }
            } else {
                for (uint32_t idx : hit_indices) {
                    if (idx >= verts.size()) continue;
                    const float w = weight_for(idx);
                    if (w < 0.0f) continue;
                    const Vec3 back{
                        verts[idx].x - smear_dir.x,
                        verts[idx].y - smear_dir.y,
                        verts[idx].z - smear_dir.z,
                    };
                    std::vector<uint32_t> probe;
                    spatial.query_sphere(back, radius * 0.3f, probe);
                    if (probe.empty()) continue;
                    uint32_t best = probe[0];
                    float best_d2 = std::numeric_limits<float>::max();
                    for (uint32_t p : probe) {
                        if (p >= verts.size()) continue;
                        const float dx = verts[p].x - back.x;
                        const float dy = verts[p].y - back.y;
                        const float dz = verts[p].z - back.z;
                        const float d2 = dx * dx + dy * dy + dz * dz;
                        if (d2 < best_d2) { best_d2 = d2; best = p; }
                    }
                    const float dragged = get_vertex_weight(mesh, session.brush.vertex_group_index, best);
                    const float current = get_vertex_weight(mesh, session.brush.vertex_group_index, idx);
                    const float next = current + (dragged - current) * w * 0.5f;
                    set_vertex_weight(mesh, session.brush.vertex_group_index, idx, next);
                    any_changed = true;
                }
            }
            break;
        }
        case PaintBrushType::Add: {
            // Weight only.
            for (uint32_t idx : hit_indices) {
                if (idx >= verts.size()) continue;
                const float w = weight_for(idx);
                if (w < 0.0f) continue;
                const float current = (idx < session.weight_cache.size())
                    ? session.weight_cache[idx]
                    : get_vertex_weight(mesh, session.brush.vertex_group_index, idx);
                const float delta = invert ? -session.brush.weight * w
                                           :  session.brush.weight * w;
                const float next = std::clamp(current + delta, 0.0f, 1.0f);
                set_vertex_weight(mesh, session.brush.vertex_group_index, idx, next);
                any_changed = true;
            }
            break;
        }
        case PaintBrushType::Subtract: {
            // Weight only.
            for (uint32_t idx : hit_indices) {
                if (idx >= verts.size()) continue;
                const float w = weight_for(idx);
                if (w < 0.0f) continue;
                const float current = (idx < session.weight_cache.size())
                    ? session.weight_cache[idx]
                    : get_vertex_weight(mesh, session.brush.vertex_group_index, idx);
                const float delta = invert ?  session.brush.weight * w
                                           : -session.brush.weight * w;
                const float next = std::clamp(current + delta, 0.0f, 1.0f);
                set_vertex_weight(mesh, session.brush.vertex_group_index, idx, next);
                any_changed = true;
            }
            break;
        }
        default: break;
    }

    // Weight paint normalization across all groups.
    if (any_changed && mode == PaintMode::Weight && session.brush.auto_normalize) {
        for (uint32_t idx : hit_indices) {
            if (idx >= mesh.vertex_count()) continue;
            mesh.normalize_vertex_weights_public(idx);
        }
    }

    session.last_point = new_point;
    session.first_sample = false;
    return any_changed;
}

void PaintEngine::end_stroke(PaintSession& session) {
    session.active = false;
}

// ---------------------------------------------------------------------------
// Color / weight access — delegate to EditableMesh's public Stage 5 methods.
// ---------------------------------------------------------------------------
Vec4 PaintEngine::get_vertex_color(const EditableMesh& mesh, uint32_t vertex) {
    return mesh.get_vertex_color(vertex);
}

void PaintEngine::set_vertex_color(EditableMesh& mesh, uint32_t vertex, const Vec4& color) {
    mesh.set_vertex_color(vertex, color);
}

bool PaintEngine::has_vertex_color_layer(const EditableMesh& mesh) {
    return mesh.has_vertex_color_layer();
}

float PaintEngine::get_vertex_weight(const EditableMesh& mesh,
                                     size_t vertex_group_index, uint32_t vertex) {
    return mesh.get_vertex_weight_public(vertex_group_index, vertex);
}

void PaintEngine::set_vertex_weight(EditableMesh& mesh,
                                    size_t vertex_group_index,
                                    uint32_t vertex, float weight) {
    mesh.set_vertex_weight_public(vertex_group_index, vertex, weight);
}

// ---------------------------------------------------------------------------
// Blend helpers — Blender IMB_blend_mode subset.
// ---------------------------------------------------------------------------
Vec4 PaintEngine::blend_colors(PaintBlendMode mode,
                               const Vec4& base, const Vec4& brush_color, float t) {
    Vec4 result = base;
    switch (mode) {
        case PaintBlendMode::Mix:
            result = lerp_color(base, brush_color, t);
            break;
        case PaintBlendMode::Add:
            result = Vec4{
                std::clamp(base.x + brush_color.x * t, 0.0f, 1.0f),
                std::clamp(base.y + brush_color.y * t, 0.0f, 1.0f),
                std::clamp(base.z + brush_color.z * t, 0.0f, 1.0f),
                base.w,
            };
            break;
        case PaintBlendMode::Subtract:
            result = Vec4{
                std::clamp(base.x - brush_color.x * t, 0.0f, 1.0f),
                std::clamp(base.y - brush_color.y * t, 0.0f, 1.0f),
                std::clamp(base.z - brush_color.z * t, 0.0f, 1.0f),
                base.w,
            };
            break;
        case PaintBlendMode::Multiply:
            result = Vec4{
                base.x * (1.0f - t + brush_color.x * t),
                base.y * (1.0f - t + brush_color.y * t),
                base.z * (1.0f - t + brush_color.z * t),
                base.w,
            };
            break;
        case PaintBlendMode::Lighten: {
            // Per-channel max blended by t.
            const Vec4 mixed = lerp_color(base, brush_color, t);
            result = Vec4{
                std::max(base.x, mixed.x),
                std::max(base.y, mixed.y),
                std::max(base.z, mixed.z),
                base.w,
            };
            break;
        }
        case PaintBlendMode::Darken: {
            const Vec4 mixed = lerp_color(base, brush_color, t);
            result = Vec4{
                std::min(base.x, mixed.x),
                std::min(base.y, mixed.y),
                std::min(base.z, mixed.z),
                base.w,
            };
            break;
        }
        case PaintBlendMode::Color: {
            // HSV: take hue/sat from brush, value from base.
            // Simplified: lerp base RGB toward brush RGB keeping base luminance.
            const float target_lum = luminance(brush_color);
            const float base_lum = luminance(base);
            const float shift = (target_lum - base_lum) * t;
            Vec4 mixed = lerp_color(base, brush_color, t);
            result = Vec4{
                std::clamp(mixed.x + shift * 0.5f, 0.0f, 1.0f),
                std::clamp(mixed.y + shift * 0.5f, 0.0f, 1.0f),
                std::clamp(mixed.z + shift * 0.5f, 0.0f, 1.0f),
                base.w,
            };
            break;
        }
        default:
            result = lerp_color(base, brush_color, t);
            break;
    }
    return result;
}

// ---------------------------------------------------------------------------
// Names
// ---------------------------------------------------------------------------
const char* PaintEngine::brush_name_tr(PaintBrushType type) {
    switch (type) {
        case PaintBrushType::Draw:     return "Boyama (Draw)";
        case PaintBrushType::Blur:     return "Bulanıklaştır (Blur)";
        case PaintBrushType::Smear:    return "Sürükle (Smear)";
        case PaintBrushType::Average:  return "Ortala (Average)";
        case PaintBrushType::Set:      return "Ayarla (Set)";
        case PaintBrushType::Add:      return "Ekle (Add)";
        case PaintBrushType::Subtract: return "Çıkar (Subtract)";
        case PaintBrushType::Count:    break;
    }
    return "?";
}

const char* PaintEngine::brush_name_en(PaintBrushType type) {
    switch (type) {
        case PaintBrushType::Draw:     return "Draw";
        case PaintBrushType::Blur:     return "Blur";
        case PaintBrushType::Smear:    return "Smear";
        case PaintBrushType::Average:  return "Average";
        case PaintBrushType::Set:      return "Set";
        case PaintBrushType::Add:      return "Add";
        case PaintBrushType::Subtract: return "Subtract";
        case PaintBrushType::Count:    break;
    }
    return "?";
}

const char* PaintEngine::blend_mode_name(PaintBlendMode mode) {
    switch (mode) {
        case PaintBlendMode::Mix:      return "Mix (Karışım)";
        case PaintBlendMode::Add:      return "Add (Ekle)";
        case PaintBlendMode::Subtract: return "Subtract (Çıkar)";
        case PaintBlendMode::Multiply: return "Multiply (Çarp)";
        case PaintBlendMode::Lighten:  return "Lighten (Açık)";
        case PaintBlendMode::Darken:   return "Darken (Koyu)";
        case PaintBlendMode::Color:    return "Color (Renk)";
        case PaintBlendMode::Count:    break;
    }
    return "?";
}

// ---------------------------------------------------------------------------
// PAINT_OT_* one-shot operators
// ---------------------------------------------------------------------------
void PaintEngine::set_all_vertex_colors(EditableMesh& mesh, const Vec4& color) {
    for (uint32_t v = 0; v < mesh.vertex_count(); ++v) {
        set_vertex_color(mesh, v, color);
    }
}

void PaintEngine::clear_vertex_colors(EditableMesh& mesh) {
    set_all_vertex_colors(mesh, Vec4{1.0f, 1.0f, 1.0f, 1.0f});
}

void PaintEngine::set_all_weights(EditableMesh& mesh, size_t group_index, float weight) {
    if (group_index >= mesh.vertex_group_count_public()) return;
    for (uint32_t v = 0; v < mesh.vertex_count(); ++v) {
        mesh.set_vertex_weight_public(group_index, v, std::clamp(weight, 0.0f, 1.0f));
    }
}

void PaintEngine::normalize_all_weights(EditableMesh& mesh) {
    for (uint32_t v = 0; v < mesh.vertex_count(); ++v) {
        mesh.normalize_vertex_weights_public(v);
    }
}

void PaintEngine::invert_vertex_colors(EditableMesh& mesh) {
    for (uint32_t v = 0; v < mesh.vertex_count(); ++v) {
        const Vec4 c = get_vertex_color(mesh, v);
        set_vertex_color(mesh, v, Vec4{1.0f - c.x, 1.0f - c.y, 1.0f - c.z, c.w});
    }
}

void PaintEngine::blur_all_vertex_colors(EditableMesh& mesh) {
    // One pass of neighbor-averaging.
    std::vector<Vec4> blurred(mesh.vertex_count(), Vec4{0, 0, 0, 0});
    for (uint32_t v = 0; v < mesh.vertex_count(); ++v) {
        std::vector<EdgeKey> edges = mesh.get_vertex_edges(v);
        if (edges.empty()) { blurred[v] = get_vertex_color(mesh, v); continue; }
        Vec4 sum{0, 0, 0, 0};
        int n = 0;
        for (const EdgeKey& ek : edges) {
            const uint32_t other = (ek.v0 == v) ? ek.v1 : (ek.v1 == v ? ek.v0 : UINT32_MAX);
            if (other >= mesh.vertex_count()) continue;
            sum = sum + get_vertex_color(mesh, other);
            ++n;
        }
        if (n == 0) { blurred[v] = get_vertex_color(mesh, v); continue; }
        const Vec4 avg = sum * (1.0f / static_cast<float>(n));
        const Vec4 current = get_vertex_color(mesh, v);
        blurred[v] = lerp_color(current, avg, 0.5f);
    }
    for (uint32_t v = 0; v < mesh.vertex_count(); ++v) {
        set_vertex_color(mesh, v, blurred[v]);
    }
}

void PaintEngine::levels_vertex_colors(EditableMesh& mesh, float gain, float bias) {
    for (uint32_t v = 0; v < mesh.vertex_count(); ++v) {
        const Vec4 c = get_vertex_color(mesh, v);
        set_vertex_color(mesh, v, Vec4{
            std::clamp(c.x * gain + bias, 0.0f, 1.0f),
            std::clamp(c.y * gain + bias, 0.0f, 1.0f),
            std::clamp(c.z * gain + bias, 0.0f, 1.0f),
            c.w,
        });
    }
}

void PaintEngine::dirty_vertex_colors(EditableMesh& mesh, float strength) {
    // "Dirty" : measure curvature as |dN/dx| and write it as grayscale.
    // We approximate curvature as the variance of neighbor normals.
    std::vector<Vec4> out(mesh.vertex_count(), Vec4{0, 0, 0, 1});
    for (uint32_t v = 0; v < mesh.vertex_count(); ++v) {
        const Vec3 n0 = mesh.vertex_normal(v);
        std::vector<EdgeKey> edges = mesh.get_vertex_edges(v);
        if (edges.empty()) { out[v] = get_vertex_color(mesh, v); continue; }
        float total = 0.0f;
        int n = 0;
        for (const EdgeKey& ek : edges) {
            const uint32_t other = (ek.v0 == v) ? ek.v1 : (ek.v1 == v ? ek.v0 : UINT32_MAX);
            if (other >= mesh.vertex_count()) continue;
            const Vec3 nn = mesh.vertex_normal(other);
            const Vec3 d{n0.x - nn.x, n0.y - nn.y, n0.z - nn.z};
            total += d.length();
            ++n;
        }
        const float curvature = (n > 0) ? total / static_cast<float>(n) : 0.0f;
        const float v01 = std::clamp(curvature * strength, 0.0f, 1.0f);
        out[v] = Vec4{v01, v01, v01, 1.0f};
    }
    for (uint32_t v = 0; v < mesh.vertex_count(); ++v) {
        set_vertex_color(mesh, v, out[v]);
    }
}

} // namespace mechatron
