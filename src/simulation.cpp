#include "simulation.hpp"
#include "table.hpp"

namespace snooker {
namespace {

struct contact {
    std::size_t a;     // collider index
    std::size_t b;     // collider index
    glm::vec2 normal;  // from A to B (or into circle for wall)
    float penetration; // overlap depth
};

struct collision_info
{
    glm::vec2 normal;
    float     penetration;
};

auto safe_inverse(float x) -> float
{
    if (x == 0.0f) { return 0.0f; }
    return 1.0f / x;
}

auto inv_mass(const collider& c) -> float
{
    if (!std::holds_alternative<dynamic_body>(c.body)) return 0;
    return safe_inverse(std::get<dynamic_body>(c.body).mass);
}

auto velocity(const collider& c) -> glm::vec2
{
    if (std::holds_alternative<dynamic_body>(c.body)) {
        return std::get<dynamic_body>(c.body).vel;
    }
    return {0.0f, 0.0f};
}

auto apply_impulse(collider& c, glm::vec2 impulse) -> void
{
    if (std::holds_alternative<dynamic_body>(c.body)) {
        auto& body = std::get<dynamic_body>(c.body);
        body.vel += impulse * safe_inverse(body.mass);
    }
}

auto flip_normal(const collision_info& info) -> collision_info
{
    auto copy = info;
    copy.normal *= -1;
    return copy;
}

auto collision_circle_circle(glm::vec2 pos_a, glm::vec2 pos_b, circle_shape shape_a, circle_shape shape_b) -> std::optional<collision_info>
{
    const auto delta = pos_b - pos_a;
    const auto dist = glm::length(delta);
    const auto r = shape_a.radius + shape_b.radius;
    if (dist < r) {
        const auto n = (dist > 1e-6f) ? delta / dist : glm::vec2(1, 0);
        return collision_info{n, r - dist};
    }
    return {};
}

auto collision_circle_box(glm::vec2 pos_a, glm::vec2 pos_b, circle_shape shape_a, box_shape shape_b) -> std::optional<collision_info>
{
    const auto half_extents = glm::vec2{shape_b.width, shape_b.height} / 2.0f;
            
    // closest point in the box to the circles centre
    const auto P = glm::clamp(pos_a, pos_b - half_extents, pos_b + half_extents);

    const auto delta = P - pos_a;
    const auto dist = glm::length(delta);
    if (dist < shape_a.radius) { // collision
        if (pos_a != P) { // circle centre is outside the box (the clamp moved the centre)
            const auto n = (dist > 1e-6f) ? delta / dist : glm::vec2(1, 0);
            return collision_info{n, shape_a.radius - dist};
        }
        assert_that(false, "TODO: make this collision happen");
    }
    return {};
}

auto collision_circle_line(glm::vec2 pos_a, glm::vec2 pos_b, circle_shape shape_a, line_shape shape_b) -> std::optional<collision_info>
{
    const auto circle_pos = pos_a;
    const auto circle_radius = shape_a.radius;
    const auto line_start = pos_b + shape_b.start;
    const auto line_end = pos_b + shape_b.end;
    
    auto diff = glm::vec2{0.0f, 0.0f};
    if (line_start == line_end) { // degenerate line
        diff = line_start - circle_pos;
    } else {
        // project circle center onto line segment
        const auto line_vec = line_end - line_start;
        const auto line_len_sq = glm::dot(line_vec, line_vec);
        const auto t = glm::clamp(glm::dot(circle_pos - line_start, line_vec) / line_len_sq, 0.0f, 1.0f);
    
        const auto closest = line_start + t * line_vec;
        diff = closest - circle_pos;
    }

    const auto dist = glm::length(diff);
    if (dist >= circle_radius) return {};
    const auto normal = (dist > 0.0f) ? diff / dist : glm::vec2(1, 0); // arbitrary normal if center exactly on line
    return collision_info{normal, circle_radius - dist};
}

auto collision_box_box(glm::vec2 pos_a, glm::vec2 pos_b, box_shape shape_a, box_shape shape_b) -> std::optional<collision_info>
{
    assert_that(false, "unhandled collision type! box-box");
    return {};
}

auto collision_box_line(glm::vec2 pos_a, glm::vec2 pos_b, box_shape shape_a, line_shape shape_b) -> std::optional<collision_info>
{
    assert_that(false, "unhandled collision type! box-line");
    return {};
}

auto collision_line_line(glm::vec2 pos_a, glm::vec2 pos_b, line_shape shape_a, line_shape shape_b) -> std::optional<collision_info>
{
    assert_that(false, "unhandled collision type! line-line");
    return {};
}

// Checks if two colliders are colliding, and returns the normal of the collision if the yare
auto collision_test(const collider& a, const collider& b) -> std::optional<collision_info>
{
    // only check for collisions between two dynamic bodies
    if (!std::holds_alternative<dynamic_body>(a.body) && !std::holds_alternative<dynamic_body>(b.body)) return {};

    return std::visit(overloaded{
        [&](const circle_shape& A, const circle_shape& B) {
            return collision_circle_circle(a.pos, b.pos, A, B);
        },
        [&](const circle_shape& A, const box_shape& B) {
            return collision_circle_box(a.pos, b.pos, A, B);
        },
        [&](const circle_shape& A, const line_shape& B) {
            return collision_circle_line(a.pos, b.pos, A, B);
        },

        [&](const box_shape& A, const circle_shape& B) {
            return collision_circle_box(b.pos, a.pos, B, A).transform(flip_normal);
        },
        [&](const box_shape& A, const box_shape& B) {
            return collision_box_box(a.pos, b.pos, A, B);
        },
        [&](const box_shape& A, const line_shape& B) {
            return collision_box_line(a.pos, b.pos, A, B);
        },

        [&](const line_shape& A, const circle_shape& B) {
            return collision_circle_line(b.pos, a.pos, B, A).transform(flip_normal);
        },
        [&](const line_shape& A, const box_shape& B) {
            return collision_box_line(b.pos, a.pos, B, A).transform(flip_normal);
        },
        [&](const line_shape& A, const line_shape& B) {
            return collision_line_line(a.pos, b.pos, A, B);
        },

        [&](auto&&, auto&&) -> std::optional<collision_info> {
            assert_that(false, "unhandled collision type!");
            return {};
        }
    }, a.shape, b.shape);
}

auto generate_contacts(const std::vector<collider>& colliders) -> std::vector<contact>
{
    std::vector<contact> contacts;

    for (std::size_t i = 0; i < colliders.size(); ++i) {
        for (std::size_t j = i + 1; j < colliders.size(); ++j) {
            if (const auto ci = collision_test(colliders[i], colliders[j])) {
                contacts.push_back({ i, j, ci->normal, ci->penetration });
            }
        }
    }

    return contacts;
}

void solve_contacts(std::vector<collider>& colliders,
                    const std::vector<contact>& contacts,
                    float restitution = 0.8f)
{
    const auto N = contacts.size();
    if (N == 0) {
        return;
    }

    // set up equation A*j = b, where A is the constraint matrix, j is the unknown
    // impulse vector, and b is the desired velocity change (the bias term).
    // to do this, start with A*b and use Gaussian elimination, which results in b
    // turning into j

    std::vector<float> A(N * N, 0.0f);
    std::vector<float> b(N, 0.0f);

    for (int i = 0; i < N; ++i) {
        const auto& ci = contacts[i];
        const auto a1 = ci.a;
        const auto b1 = ci.b;
        const auto normal_i = ci.normal;

        const auto rv = velocity(colliders[b1]) - velocity(colliders[a1]);
        const auto rel_vel = glm::dot(rv, normal_i);

        // only apply restitution if moving into contact
        b[i] = (rel_vel < -1e-6f) ? -(1.0f + restitution) * rel_vel : 0.0f;

        // baumgarte positional bias
        b[i] += 0.2f * std::max(ci.penetration, 0.0f);

        // fill constraint matrix
        for (int j = 0; j < N; ++j) {
            const auto& cj = contacts[j];
            const auto a2 = cj.a;
            const auto b2 = cj.b;
            const auto normal_j = cj.normal;

            auto val = 0.0f;
            const auto dot = glm::dot(normal_i, normal_j);
            if (a1 == a2) {
                val += dot * inv_mass(colliders[a1]);
            }
            if (b1 >= 0 && b1 == a2) {
                val -= dot * inv_mass(colliders[b1]);
            }
            if (a1 == b2) {
                val -= dot * inv_mass(colliders[a1]);
            }
            if (b1 >= 0 && b1 == b2) {
                val += dot * inv_mass(colliders[b1]);
            }

            A[i * N + j] = val;
        }
    }

    // naive Gaussian elimination
    for (int k = 0; k < N; ++k) {
        const auto diag = A[k * N + k];
        if (glm::abs(diag) < 1e-8f) {
            continue;
        }
        const auto inv_diag = 1.0f / diag;
        for (int col = k; col < N; ++col) {
            A[k * N + col] *= inv_diag;
        }
        b[k] *= inv_diag;

        for (int row = 0; row < N; ++row) {
            if (row == k) continue;
            const auto factor = A[row * N + k];
            for (int col = k; col < N; ++col) {
                A[row * N + col] -= factor * A[k * N + col];
            }
            b[row] -= factor * b[k];
        }
    }
    auto& j = b; // in reduced echelon form, b now stores j

    // apply impulses
    for (int i = 0; i < N; ++i) {
        const auto& c = contacts[i];
        const auto impulse = j[i] * c.normal;
        apply_impulse(colliders[c.a], -impulse);
        apply_impulse(colliders[c.b], impulse);
    }
}

void fix_positions(std::vector<collider>& colliders, const std::vector<contact>& contacts) {
    for (auto& c : contacts) {
        if (c.penetration <= 0) continue;
        const auto inv_a = inv_mass(colliders[c.a]);
        const auto inv_b = inv_mass(colliders[c.b]);
        const auto correction = c.penetration * 0.4f * c.normal / (inv_a + inv_b);
        colliders[c.a].pos -= inv_a * correction;
        colliders[c.b].pos += inv_b * correction;
    }
}

}

void simulation::step(float frame_dt)
{
    auto& colliders = d_colliders.data();

    const auto num_substeps = 20;
    const auto dt = frame_dt / num_substeps;

    for (int i = 0; i != num_substeps; ++i) {
        // 1. integrate positions
        for (auto& c : colliders) {
            if (std::holds_alternative<dynamic_body>(c.body)) {
                c.pos += std::get<dynamic_body>(c.body).vel * dt;
            }
        }
    
        // 2. generate contacts and handle attraction
        std::vector<contact> contacts;

        for (std::size_t i = 0; i < colliders.size(); ++i) {
            for (std::size_t j = i + 1; j < colliders.size(); ++j) {
                auto& ci = colliders[i];
                auto& cj = colliders[j];
                if (const auto col = collision_test(ci, cj)) {
                    if (std::holds_alternative<attractor_body>(ci.body) && std::holds_alternative<attractor_body>(cj.body)) {
                        // nothing to do, attractors don't affect each other
                    }
                    else if (std::holds_alternative<attractor_body>(ci.body)) {
                        const auto dist = glm::length(ci.pos - cj.pos);
                        const auto direction = -col->normal;
                        const auto strength = col->penetration * 20.0f;
                        const auto attraction = strength * strength;
                        auto& vel = std::get<dynamic_body>(cj.body).vel;
                        vel += direction * attraction * dt;
                        vel *= (1.0f - 0.2f * strength * dt);
                    }
                    else if (std::holds_alternative<attractor_body>(cj.body)) {
                        const auto dist = glm::length(ci.pos - cj.pos);
                        const auto direction = col->normal;
                        const auto strength = col->penetration * 20.0f;
                        const auto attraction = strength * strength;
                        auto& vel = std::get<dynamic_body>(ci.body).vel;
                        vel += direction * attraction * dt;
                        vel *= (1.0f - 0.2f * strength * dt);
                    }
                    else {
                        contacts.push_back({ i, j, col->normal, col->penetration });
                    }
                }
            }
        }
    
        // 3. solve collisions
        solve_contacts(colliders, contacts);
    
        // 4. positional correction
        fix_positions(colliders, contacts);
    
        // 5. time-step–dependent global damping
        const auto damping = std::exp(-1.1f * dt); 
        for (auto& c : colliders) {
            if (std::holds_alternative<dynamic_body>(c.body)) {
                auto& vel = std::get<dynamic_body>(c.body).vel;
                vel *= damping;
                if (glm::length(vel) < 0.01f) {
                    vel = glm::vec2{0, 0};
                }
            }
        }
    }
}

}