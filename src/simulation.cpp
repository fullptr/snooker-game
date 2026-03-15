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

auto inv_inertia(const collider& c) -> float
{
    if (!std::holds_alternative<dynamic_body>(c.body)) return 0;
    return safe_inverse(std::get<dynamic_body>(c.body).moment_of_inertia);
}

auto velocity(const collider& c) -> glm::vec2
{
    if (std::holds_alternative<dynamic_body>(c.body)) {
        return std::get<dynamic_body>(c.body).vel;
    }
    return {0.0f, 0.0f};
}

auto spin(const collider& c) -> glm::vec2
{
    if (std::holds_alternative<dynamic_body>(c.body)) {
        return std::get<dynamic_body>(c.body).spin;
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

auto apply_spin_impulse(collider& c, glm::vec2 torque_impulse) -> void
{
    if (std::holds_alternative<dynamic_body>(c.body)) {
        auto& body = std::get<dynamic_body>(c.body);
        body.spin += torque_impulse * safe_inverse(body.moment_of_inertia);
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


// Applies cloth friction to a single ball for one substep.
// Distinguishes sliding (high friction, spin catching up to velocity) from
// rolling (low friction, pure deceleration once spin and velocity are matched).
void apply_cloth_friction(collider& c, float dt)
{
    if (!std::holds_alternative<dynamic_body>(c.body)) return;
    if (!std::holds_alternative<circle_shape>(c.shape)) return;

    auto& body         = std::get<dynamic_body>(c.body);
    const auto radius  = std::get<circle_shape>(c.shape).radius;
    const auto inv_m   = safe_inverse(body.mass);
    const auto inv_I   = safe_inverse(body.moment_of_inertia);

    // Velocity of the contact point on the cloth surface.
    // In 3D, contact is at (0,0,-r). With spin = (ωx, ωy, 0):
    //   v_contact = vel + spin × (0,0,-r) = vel + (-spin.y, spin.x) * r
    const auto v_slip = body.vel + glm::vec2{-body.spin.y, body.spin.x} * radius;
    const auto slip_speed = glm::length(v_slip);

    if (slip_speed > simulation::slip_threshold) {
        // --- Sliding ---
        // Apply a Coulomb friction impulse opposing the slip direction,
        // capped so it can't reverse the slip (p_stop) or exceed μ_k*m*g*dt (p_max).
        const auto n_slip = v_slip / slip_speed;

        // Effective inverse mass at the contact point for a tangential impulse:
        //   Δv_slip = P * (1/m + r²/I)  →  for I=2/5·m·r², this equals P * 7/(2m)
        const auto eff_inv_mass = inv_m + radius * radius * inv_I;
        const auto p_stop = slip_speed / eff_inv_mass;
        const auto p_max  = simulation::friction_sliding * body.mass * dt;
        const auto p      = std::min(p_stop, p_max);

        // Linear impulse: oppose slip
        body.vel -= p * inv_m * n_slip;

        // Spin impulse: torque from friction at contact = r_contact × F
        //   Δspin = p * r / I * (-n.y, n.x)
        body.spin += p * radius * inv_I * glm::vec2{-n_slip.y, n_slip.x};
    } else {
        // --- Rolling ---
        // Snap spin to the exact rolling value to avoid drift.
        body.spin = glm::vec2{-body.vel.y, body.vel.x} / radius;

        // Low rolling-resistance deceleration.
        const auto speed = glm::length(body.vel);
        if (speed > 1e-4f) {
            const auto decel = std::min(simulation::friction_rolling * dt, speed);
            body.vel  -= (decel / speed) * body.vel;
            body.spin  = glm::vec2{-body.vel.y, body.vel.x} / radius;
        } else {
            body.vel  = {0.0f, 0.0f};
            body.spin = {0.0f, 0.0f};
        }
    }
}

}

void simulation::step()
{
    auto& colliders = d_colliders.data();

    const auto dt = time_step / num_substeps;

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
    
        // 5. cloth friction (sliding or rolling per ball)
        for (auto& c : colliders) {
            apply_cloth_friction(c, dt);
        }
    }
}

}