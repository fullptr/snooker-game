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

// Checks if two colliders are colliding, and returns the normal of the collision if the yare
auto collision_test(const collider& a, const collider& b) -> std::optional<collision_info>
{
    if (a.mass < 0 && b.mass < 0) return {}; // static bodies, so no collision

    return std::visit(overloaded{
        [&](const circle_shape& A, const circle_shape& B) -> std::optional<collision_info> {
            const auto delta = b.pos - a.pos;
            const auto dist = glm::length(delta);
            const auto r = A.radius + B.radius;
            if (dist < r) {
                const auto n = (dist > 1e-6f) ? delta / dist : glm::vec2(1, 0);
                return collision_info{n, r - dist};
            }
            return {};
        },
        [&](const circle_shape& A, const box_shape& B) -> std::optional<collision_info> {
            const auto half_extents = glm::vec2{B.width, B.height} / 2.0f;
            
            // closest point in the box to the circles centre
            const auto P = glm::clamp(a.pos, b.pos - half_extents, b.pos + half_extents);

            const auto delta = P - a.pos;
            const auto dist = glm::length(delta);
            if (dist < A.radius) { // collision
                if (a.pos != P) { // circle centre is outside the box (the clamp moved the centre)
                    const auto n = (dist > 1e-6f) ? delta / dist : glm::vec2(1, 0);
                    return collision_info{n, A.radius - dist};
                }
                assert_that(false, "TODO: make this collision happen");
            }
            return {};
        },
        [&](auto&&, auto&&) -> std::optional<collision_info> {
            assert_that(false, "unhandled collision type!");
            return {};
        }
    }, a.geometry, b.geometry);
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

        const auto rv = (b1 >= 0 ? colliders[b1].vel : glm::vec2(0)) - colliders[a1].vel;
        const auto rel_vel = glm::dot(rv, normal_i);

        // only apply restitution if moving into contact
        b[i] = (rel_vel < -1e-6f) ? -(1.0f + restitution) * rel_vel : 0.0f;

        // baumgarte positional bias
        b[i] += 0.2f * std::max(ci.penetration, 0.0f);

        // Fill constraint matrix
        for (int j = 0; j < N; ++j) {
            const auto& cj = contacts[j];
            const auto a2 = cj.a;
            const auto b2 = cj.b;
            const auto normal_j = cj.normal;

            auto val = 0.0f;
            if (a1 == a2) {
                val += glm::dot(normal_i, normal_j) * colliders[a1].inv_mass();
            }
            if (b1 >= 0 && b1 == a2) {
                val -= glm::dot(normal_i, normal_j) * colliders[b1].inv_mass();
            }
            if (a1 == b2) {
                val -= glm::dot(normal_i, normal_j) * colliders[a1].inv_mass();
            }
            if (b1 >= 0 && b1 == b2) {
                val += glm::dot(normal_i, normal_j) * colliders[b1].inv_mass();
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

    // clamp impulses to prevent negative push
    //for (int i = 0; i < N; ++i) {
    //    const contact& c = contacts[i];
    //    if (colliders[c.b].mass < 0) {
    //        const auto vn = glm::dot(colliders[c.a].vel, c.normal);
    //        const auto max_impulse = colliders[c.a].mass * std::max(0.0f, -vn);
    //        j[i] = std::min(j[i], max_impulse);
    //    }
    //    j[i] = std::max(0.0f, j[i]);
    //}

    // apply impulses
    for (int i = 0; i < N; ++i) {
        const auto& c = contacts[i];
        const auto impulse = j[i] * c.normal;
        colliders[c.a].vel -= colliders[c.a].inv_mass() * impulse;
        colliders[c.b].vel += colliders[c.b].inv_mass() * impulse;
    }
}

void fix_positions(std::vector<collider>& colliders, const std::vector<contact>& contacts) {
    for (auto& c : contacts) {
        if (c.penetration <= 0) continue;

        if (c.b < 0) { // circle to wall
            // small positional correction
            const auto percent = 0.2f;
            colliders[c.a].pos += c.normal * c.penetration * percent;

            // bounce along wall using restitution
            const auto vn = glm::dot(colliders[c.a].vel, c.normal);
            if (vn < 0.0f) {
                const auto restitution = 0.8f;
                colliders[c.a].vel -= (1.0f + restitution) * vn * c.normal;
            }
        } else { // circle to circle
            const auto inv_a = colliders[c.a].inv_mass();
            const auto inv_b = colliders[c.b].inv_mass();
            const auto correction = c.penetration * 0.4f * c.normal / (inv_a + inv_b);
            colliders[c.a].pos -= inv_a * correction;
            colliders[c.b].pos += inv_b * correction;
        }
    }
}

}

void step_simulation(std::vector<collider>& colliders, float dt)
{
    const auto num_substeps = 6;
    const auto sub_dt = dt / num_substeps;

    for (int i = 0; i != num_substeps; ++i) {
        // 1. integrate positions
        for (auto& c : colliders) {
            c.pos += c.vel * sub_dt;
        }
    
        // 2. generate contacts
        auto contacts = generate_contacts(colliders);
    
        // 3. solve collisions
        solve_contacts(colliders, contacts);
    
        // 4. positional correction
        fix_positions(colliders, contacts);
    
        // 5. time-step–dependent global damping
        const auto damping = std::exp(-1.5f * sub_dt); // 1.5 means ~77% velocity lost per second
        for (auto& c : colliders) {
            c.vel *= damping;
            if (glm::length(c.vel) < 0.01f) {
                c.vel = glm::vec2{0, 0};
            }
        }
    }
}

}