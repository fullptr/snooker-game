#include "simulation.hpp"
#include "table.hpp"

namespace snooker {
namespace {

struct contact {
    int a;             // circle index
    int b;             // -1 for wall
    glm::vec2 normal;  // from A to B (or into circle for wall)
    float penetration; // overlap depth
};

std::vector<contact> generate_contacts(const std::vector<ball>& circles,
                                       float xmin, float ymin,
                                       float xmax, float ymax)
{
    std::vector<contact> contacts;
    const auto margin = 1e-4f;

    // circle to circle
    for (std::size_t i = 0; i < circles.size(); ++i) {
        for (std::size_t j = i + 1; j < circles.size(); ++j) {
            glm::vec2 delta = circles[j].pos - circles[i].pos;
            const auto dist2 = glm::dot(delta, delta);
            const auto r = circles[i].radius + circles[j].radius;
            if (dist2 < r * r) {
                const auto dist = std::sqrt(dist2);
                glm::vec2 n = (dist > 1e-6f) ? delta / dist : glm::vec2(1, 0);
                contacts.push_back({ (int)i, (int)j, n, r - dist });
            }
        }
    }

    // circle to wall
    for (std::size_t i = 0; i < circles.size(); ++i) {
        const auto& c = circles[i];
        if (c.pos.x - c.radius < xmin + margin)
            contacts.push_back({ (int)i, -1, glm::vec2(1, 0), xmin - (c.pos.x - c.radius) });
        if (c.pos.x + c.radius > xmax - margin)
            contacts.push_back({ (int)i, -1, glm::vec2(-1, 0), (c.pos.x + c.radius) - xmax });
        if (c.pos.y - c.radius < ymin + margin)
            contacts.push_back({ (int)i, -1, glm::vec2(0, 1), ymin - (c.pos.y - c.radius) });
        if (c.pos.y + c.radius > ymax - margin)
            contacts.push_back({ (int)i, -1, glm::vec2(0, -1), (c.pos.y + c.radius) - ymax });
    }

    return contacts;
}

void solve_contacts(std::vector<ball>& balls,
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

        const auto rv = (b1 >= 0 ? balls[b1].vel : glm::vec2(0)) - balls[a1].vel;
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
                val += glm::dot(normal_i, normal_j) * balls[a1].inv_mass();
            }
            if (b1 >= 0 && b1 == a2) {
                val -= glm::dot(normal_i, normal_j) * balls[b1].inv_mass();
            }
            if (a1 == b2) {
                val -= glm::dot(normal_i, normal_j) * balls[a1].inv_mass();
            }
            if (b1 >= 0 && b1 == b2) {
                val += glm::dot(normal_i, normal_j) * balls[b1].inv_mass();
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
    for (int i = 0; i < N; ++i) {
        const contact& c = contacts[i];
        if (c.b < 0) { // wall
            const auto vn = glm::dot(balls[c.a].vel, c.normal);
            const auto max_impulse = balls[c.a].mass * std::max(0.0f, -vn);
            j[i] = std::min(j[i], max_impulse);
        }
        j[i] = std::max(0.0f, j[i]);
    }

    // apply impulses
    for (int i = 0; i < N; ++i) {
        const auto& c = contacts[i];
        const auto impulse = j[i] * c.normal;
        balls[c.a].vel -= balls[c.a].inv_mass() * impulse;
        if (c.b >= 0)
            balls[c.b].vel += balls[c.b].inv_mass() * impulse;
    }
}

void fix_positions(std::vector<ball>& circles, const std::vector<contact>& contacts) {
    for (auto& c : contacts) {
        if (c.penetration <= 0) continue;

        if (c.b < 0) { // circle to wall
            // small positional correction
            const auto percent = 0.2f;
            circles[c.a].pos += c.normal * c.penetration * percent;

            // bounce along wall using restitution
            const auto vn = glm::dot(circles[c.a].vel, c.normal);
            if (vn < 0.0f) {
                const auto restitution = 0.8f;
                circles[c.a].vel -= (1.0f + restitution) * vn * c.normal;
            }
        } else { // circle to circle
            const auto inv_a = circles[c.a].inv_mass();
            const auto inv_b = circles[c.b].inv_mass();
            const auto correction = c.penetration * 0.4f * c.normal / (inv_a + inv_b);
            circles[c.a].pos -= inv_a * correction;
            circles[c.b].pos += inv_b * correction;
        }
    }
}

}

void step_simulation(std::vector<ball>& circles, float dt,
                      float xmin, float ymin, float xmax, float ymax)
{
    const auto num_substeps = 6;
    const auto sub_dt = dt / num_substeps;

    for (int i = 0; i != num_substeps; ++i) {
        // 1. integrate positions
        for (auto& c : circles) {
            c.pos += c.vel * sub_dt;
        }
    
        // 2. generate contacts
        auto contacts = generate_contacts(circles, xmin, ymin, xmax, ymax);
    
        // 3. solve collisions
        solve_contacts(circles, contacts);
    
        // 4. positional correction
        fix_positions(circles, contacts);
    
        // 5. time-step–dependent global damping
        const auto damping = std::exp(-1.5f * sub_dt); // 1.5 means ~77% velocity lost per second
        for (auto& c : circles) {
            c.vel *= damping;
            if (glm::length(c.vel) < 0.01f) {
                c.vel = glm::vec2{0, 0};
            }
        }
    }
}

}