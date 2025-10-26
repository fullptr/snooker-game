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
    int N = (int)contacts.size();
    if (N == 0) return;

    std::vector<float> invMass(balls.size());
    for (size_t i = 0; i < balls.size(); ++i)
        invMass[i] = (balls[i].mass > 0.0f) ? 1.0f / balls[i].mass : 0.0f;

    std::vector<float> A(N * N, 0.0f);
    std::vector<float> b(N, 0.0f);

    for (int i = 0; i < N; ++i) {
        const contact& ci = contacts[i];
        const auto body_a = ci.a;
        const auto body_b = ci.b;
        glm::vec2 n_i = ci.normal;

        glm::vec2 rv = (body_b >= 0 ? balls[body_b].vel : glm::vec2(0)) - balls[body_a].vel;
        const auto rel_vel = glm::dot(rv, n_i);

        // only apply restitution if moving into contact
        b[i] = (rel_vel < -1e-6f) ? -(1.0f + restitution) * rel_vel : 0.0f;

        // baumgarte positional bias
        b[i] += 0.2f * std::max(ci.penetration, 0.0f);

        // Fill constraint matrix
        for (int j = 0; j < N; ++j) {
            const contact& cj = contacts[j];
            int a2 = cj.a;
            int b2 = cj.b;
            glm::vec2 n_j = cj.normal;

            float val = 0.0f;
            if (body_a == a2) val += glm::dot(n_i, n_j) * invMass[body_a];
            if (body_b >= 0 && body_b == a2) val -= glm::dot(n_i, n_j) * invMass[body_b];
            if (body_a == b2) val -= glm::dot(n_i, n_j) * invMass[body_a];
            if (body_b >= 0 && body_b == b2) val += glm::dot(n_i, n_j) * invMass[body_b];

            A[i * N + j] = val;
        }
    }

    // naive Gaussian elimination
    std::vector<float> j(N, 0.0f);
    for (int k = 0; k < N; ++k) {
        float diag = A[k * N + k];
        if (glm::abs(diag) < 1e-8f) continue;
        float invDiag = 1.0f / diag;
        for (int col = k; col < N; ++col)
            A[k * N + col] *= invDiag;
        b[k] *= invDiag;

        for (int row = 0; row < N; ++row) {
            if (row == k) continue;
            float factor = A[row * N + k];
            for (int col = k; col < N; ++col)
                A[row * N + col] -= factor * A[k * N + col];
            b[row] -= factor * b[k];
        }
    }
    j = b;

    // clamp impulses to prevent negative push
    for (int i = 0; i < N; ++i) {
        const contact& c = contacts[i];
        if (c.b < 0) { // wall
            float vn = glm::dot(balls[c.a].vel, c.normal);
            float maxImpulse = balls[c.a].mass * std::max(0.0f, -vn);
            j[i] = std::min(j[i], maxImpulse);
        }
        j[i] = std::max(0.0f, j[i]);
    }

    // apply impulses
    for (int i = 0; i < N; ++i) {
        const contact& c = contacts[i];
        glm::vec2 impulse = j[i] * c.normal;
        balls[c.a].vel -= invMass[c.a] * impulse;
        if (c.b >= 0)
            balls[c.b].vel += invMass[c.b] * impulse;
    }
}

void fix_positions(std::vector<ball>& circles, const std::vector<contact>& contacts) {
    for (auto& c : contacts) {
        if (c.penetration <= 0) continue;

        if (c.b < 0) { // circle to wall
            // small positional correction
            const auto percent = 0.2f;
            circles[c.a].pos += c.normal * c.penetration * percent;

            // nounce along wall using restitution
            const auto vn = glm::dot(circles[c.a].vel, c.normal);
            if (vn < 0.0f) {
                const auto restitution = 0.8f;
                circles[c.a].vel -= (1.0f + restitution) * vn * c.normal;
            }
        } else { // circle to circle
            const auto invA = 1.0f / circles[c.a].mass;
            const auto invB = 1.0f / circles[c.b].mass;
            const auto totalInv = invA + invB;
            glm::vec2 correction = c.penetration * 0.4f * c.normal / totalInv;
            circles[c.a].pos -= invA * correction;
            circles[c.b].pos += invB * correction;
        }
    }
}

}

void step_simulation(std::vector<ball>& circles, float dt,
                      float xmin, float ymin, float xmax, float ymax)
{
    // 1. integrate positions
    for (auto& c : circles)
        c.pos += c.vel * dt;

    // 2. generate contacts
    auto contacts = generate_contacts(circles, xmin, ymin, xmax, ymax);

    // 3. solve collisions
    solve_contacts(circles, contacts);

    // 4. positional correction
    fix_positions(circles, contacts);

    // 5. time-step–dependent global damping
    const auto damping = std::exp(-1.5f * dt); // 1.5 means ~77% velocity lost per second
    for (auto& c : circles) {
        c.vel *= damping;
    }
}

}