// ppo_agent.hpp
#pragma once
#include <torch/torch.h>

// ─── PER-MOTOR LIMITS (set at runtime from CLI, defaults to rs03) ─────
// These are written once in main() before training starts.
inline float KP_MAX     = 500.0f;
inline float KD_MAX     = 10.0f;
inline float TORQUE_MAX = 60.0f;

// ─── NETWORK ─────────────────────────────────────────────────────────

struct ActorCritic : torch::nn::Module {

    // layers
    torch::nn::Linear fc1{nullptr};      // shared: 4  → 64
    torch::nn::Linear fc2{nullptr};      // shared: 64 → 64
    torch::nn::Linear actor{nullptr};    // actor:  64 → 3  (kp, kd, torque_ff)
    torch::nn::Linear critic{nullptr};   // critic: 64 → 1  (V(s))
    torch::Tensor     log_std;           // learned exploration noise

    // ── constructor: register all layers ─────────────────────────────
    ActorCritic() {
        // TODO: register fc1, fc2, actor, critic
        fc1 = register_module("fc1", torch::nn::Linear(4, 64));
        fc2 = register_module("fc2", torch::nn::Linear(64, 64));
        actor = register_module("actor", torch::nn::Linear(64, 3));
        critic = register_module("critic", torch::nn::Linear(64, 1));
        // TODO: register log_std as parameter → torch::zeros({3})
        log_std = register_parameter("log_std", torch::zeros({3}));
    }

  // ── forward: obs → (action_mean, std, value) ─────────────────────
std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> forward(torch::Tensor x)
{
    // Hidden layer 1
    x = torch::tanh(fc1->forward(x));

    // Hidden layer 2
    x = torch::tanh(fc2->forward(x));

    // Actor mean in [-1, 1]
    torch::Tensor mean =
        torch::tanh(actor->forward(x));

    // Standard deviation
    torch::Tensor std =
        torch::exp(log_std);

    // Critic value
    torch::Tensor value =
        critic->forward(x);

    return {mean, std, value};
}

    // ── sample: forward + add gaussian noise → action, log_prob ──────
    std::pair<torch::Tensor, torch::Tensor>
sample(torch::Tensor obs) {
    auto [mean, std, value] = forward(obs);

    // manual Normal sample: μ + σ * ε,  ε ~ N(0,1)
    auto action   = mean + std * torch::randn_like(mean);

    // log_prob of Normal: -0.5*((x-μ)/σ)² - log(σ) - 0.5*log(2π)
    auto log_prob = -0.5f * torch::pow((action - mean) / std, 2)
                  - torch::log(std)
                  - 0.5f * std::log(2.0f * M_PI);
    log_prob = log_prob.sum(-1);

    return {action, log_prob};
}

    // ── evaluate: used during PPO update ─────────────────────────────
    std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
evaluate(torch::Tensor obs, torch::Tensor action) {
    auto [mean, std, value] = forward(obs);

    // same log_prob formula
    auto log_prob = -0.5f * torch::pow((action - mean) / std, 2)
                  - torch::log(std)
                  - 0.5f * std::log(2.0f * M_PI);
    log_prob = log_prob.sum(-1);

    // entropy of Normal: 0.5 * log(2πe * σ²)
    auto entropy = (0.5f + 0.5f * std::log(2.0f * M_PI)
                  + torch::log(std)).sum(-1);

    return {log_prob, entropy, value};
}

    // ── decode: map [-1,1] action tensor → physical kp, kd, torque ───
   // remove the static constexpr
// static constexpr float TORQUE_MAX = 5.0f;   ← delete this

// replace with a runtime variable passed into decode()
void decode(torch::Tensor action,
            float& kp, float& kd, float& torque_ff,
            float torque_limit) {               // ← add this
    kp        = (action[0].item<float>() + 1.0f) / 2.0f * KP_MAX;
    kd        = (action[1].item<float>() + 1.0f) / 2.0f * KD_MAX;
    torque_ff =  action[2].item<float>() * torque_limit;  // ← use limit
}
};