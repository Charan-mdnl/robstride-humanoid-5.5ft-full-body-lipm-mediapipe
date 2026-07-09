// td3_agent.hpp
// Twin Delayed Deep Deterministic Policy Gradient (TD3)
// Fujimoto et al. 2018 — https://arxiv.org/abs/1802.09477
//
// Key differences from DDPG:
//   1. Twin critics  — take min(Q1, Q2) to reduce overestimation bias
//   2. Delayed actor — update actor every POLICY_DELAY critic updates
//   3. Target noise  — add clipped noise to target actions (smoothing)
#pragma once
#include <torch/torch.h>
#include <deque>
#include <random>
#include <tuple>

// ─── PER-MOTOR LIMITS (shared with motor_env via extern) ─────────────
// Defined in td3_train.cpp main() from CLI args
inline float TD3_KP_MAX     = 500.0f;
inline float TD3_KD_MAX     = 10.0f;
inline float TD3_TORQUE_MAX = 60.0f;

static constexpr int OBS_DIM = 4;   // [error, vel, torque, integral]
static constexpr int ACT_DIM = 3;   // [kp, kd, torque_ff] in [-1, 1]

// ─── ACTOR ───────────────────────────────────────────────────────────
// Deterministic policy: obs → action in [-1, 1]
struct TD3Actor : torch::nn::Module {
    torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc3{nullptr};

    TD3Actor() {
        fc1 = register_module("fc1", torch::nn::Linear(OBS_DIM, 256));
        fc2 = register_module("fc2", torch::nn::Linear(256, 256));
        fc3 = register_module("fc3", torch::nn::Linear(256, ACT_DIM));
    }

    torch::Tensor forward(torch::Tensor x) {
        x = torch::relu(fc1->forward(x));
        x = torch::relu(fc2->forward(x));
        return torch::tanh(fc3->forward(x));  // output in [-1, 1]
    }
};

// ─── CRITIC ──────────────────────────────────────────────────────────
// Q(obs, action) → scalar value
struct TD3Critic : torch::nn::Module {
    // Q1
    torch::nn::Linear q1_fc1{nullptr}, q1_fc2{nullptr}, q1_fc3{nullptr};
    // Q2
    torch::nn::Linear q2_fc1{nullptr}, q2_fc2{nullptr}, q2_fc3{nullptr};

    TD3Critic() {
        int in = OBS_DIM + ACT_DIM;
        q1_fc1 = register_module("q1_fc1", torch::nn::Linear(in, 256));
        q1_fc2 = register_module("q1_fc2", torch::nn::Linear(256, 256));
        q1_fc3 = register_module("q1_fc3", torch::nn::Linear(256, 1));

        q2_fc1 = register_module("q2_fc1", torch::nn::Linear(in, 256));
        q2_fc2 = register_module("q2_fc2", torch::nn::Linear(256, 256));
        q2_fc3 = register_module("q2_fc3", torch::nn::Linear(256, 1));
    }

    // returns both Q values
    std::pair<torch::Tensor, torch::Tensor>
    forward(torch::Tensor obs, torch::Tensor act) {
        auto x = torch::cat({obs, act}, /*dim=*/1);

        auto q1 = torch::relu(q1_fc1->forward(x));
        q1 = torch::relu(q1_fc2->forward(q1));
        q1 = q1_fc3->forward(q1);

        auto q2 = torch::relu(q2_fc1->forward(x));
        q2 = torch::relu(q2_fc2->forward(q2));
        q2 = q2_fc3->forward(q2);

        return {q1, q2};
    }

    // Q1 only — used for actor loss
    torch::Tensor Q1(torch::Tensor obs, torch::Tensor act) {
        auto x = torch::cat({obs, act}, 1);
        auto q = torch::relu(q1_fc1->forward(x));
        q = torch::relu(q1_fc2->forward(q));
        return q1_fc3->forward(q);
    }
};

// ─── REPLAY BUFFER ───────────────────────────────────────────────────
struct ReplayBuffer {
    struct Transition {
        torch::Tensor obs, action, next_obs;
        float reward;
        float done;
    };

    std::deque<Transition> buf;
    size_t max_size;
    std::mt19937 rng{std::random_device{}()};

    ReplayBuffer(size_t max_size = 100000) : max_size(max_size) {}

    void push(torch::Tensor obs, torch::Tensor action,
              torch::Tensor next_obs, float reward, float done) {
        if (buf.size() >= max_size) buf.pop_front();
        buf.push_back({obs, action, next_obs, reward, done});
    }

    // sample a random minibatch — returns [B, dim] tensors
    std::tuple<torch::Tensor, torch::Tensor, torch::Tensor,
               torch::Tensor, torch::Tensor>
    sample(int batch_size) {
        std::vector<int> idx(buf.size());
        std::iota(idx.begin(), idx.end(), 0);
        std::shuffle(idx.begin(), idx.end(), rng);

        std::vector<torch::Tensor> obs_v, act_v, next_v;
        std::vector<float> rew_v, done_v;

        for (int i = 0; i < batch_size; i++) {
            auto& t = buf[idx[i]];
            obs_v.push_back(t.obs);
            act_v.push_back(t.action);
            next_v.push_back(t.next_obs);
            rew_v.push_back(t.reward);
            done_v.push_back(t.done);
        }

        return {
            torch::stack(obs_v),
            torch::stack(act_v),
            torch::stack(next_v),
            torch::tensor(rew_v).unsqueeze(1),
            torch::tensor(done_v).unsqueeze(1)
        };
    }

    size_t size() const { return buf.size(); }
};

// ─── SOFT UPDATE ─────────────────────────────────────────────────────
// θ_target ← τ * θ + (1-τ) * θ_target
template<typename M>
void soft_update(M& target, M& source, float tau) {
    torch::NoGradGuard no_grad;
    auto tp = target->parameters();
    auto sp = source->parameters();
    for (size_t i = 0; i < tp.size(); i++)
        tp[i].data().copy_(tau * sp[i].data() + (1.0f - tau) * tp[i].data());
}
