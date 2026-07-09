// ppo_train.cpp
// Build:
//   g++ -std=c++17 \
//       -I../include \
//       -I./libtorch/include \
//       -I./libtorch/include/torch/csrc/api/include \
//       -L./libtorch/lib \
//       -Wl,-rpath,./libtorch/lib \
//       -ltorch -ltorch_cpu -lc10 \
//       -o ppo_train ppo_train.cpp

#include "ppo_agent.hpp"
#include "motor_env.hpp"
#include <torch/torch.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>
#include <cstdio>

// ─── HYPERPARAMETERS ─────────────────────────────────────────────────

static constexpr int    N_STEPS     = 1024;   // steps per rollout
static constexpr int    N_EPOCHS    = 10;     // update passes per rollout
static constexpr int    BATCH_SIZE  = 64;     // minibatch size
static constexpr int    MAX_ITER    = 200;    // total training iterations
static constexpr float  GAMMA       = 0.99f;  // discount factor
static constexpr float  LAMBDA      = 0.95f;  // GAE smoothing
static constexpr float  CLIP_EPS    = 0.2f;   // PPO clip range
static constexpr float  LR          = 1e-4f;  // Adam learning rate (reduced to stabilise)
static constexpr float  VALUE_COEF  = 0.5f;   // critic loss weight
static constexpr float  ENTROPY_COEF= 0.001f; // entropy bonus weight (reduced to reduce exploration)
static constexpr float  GRAD_CLIP   = 0.5f;   // gradient norm clip

// ─── ROLLOUT BUFFER ───────────────────────────────────────────────────

struct Buffer {
    std::vector<torch::Tensor> obs;        // [4]   each step
    std::vector<torch::Tensor> actions;    // [3]   each step
    std::vector<float>         rewards;
    std::vector<float>         values;
    std::vector<float>         log_probs;
    std::vector<float>         dones;

    void clear() {
        obs.clear(); actions.clear(); rewards.clear();
        values.clear(); log_probs.clear(); dones.clear();
    }
};

// ─── PHASE 1 — COLLECT ROLLOUT ────────────────────────────────────────
//
//   Run the current policy on the motor for N_STEPS.
//   Store everything needed for the update.

void collect_rollout(ActorCritic& net, MotorEnv& env, Buffer& buf) {

    buf.clear();

    // get starting observation — std::array<float,4> always produces a [4] tensor
    auto obs_arr = env.reset();
    torch::Tensor obs = torch::tensor(
        std::vector<float>(obs_arr.begin(), obs_arr.end()),
        torch::dtype(torch::kFloat32)
    );

    for (int t = 0; t < N_STEPS; t++) {

        torch::NoGradGuard no_grad;   // no gradient during collection

        // actor samples action + log_prob (add batch dimension)
        auto obs_batch = obs.unsqueeze(0);
        auto [action, log_prob] = net.sample(obs_batch);

        // critic estimates value of current state (add batch dimension)
        auto [mean, std, value] = net.forward(obs_batch);

        // squeeze batch dimension: [1, 3] -> [3]
        action = action.squeeze(0);

        // step the motor
        auto result = env.step(action);

        // store transition (keep as [4] and [3] without batch dimension)
        buf.obs.push_back(obs);
        buf.actions.push_back(action);
        buf.rewards.push_back(result.reward);
        buf.values.push_back(value.squeeze().item<float>());
        buf.log_probs.push_back(log_prob.squeeze().item<float>());
        buf.dones.push_back(result.done ? 1.0f : 0.0f);

        // advance obs (always [4] — same as initial obs)
        obs = torch::tensor(
            std::vector<float>(result.obs.begin(), result.obs.end()),
            torch::dtype(torch::kFloat32)
        );

        if (result.done) {
            // store result before using iterators — calling reset() twice
            // would create two adjacent temporaries whose begin/end span 8 elements
            auto reset_arr = env.reset();
            obs = torch::tensor(
                std::vector<float>(reset_arr.begin(), reset_arr.end()),
                torch::dtype(torch::kFloat32)
            );
        }
    }
}

// ─── PHASE 2 — COMPUTE ADVANTAGES (GAE) ──────────────────────────────
//
//   delta[t]  = r[t] + γ * V(s[t+1]) * (1 - done[t]) - V(s[t])
//   A[t]      = delta[t] + γλ * A[t+1] * (1 - done[t])
//   returns[t]= A[t] + V(s[t])

std::pair<torch::Tensor, torch::Tensor>
compute_advantages(ActorCritic& net, Buffer& buf,
                   torch::Tensor last_obs) {

    int T = buf.rewards.size();

    // bootstrap value of the state after the last step
    float last_value = 0.0f;
    {
        torch::NoGradGuard no_grad;
        auto [m, s, v] = net.forward(last_obs.unsqueeze(0));  // add batch dimension
        last_value = v.squeeze().item<float>();
    }

    std::vector<float> advantages(T, 0.0f);
    std::vector<float> returns(T,    0.0f);

    float gae = 0.0f;

    // walk backwards through the buffer
    for (int t = T - 1; t >= 0; t--) {
        float next_value = (t == T - 1) ? last_value : buf.values[t + 1];
        float not_done   = 1.0f - buf.dones[t];

        float delta   = buf.rewards[t]
                      + GAMMA * next_value * not_done
                      - buf.values[t];

        gae           = delta + GAMMA * LAMBDA * not_done * gae;
        advantages[t] = gae;
        returns[t]    = gae + buf.values[t];
    }

    auto adv_t = torch::tensor(advantages);
    auto ret_t = torch::tensor(returns);

    // normalise advantages — stabilises training
    adv_t = (adv_t - adv_t.mean()) / (adv_t.std() + 1e-8f);

    return {adv_t, ret_t};
}

// ─── PHASE 3 — PPO UPDATE ─────────────────────────────────────────────
//
//   For K epochs, shuffle the buffer into minibatches and update.
//
//   ratio      = exp(log_prob_new - log_prob_old)
//   L_actor    = -min(ratio * A, clip(ratio, 1-ε, 1+ε) * A)
//   L_critic   = (V_pred - returns)²
//   L_entropy  = -entropy
//   loss       = L_actor + 0.5 * L_critic + 0.01 * L_entropy

void ppo_update(ActorCritic& net,
                torch::optim::Adam& optimizer,
                Buffer& buf,
                torch::Tensor advantages,
                torch::Tensor returns) {

    int T = buf.rewards.size();

    // stack buffer into tensors  [T, dims]
    auto obs_t      = torch::stack(buf.obs);
    auto actions_t  = torch::stack(buf.actions);
    auto old_lp_t   = torch::tensor(buf.log_probs);

    for (int epoch = 0; epoch < N_EPOCHS; epoch++) {

        // shuffle indices each epoch
        auto indices = torch::randperm(T);

        for (int start = 0; start < T; start += BATCH_SIZE) {

            auto end = std::min(start + BATCH_SIZE, T);
            auto idx = indices.slice(0, start, end);

            // minibatch slices
            auto b_obs  = obs_t.index_select(0, idx);
            auto b_act  = actions_t.index_select(0, idx);
            auto b_old  = old_lp_t.index_select(0, idx);
            auto b_adv  = advantages.index_select(0, idx);
            auto b_ret  = returns.index_select(0, idx);

            // evaluate current policy on old (obs, action) pairs
            auto [new_log_prob, entropy, value] = net.evaluate(b_obs, b_act);

            // ── actor loss ──────────────────────────────────────────
            auto ratio  = torch::exp(new_log_prob - b_old);
            auto surr1  = ratio * b_adv;
            auto surr2  = torch::clamp(ratio, 1.0f - CLIP_EPS,
                                              1.0f + CLIP_EPS) * b_adv;
            auto L_actor = -torch::min(surr1, surr2).mean();

            // ── critic loss ─────────────────────────────────────────
            auto L_critic = torch::pow(value.squeeze() - b_ret, 2).mean();

            // ── entropy bonus ───────────────────────────────────────
            auto L_entropy = -entropy.mean();

            // ── total loss ──────────────────────────────────────────
            auto loss = L_actor
                      + VALUE_COEF   * L_critic
                      + ENTROPY_COEF * L_entropy;

            // ── backprop ────────────────────────────────────────────
            optimizer.zero_grad();
            loss.backward();
            torch::nn::utils::clip_grad_norm_(
                net.parameters(), GRAD_CLIP);
            optimizer.step();
        }
    }
}

// ─── MAIN LOOP ────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {

    if (argc < 4) {
        std::cerr << "Usage: sudo ./ppo_train <motor_id> <motor_type> <torque_limit_Nm>"
                     " [kp_max] [kd_max] [min_deg=-90] [max_deg=90]\n";
        std::cerr << "  motor_type: rs00 | rs01 | rs02 | rs03 | rs04\n";
        std::cerr << "  kp_max / kd_max default to the hardware max for the chosen type\n";
        return -1;
    }

    int         motor_id     = std::atoi(argv[1]);
    std::string motor_type   = argv[2];
    float       torque_limit = static_cast<float>(std::atof(argv[3]));

    // ── per-type hardware limits ──────────────────────────────────────
    struct MotorLimits { float kp_max, kd_max, torque_max; };
    auto get_limits = [](const std::string& t) -> MotorLimits {
        if (t == "rs03") return { 5000.0f, 100.0f,  60.0f };
        if (t == "rs04") return { 5000.0f, 100.0f, 120.0f };
        if (t == "rs00") return {  500.0f,   5.0f,  17.0f };
        if (t == "rs01") return {  500.0f,   5.0f,  17.0f };
        if (t == "rs02") return {  500.0f,   5.0f,  17.0f };
        return { 500.0f, 10.0f, 60.0f };
    };
    auto lim = get_limits(motor_type);

    // optional CLI overrides for kp_max / kd_max / angle range
    float  kp_max  = (argc >= 5) ? static_cast<float>(std::atof(argv[4])) : lim.kp_max;
    float  kd_max  = (argc >= 6) ? static_cast<float>(std::atof(argv[5])) : lim.kd_max;
    double min_deg = (argc >= 7) ? std::atof(argv[6]) : -90.0;
    double max_deg = (argc >= 8) ? std::atof(argv[7]) :  90.0;

    // validate
    if (torque_limit <= 0.0f || torque_limit > lim.torque_max) {
        std::cerr << "torque_limit must be in (0, " << lim.torque_max << "] Nm for " << motor_type << "\n";
        return -1;
    }
    if (kp_max <= 0.0f || kp_max > lim.kp_max) {
        std::cerr << "kp_max must be in (0, " << lim.kp_max << "] for " << motor_type << "\n";
        return -1;
    }
    if (kd_max <= 0.0f || kd_max > lim.kd_max) {
        std::cerr << "kd_max must be in (0, " << lim.kd_max << "] for " << motor_type << "\n";
        return -1;
    }
    if (min_deg > max_deg) {
        std::cerr << "min_deg must be <= max_deg\n";
        return -1;
    }

    // set global limits used by ActorCritic and MotorEnv
    KP_MAX     = kp_max;
    KD_MAX     = kd_max;
    TORQUE_MAX = lim.torque_max;

    std::cout << "motor_id=" << motor_id
              << "  type=" << motor_type
              << "  kp_max=" << KP_MAX << "/" << lim.kp_max
              << "  kd_max=" << KD_MAX << "/" << lim.kd_max
              << "  torque_limit=" << torque_limit << "/" << lim.torque_max << " Nm"
              << "  angle_range=[" << min_deg << ", " << max_deg << "] deg\n";

    // ── setup CAN + motor ─────────────────────────────────────────────
    CanInterface can;
    if (!can.init("can0") || !can.is_ready()) {
        std::cerr << "CAN init failed\n";
        return -1;
    }

    Motor motor;
    motor.id   = motor_id;
    motor.type = motor_type;

    send_enable(can, motor.id);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    send_mode(can, motor.id, ControlMode::MIT_MODE);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ── setup PPO ─────────────────────────────────────────────────────
    auto net = std::make_shared<ActorCritic>();
    torch::optim::Adam optimizer(net->parameters(),
                                 torch::optim::AdamOptions(LR));

    MotorEnv env(can, motor, torque_limit, min_deg, max_deg);

    // ── CSV log ───────────────────────────────────────────────────────
    std::ofstream log("ppo_train_log.csv");
    log << "elapsed_s,iteration,target_deg,mean_reward,mean_error_deg,mean_kp,mean_kd,mean_torque\n";

    // ── gnuplot live plot ─────────────────────────────────────────────
    FILE* gp = popen("gnuplot", "w");
    if (gp) {
        fprintf(gp, "set terminal qt size 1400,500 title 'PPO Training'\n");
        fprintf(gp, "set datafile separator ','\n");
        fprintf(gp, "set style data linespoints\n");
        fprintf(gp, "set grid\n");
        fflush(gp);
    }

    // ── training loop ─────────────────────────────────────────────────
    auto train_start = std::chrono::steady_clock::now();
    for (int iter = 0; iter < MAX_ITER; iter++) {

        Buffer buf;

        // phase 1 — collect
        collect_rollout(*net, env, buf);

        // get last obs for bootstrap
        auto last_arr = env.get_obs();
        auto last_obs = torch::tensor(
            std::vector<float>(last_arr.begin(), last_arr.end())
        );  // keep as [4], batch dimension added in compute_advantages

        // phase 2 — advantages
        auto [advantages, returns] =
            compute_advantages(*net, buf, last_obs);

        // phase 3 — update
        ppo_update(*net, optimizer, buf, advantages, returns);

        // ── logging ───────────────────────────────────────────────────
        float mean_reward = 0.0f;
        for (auto r : buf.rewards) mean_reward += r;
        mean_reward /= buf.rewards.size();

        // error_deg is obs[0] * 720
        float mean_error = 0.0f;
        for (auto& o : buf.obs)
            mean_error += std::abs(o[0].item<float>() * 720.0f);
        mean_error /= buf.obs.size();

        // decode mean kp, kd, torque from stored actions
        // action[0] in [-1,1] -> kp, action[1] -> kd, action[2] -> torque_ff
        float mean_kp = 0.0f, mean_kd = 0.0f, mean_torque = 0.0f;
        for (auto& a : buf.actions) {
            mean_kp     += (a[0].item<float>() + 1.0f) / 2.0f * KP_MAX;
            mean_kd     += (a[1].item<float>() + 1.0f) / 2.0f * KD_MAX;
            mean_torque += std::abs(a[2].item<float>() * torque_limit);
        }
        int N = static_cast<int>(buf.actions.size());
        mean_kp     /= N;
        mean_kd     /= N;
        mean_torque /= N;

        std::cout << "iter=" << iter
                  << "  t=" << std::chrono::duration<double>(std::chrono::steady_clock::now() - train_start).count() << "s"
                  << "  target=" << env.target_deg << " deg"
                  << "  reward=" << mean_reward
                  << "  err=" << mean_error << " deg"
                  << "  kp=" << mean_kp
                  << "  kd=" << mean_kd
                  << "  torque=" << mean_torque << " Nm\n";

        double elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - train_start).count();
        log << elapsed_s << "," << iter << "," << env.target_deg << "," << mean_reward << ","
            << mean_error << "," << mean_kp << "," << mean_kd << "," << mean_torque << "\n";
        log.flush();

        // ── gnuplot replot ────────────────────────────────────────────
        if (gp && iter >= 1) {
            fprintf(gp,
                "set multiplot layout 1,3\n"
                "set title 'Mean Kp'\n"
                "set xlabel 'time (s)'\n"
                "set ylabel 'Kp'\n"
                "plot 'ppo_train_log.csv' using 1:6 with linespoints lc rgb 'blue' title 'kp'\n"
                "set title 'Mean Kd'\n"
                "set xlabel 'time (s)'\n"
                "set ylabel 'Kd'\n"
                "plot 'ppo_train_log.csv' using 1:7 with linespoints lc rgb 'red' title 'kd'\n"
                "set title 'Mean Error (deg)'\n"
                "set xlabel 'time (s)'\n"
                "set ylabel 'degrees'\n"
                "plot 'ppo_train_log.csv' using 1:5 with linespoints lc rgb 'green' title 'error'\n"
                "unset multiplot\n"
            );
            fflush(gp);
        }
    }

    // ── save trained model ────────────────────────────────────────────
    torch::save(net, "ppo_motor.pt");
    std::cout << "Model saved to ppo_motor.pt\n";

    // ── close gnuplot (keep window open with -persistent style) ──────
    if (gp) {
        fprintf(gp, "pause mouse close\n");
        pclose(gp);
    }

    // ── disable motor ─────────────────────────────────────────────────
    disable(can, motor.id);
    log.close();
    return 0;
}