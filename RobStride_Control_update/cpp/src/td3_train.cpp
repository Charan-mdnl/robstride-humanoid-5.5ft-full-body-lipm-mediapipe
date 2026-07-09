// td3_train.cpp
// Twin Delayed DDPG (TD3) — motor gain tuning on real hardware
//
// Usage:
//   sudo ./td3_train <motor_id> <motor_type> <torque_limit_Nm>
//                   [kp_max] [kd_max] [min_deg=-45] [max_deg=45]
//
// Key advantage over PPO:
//   Off-policy — reuses past experience via replay buffer.
//   Converges faster on real hardware where each step costs ~20ms.

#include "td3_agent.hpp"
#include "motor_env.hpp"
#include <torch/torch.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <cstdio>
#include <algorithm>
#include <numeric>

// ─── HYPERPARAMETERS ─────────────────────────────────────────────────
static constexpr int    MAX_STEPS       = 100000; // total environment steps
static constexpr int    WARMUP_STEPS    = 1000;   // random actions before training
static constexpr int    BATCH_SIZE      = 256;
static constexpr float  GAMMA           = 0.99f;
static constexpr float  TAU             = 0.005f; // soft update rate
static constexpr float  LR_ACTOR        = 1e-4f;
static constexpr float  LR_CRITIC       = 1e-3f;
static constexpr int    POLICY_DELAY    = 2;      // update actor every N critic steps
static constexpr float  EXPLORE_NOISE   = 0.1f;  // Gaussian exploration noise std
static constexpr float  TARGET_NOISE    = 0.2f;  // target policy smoothing noise
static constexpr float  NOISE_CLIP      = 0.5f;  // clip target noise
static constexpr int    LOG_INTERVAL    = 50;    // log every N steps

int main(int argc, char* argv[]) {

    if (argc < 4) {
        std::cerr << "Usage: sudo ./td3_train <motor_id> <motor_type> <torque_limit_Nm>"
                     " [kp_max] [kd_max] [min_deg=-45] [max_deg=45]\n";
        std::cerr << "  motor_type: rs00 | rs01 | rs02 | rs03 | rs04\n";
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

    float  kp_max  = (argc >= 5) ? static_cast<float>(std::atof(argv[4])) : lim.kp_max;
    float  kd_max  = (argc >= 6) ? static_cast<float>(std::atof(argv[5])) : lim.kd_max;
    double min_deg = (argc >= 7) ? std::atof(argv[6]) : -45.0;
    double max_deg = (argc >= 8) ? std::atof(argv[7]) :  45.0;

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

    // set global limits (used by motor_env decode)
    TD3_KP_MAX     = kp_max;
    TD3_KD_MAX     = kd_max;
    TD3_TORQUE_MAX = lim.torque_max;
    // also set the ppo_agent globals that motor_env.hpp reads
    KP_MAX     = kp_max;
    KD_MAX     = kd_max;
    TORQUE_MAX = lim.torque_max;

    std::cout << "motor_id=" << motor_id
              << "  type=" << motor_type
              << "  kp_max=" << KP_MAX << "/" << lim.kp_max
              << "  kd_max=" << KD_MAX << "/" << lim.kd_max
              << "  torque_limit=" << torque_limit << "/" << lim.torque_max << " Nm"
              << "  angle_range=[" << min_deg << ", " << max_deg << "] deg\n";

    // ── CAN + motor setup ─────────────────────────────────────────────
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

    // ── TD3 networks ─────────────────────────────────────────────────
    auto actor        = std::make_shared<TD3Actor>();
    auto actor_target = std::make_shared<TD3Actor>();
    auto critic        = std::make_shared<TD3Critic>();
    auto critic_target = std::make_shared<TD3Critic>();

    // initialise targets as copies of main networks
    auto copy_params = [](auto& dst, auto& src) {
        torch::NoGradGuard ng;
        auto dp = dst->parameters();
        auto sp = src->parameters();
        for (size_t i = 0; i < dp.size(); i++)
            dp[i].data().copy_(sp[i].data());
    };
    copy_params(actor_target, actor);
    copy_params(critic_target, critic);

    torch::optim::Adam actor_opt(actor->parameters(),
                                 torch::optim::AdamOptions(LR_ACTOR));
    torch::optim::Adam critic_opt(critic->parameters(),
                                  torch::optim::AdamOptions(LR_CRITIC));

    // ── Replay buffer + environment ───────────────────────────────────
    ReplayBuffer replay;
    MotorEnv env(can, motor, torque_limit, min_deg, max_deg);

    std::mt19937 rng(std::random_device{}());
    std::normal_distribution<float> explore_dist(0.0f, EXPLORE_NOISE);
    std::normal_distribution<float> target_dist(0.0f, TARGET_NOISE);

    // ── CSV log ───────────────────────────────────────────────────────
    std::ofstream log("td3_train_log.csv");
    log << "elapsed_s,iteration,target_deg,mean_reward,mean_error_deg,mean_kp,mean_kd,mean_torque,critic_loss,actor_loss\n";

    // ── gnuplot ───────────────────────────────────────────────────────
    FILE* gp = popen("gnuplot", "w");
    if (gp) {
        fprintf(gp, "set terminal qt size 1400,500 title 'TD3 Training'\n");
        fprintf(gp, "set datafile separator ','\n");
        fprintf(gp, "set style data linespoints\n");
        fprintf(gp, "set grid\n");
        fflush(gp);
    }

    // ── initial reset ─────────────────────────────────────────────────
    auto obs_arr = env.reset();
    auto obs = torch::tensor(
        std::vector<float>(obs_arr.begin(), obs_arr.end()),
        torch::dtype(torch::kFloat32));

    auto train_start = std::chrono::steady_clock::now();
    int critic_updates = 0;
    float episode_reward = 0.0f;
    float last_critic_loss = 0.0f;
    float last_actor_loss  = 0.0f;

    // ── main training loop ────────────────────────────────────────────
    for (int step = 0; step < MAX_STEPS; step++) {

        torch::Tensor action;

        if (step < WARMUP_STEPS) {
            // random actions during warmup
            action = torch::rand({ACT_DIM}) * 2.0f - 1.0f;
        } else {
            // deterministic action + exploration noise
            torch::NoGradGuard ng;
            action = actor->forward(obs.unsqueeze(0)).squeeze(0);
            for (int i = 0; i < ACT_DIM; i++)
                action[i] = torch::clamp(
                    action[i] + explore_dist(rng),
                    -1.0f, 1.0f);
        }

        // ── step environment ─────────────────────────────────────────
        auto result = env.step(action);

        auto next_obs = torch::tensor(
            std::vector<float>(result.obs.begin(), result.obs.end()),
            torch::dtype(torch::kFloat32));

        replay.push(obs, action, next_obs,
                    result.reward, result.done ? 1.0f : 0.0f);

        episode_reward += result.reward;
        obs = next_obs;

        if (result.done) {
            auto reset_arr = env.reset();
            obs = torch::tensor(
                std::vector<float>(reset_arr.begin(), reset_arr.end()),
                torch::dtype(torch::kFloat32));
            episode_reward = 0.0f;
        }

        // ── train if enough samples ───────────────────────────────────
        if ((int)replay.size() < BATCH_SIZE) continue;

        auto [b_obs, b_act, b_next, b_rew, b_done] = replay.sample(BATCH_SIZE);

        // ── critic update ─────────────────────────────────────────────
        torch::Tensor target_Q;
        {
            torch::NoGradGuard ng;

            // target action with clipped noise (target policy smoothing)
            auto next_act = actor_target->forward(b_next);
            auto noise = torch::clamp(
                torch::randn_like(next_act) * TARGET_NOISE,
                -NOISE_CLIP, NOISE_CLIP);
            next_act = torch::clamp(next_act + noise, -1.0f, 1.0f);

            // min of twin target Q values
            auto [tq1, tq2] = critic_target->forward(b_next, next_act);
            auto target_q_min = torch::min(tq1, tq2);

            target_Q = b_rew + GAMMA * (1.0f - b_done) * target_q_min;
        }

        auto [q1, q2] = critic->forward(b_obs, b_act);
        auto critic_loss = torch::nn::functional::mse_loss(q1, target_Q)
                         + torch::nn::functional::mse_loss(q2, target_Q);

        critic_opt.zero_grad();
        critic_loss.backward();
        critic_opt.step();

        last_critic_loss = critic_loss.item<float>();
        critic_updates++;

        // ── delayed actor update ──────────────────────────────────────
        if (critic_updates % POLICY_DELAY == 0) {
            auto actor_loss = -critic->Q1(b_obs, actor->forward(b_obs)).mean();

            actor_opt.zero_grad();
            actor_loss.backward();
            actor_opt.step();

            last_actor_loss = actor_loss.item<float>();

            // soft update target networks
            soft_update(actor_target,  actor,  TAU);
            soft_update(critic_target, critic, TAU);
        }

        // ── logging ───────────────────────────────────────────────────
        if (step % LOG_INTERVAL == 0 && step >= WARMUP_STEPS) {
            torch::NoGradGuard ng;
            auto det_act = actor->forward(obs.unsqueeze(0)).squeeze(0);

            float kp = (det_act[0].item<float>() + 1.0f) / 2.0f * KP_MAX;
            float kd = (det_act[1].item<float>() + 1.0f) / 2.0f * KD_MAX;
            float torque_ff = det_act[2].item<float>() * torque_limit;

            double actual_deg = motor.pos * 180.0 / M_PI;
            float  error_deg  = std::abs(env.target_deg - actual_deg);

            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - train_start).count();

            std::cout << "iter=" << step
                      << "  t=" << elapsed << "s"
                      << "  target=" << env.target_deg << " deg"
                      << "  reward=" << result.reward
                      << "  err=" << error_deg << " deg"
                      << "  kp=" << kp
                      << "  kd=" << kd
                      << "  torque=" << std::abs(torque_ff) << " Nm"
                      << "  Lc=" << last_critic_loss
                      << "  La=" << last_actor_loss << "\n";

            log << elapsed << "," << step << "," << env.target_deg
                << "," << result.reward << "," << error_deg
                << "," << kp << "," << kd << "," << std::abs(torque_ff)
                << "," << last_critic_loss << "," << last_actor_loss << "\n";
            log.flush();

            // gnuplot live update
            if (gp && step >= WARMUP_STEPS + LOG_INTERVAL * 2) {
                fprintf(gp,
                    "set multiplot layout 1,3\n"
                    "set title 'Mean Kp'\n"
                    "set xlabel 'time (s)'\n"
                    "set ylabel 'Kp'\n"
                    "plot 'td3_train_log.csv' using 1:6 with linespoints lc rgb 'blue' title 'kp'\n"
                    "set title 'Mean Kd'\n"
                    "set xlabel 'time (s)'\n"
                    "set ylabel 'Kd'\n"
                    "plot 'td3_train_log.csv' using 1:7 with linespoints lc rgb 'red' title 'kd'\n"
                    "set title 'Mean Error (deg)'\n"
                    "set xlabel 'time (s)'\n"
                    "set ylabel 'degrees'\n"
                    "plot 'td3_train_log.csv' using 1:5 with linespoints lc rgb 'green' title 'error'\n"
                    "unset multiplot\n");
                fflush(gp);
            }
        }
    }

    // ── save models ───────────────────────────────────────────────────
    torch::save(actor,  "td3_actor.pt");
    torch::save(critic, "td3_critic.pt");
    std::cout << "\nTraining complete. Saved td3_actor.pt and td3_critic.pt\n";

    // ── close gnuplot (keep window open with -persistent style) ──────
    if (gp) {
        fprintf(gp, "pause mouse close\n");
        pclose(gp);
    }

    // ── safe stop ─────────────────────────────────────────────────────
    send_position(can, motor.id, 0.0, 10.0, 1.0, motor_type, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    disable(can, motor.id);

    log.close();
    return 0;
}
