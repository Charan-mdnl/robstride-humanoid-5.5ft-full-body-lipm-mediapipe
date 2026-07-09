// motor_env.hpp
#pragma once
#include "../include/motor_control.hpp"
#include "ppo_agent.hpp"
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <thread>
#include <chrono>

// ─── OBSERVATION INDICES ─────────────────────────────────────────────
//  obs[0] = error_deg      / 720.0
//  obs[1] = vel_deg_s      / 2864.8
//  obs[2] = actual_torque  / 60.0
//  obs[3] = integral_error / 720.0

// ─── ENVIRONMENT ─────────────────────────────────────────────────────

struct MotorEnv {
    CanInterface&      can;
    Motor&             motor;
    double             min_deg;            // angle range for randomised targets
    double             max_deg;
    double             target_deg;         // current episode target (randomised each reset)
    float              torque_limit;       // CLI-supplied torque cap (Nm)
    int                max_steps;
    int                step_count     = 0;
    double             integral_error = 0.0;
    float              prev_kp        = 0.0f;  // action smoothing state
    float              prev_kd        = 0.0f;
    float              prev_torque_ff = 0.0f;
    std::vector<Motor> motors_vec;
    std::mt19937       rng;

    MotorEnv(CanInterface& can, Motor& motor,
             float  torque_limit = 5.0f,
             double min_deg      = -90.0,
             double max_deg      =  90.0,
             int    max_steps    = 200)
        : can(can), motor(motor),
          min_deg(min_deg),
          max_deg(max_deg),
          target_deg(0.0),
          torque_limit(torque_limit),
          max_steps(max_steps),
          motors_vec({motor}),
          rng(std::random_device{}())
    {}
    // ── reset: pick random target, home motor, return first obs ─────────
    std::array<float, 4> reset() {
        step_count     = 0;
        integral_error = 0.0;
        prev_kp        = 0.0f;
        prev_kd        = 0.0f;
        prev_torque_ff = 0.0f;

        // randomise target angle so the policy learns for all angles
        // when min_deg == max_deg, use a fixed angle
        if (min_deg == max_deg) {
            target_deg = min_deg;
        } else {
            std::uniform_real_distribution<double> dist(min_deg, max_deg);
            target_deg = dist(rng);
        }

        // send to zero with safe low gains
        send_position(can, motor.id, 0.0, 10.0, 1.0, motor.type, 0.0);

        // let the motor physically settle
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // read back and sync motor struct
        collect_status(can, motors_vec);
        motor = motors_vec[0];

        return get_obs();
    }

    // ── step result ───────────────────────────────────────────────────
    struct StepResult {
        std::array<float, 4> obs;
        float                reward;
        bool                 done;
    };

    // ── step: decode action → send → read → reward ────────────────────
    StepResult step(torch::Tensor action) {
    float kp, kd, torque_ff;
        // action is [3], no need to squeeze
        // decode [-1,1] action tensor → physical units
    kp        = (action[0].item<float>() + 1.0f) / 2.0f * KP_MAX;
    kd        = (action[1].item<float>() + 1.0f) / 2.0f * KD_MAX;
    torque_ff = ( action[2].item<float>() * torque_limit);  // ← runtime limit

    // clamp kp and kd to non-negative values
    kp = std::max(kp, 0.0f);
    kd = std::max(kd, 0.0f);
    // clamp uses torque_limit not TORQUE_MAX
    torque_ff = std::clamp(torque_ff, -torque_limit, torque_limit);

    // exponential low-pass filter — smooths action changes, reduces jitter
    // alpha=0.3: new action contributes 30%, previous 70%
    constexpr float alpha = 0.3f;
    kp        = alpha * kp        + (1.0f - alpha) * prev_kp;
    kd        = alpha * kd        + (1.0f - alpha) * prev_kd;
    torque_ff = alpha * torque_ff + (1.0f - alpha) * prev_torque_ff;
    prev_kp = kp; prev_kd = kd; prev_torque_ff = torque_ff;

        // convert target to radians and send
        double target_rad = target_deg * M_PI / 180.0;
        send_position(can, motor.id, target_rad, kp, kd, motor.type, torque_ff);

        // wait one control tick (matches your 20ms loop)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // read back motor state into persistent vector then sync
        collect_status(can, motors_vec);
        motor = motors_vec[0];

        step_count++;
        bool done = (step_count >= max_steps);

        return { get_obs(), compute_reward(kp, kd), done };
    }

    // ── get_obs: motor state → normalised float[4] ────────────────────
    std::array<float, 4> get_obs() {
        double actual_deg = motor.pos * 180.0 / M_PI;
        double vel_deg_s  = motor.filtered_vel * 180.0 / M_PI;
        double error_deg  = target_deg - actual_deg;

        // accumulate integral, clamp to avoid windup
        integral_error = std::clamp(
            integral_error + error_deg * 0.02,   // 0.02 = dt in seconds
            -720.0, 720.0
        );

        return {
            static_cast<float>(error_deg      / 720.0),
            static_cast<float>(vel_deg_s      / 2864.8),
            static_cast<float>(motor.torque   / 60.0),
            static_cast<float>(integral_error / 720.0)
        };
    }

    // ── compute_reward ────────────────────────────────────────────────
    // Cleaner TD3 reward that uses radians, velocity error, torque cost, and gain penalties.
  float compute_reward(float kp, float kd) {
    double target_rad = target_deg * M_PI / 180.0;
    double error_rad  = target_rad - motor.pos;
    double vel        = motor.filtered_vel;

    // 1. Position error — keep yours, it works
    float r_pos = -5.0f * static_cast<float>(error_rad * error_rad);

    // 2. Velocity penalty — only penalize velocity when already close
    //    far from target: motion is desirable, don't punish it
    //    near target: velocity should be zero (settling)
    float closeness = std::exp(-10.0f * static_cast<float>(std::abs(error_rad)));
    float r_vel = -2.0f * closeness * static_cast<float>(vel * vel);

    // 3. Torque cost — keep yours
    float r_torque = -0.05f * static_cast<float>(motor.torque * motor.torque);

    // 4. Gain penalty — ASYMMETRIC: penalize Kd collapse much harder than high Kp
    //    Kd minimum enforcement: if Kd < 1.5, apply heavy quadratic penalty
    float kd_floor  = 1.5f;
    float r_kd_safe = -5.0f  * std::pow(std::max(0.0f, kd_floor - kd), 2.0f);
    float r_kp_cost = -0.005f * kp * kp;   // mild — high Kp is okay if stable
    float r_kd_cost = -0.005f * kd * kd;   // mild on normal range

    // 5. Spike penalty — explicitly punish large errors regardless of mean
    float spike_threshold = 0.15f;  // ~8.6 degrees
    float r_spike = 0.0f;
    if (std::abs(error_rad) > spike_threshold) {
        float excess = static_cast<float>(std::abs(error_rad)) - spike_threshold;
        r_spike = -10.0f * excess * excess;
    }

    // 6. Smooth bonus — replace sparse +2/+5 with a continuous Gaussian
    //    peaks at 0 error, falls off smoothly — no incentive to overshoot
    float r_bonus = 4.0f * std::exp(
        -50.0f * static_cast<float>(error_rad * error_rad)
    );

    return r_pos + r_vel + r_torque + r_kd_safe + r_kp_cost + r_kd_cost
           + r_spike + r_bonus;
}
};
