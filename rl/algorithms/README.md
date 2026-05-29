# RL Algorithms — Research Roadmap

This folder will contain algorithm implementations for training the
multi-UAV swarm policy.

## Planned Algorithms

| Algorithm | Type | Priority | Notes |
|-----------|------|----------|-------|
| **PPO** (Proximal Policy Optimization) | Single-agent | Phase 1 | Centralized critic, works well for continuous action spaces |
| **MAPPO** (Multi-Agent PPO) | Multi-agent | Phase 2 | Each UAV has its own policy; shared critic |
| **QMIX** | Cooperative MARL | Phase 3 | For fully decentralized execution with joint training |
| **IPPO** | Multi-agent | Optional | Independent PPO per agent — fast baseline |

## Why These Algorithms?

The swarm task is **cooperative, partially observable, and continuous-action**:
- Drones must coordinate without direct communication → MARL needed
- Observation is local (each UAV sees its own position + nearby crowd) → POMDP
- Actions are continuous position deltas → Policy gradient methods preferred

## Research Baseline

Before RL, document the **rule-based baseline** from `uav_swarm_controller.py`:
- Circular formation + POMDP attention (α=0.15)
- Coverage typically ~40–65% depending on scenario
- Use this as the lower bound for RL comparison

## Implementation Notes

- Use **Stable-Baselines3** (SB3) or **RLlib** for PPO implementation
- Use **EPyMARL** or **PyMARL2** for MAPPO/QMIX
- The `UAVSwarmEnv` in `rl/gym_wrapper/` is compatible with SB3's `VecEnv`

## File Structure (when implemented)

```
algorithms/
├── ppo/
│   ├── ppo_agent.py       # PPO actor-critic network
│   └── ppo_config.yaml    # Hyperparameters
├── mappo/
│   ├── mappo_agent.py     # Multi-agent PPO
│   └── mappo_config.yaml
└── baselines/
    └── rule_based.py      # Wrap existing circular patrol as "algorithm 0"
```

## Getting Started (future)

```powershell
# Install dependencies
pip install stable-baselines3 gymnasium numpy pyyaml torch

# Run PPO training (headless)
python -m rl.training.trainer --scenario downtown --headless --episodes 1000

# Evaluate trained model
python -m rl.algorithms.evaluate --model models/latest/ --scenario event
```
