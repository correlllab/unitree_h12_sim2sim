# Unitree H12 Sim2Sim Project

A complete simulation-to-simulation pipeline for training and deploying the Unitree H12 humanoid robot, bridging Isaac Lab for training with MuJoCo for deployment.

## Project Structure

```
unitree_h12_sim2sim/
├── isaaclab/                          # Isaac Lab training environments and robot definitions
│   ├── source/
│   │   └── unitree_h12_sim2sim/       # Isaac Lab extension
│   │       ├── robots/
│   │       │   └── unitree_h12.py     # H12 robot definition and configuration
│   │       └── tasks/
│   │           └── manager_based/
│   │               └── unitree_h12_sim2sim/
│   │                   ├── unitree_h12_sim2sim_stand_cfg.py   # Standing task
│   │                   ├── unitree_h12_sim2sim_walk_cfg.py    # Walking task
│   │                   ├── agents/      # RL training configurations
│   │                   └── mdp/         # MDP components (observations, rewards, etc.)
│   └── scripts/
│       └── rsl_rl/                    # RSL-RL training scripts
│
├── wbc_agile_utils/                   # Training and evaluation utilities
│   ├── train.py                       # RL policy training script
│   ├── play.py                        # Run trained policies interactively
│   ├── eval.py                        # Evaluate trained policies
│   ├── export_IODescriptors.py        # Export robot I/O descriptors
│   └── wandb_sweep/                   # Weights & Biases hyperparameter sweep configs
│
├── mujoco/                            # MuJoCo deployment and evaluation
│   └── sim2mujoco_eval.py             # Evaluate policies in MuJoCo simulator
│
└── logs/                              # Training logs and checkpoints
    └── rsl_rl/
        └── unitree_h12_walk/          # Walking task training logs
```

## Components

### Isaac Lab (`isaaclab/`)
Contains the simulation environments and robot definitions for training:
- **Robot Definition** (`robots/unitree_h12.py`): Full H12 humanoid configuration with 27 DOF (12 leg + 15 upper body)
- **Training Environments**: 
  - `unitree_h12_sim2sim_stand_cfg.py`: Standing balance task
  - `unitree_h12_sim2sim_walk_cfg.py`: Locomotion task with full-body control
- **MDP Components** (`mdp/`): Reward functions, observations, curriculum, and event definitions

### WBC Agile Utils (`wbc_agile_utils/`)
Training and evaluation infrastructure:
- **train.py**: RSL-RL policy training with support for curriculum learning
- **play.py**: Interactive policy playback and visualization
- **eval.py**: Evaluate trained policies on various metrics
- **export_IODescriptors.py**: Export robot I/O descriptors for deployment

### MuJoCo (`mujoco/`)
Deployment and validation in MuJoCo:
- **sim2mujoco_eval.py**: Evaluate Isaac Lab trained policies in MuJoCo simulator

## Installation

1. Install Isaac Lab following the [official guide](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)

2. Install the extension in editable mode:
```bash
python -m pip install -e isaaclab/source/unitree_h12_sim2sim
```

3. Verify installation by listing available environments:
```bash
python isaaclab/scripts/list_envs.py
```

## Quick Start

### Training

Train a walking policy with RSL-RL:
```bash
cd wbc_agile_utils
python train.py --task Unitree-H12-Walk-v0 --num_envs 4096 --headless
```

Train a standing policy:
```bash
cd wbc_agile_utils
python train.py --task Unitree-H12-Stand-v0 --num_envs 4096 --headless
```

### Evaluation

Run a trained policy interactively:
```bash
cd wbc_agile_utils
python play.py --task Unitree-H12-Walk-v0 --checkpoint <path_to_checkpoint>
```

Evaluate policy performance:
```bash
cd wbc_agile_utils
python eval.py --task Unitree-H12-Walk-v0 --checkpoint <path_to_checkpoint>
```

### MuJoCo Deployment

Evaluate an Isaac Lab trained policy in MuJoCo:
```bash
cd mujoco
python sim2mujoco_eval.py --checkpoint <path_to_checkpoint>
```


## References

- [Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/)
- [RSL-RL Library](https://github.com/leggedrobotics/rsl_rl)
- [Unitree Robotics H12](https://www.unitree.com/)
- [WBC Agile Repository](https://github.com/nvidia-isaac/WBC-AGILE)
