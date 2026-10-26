# Robot Reinforcement Learning: Learning Resources

Organized on October 20, 2026.

Target stack: **Isaac Lab + Newton/MuJoCo-Warp + RSL-RL PPO**, running natively on Windows.

This list collects all 13 resource links from our learning discussion. Check off a resource after working through the relevant material, not just opening the page.

## Where to start

Use two tracks in parallel:

- **Practical track:** PyTorch basics → regular MuJoCo → MuJoCo-Warp → Newton → Isaac Lab tasks.
- **Theory track:** RL concepts → policies and value functions → actor–critic methods → advantage estimation → PPO → evaluation and sim-to-real.

The strongest starting pair for our project is **NVIDIA's Newton course** for implementation and **Spinning Up** for RL/PPO theory. Refer back to the PyTorch and MuJoCo foundations whenever the code becomes unclear.

## 1. PyTorch foundations

- [ ] [PyTorch — Learn the Basics](https://docs.pytorch.org/tutorials/beginner/basics/intro.html)
  - Focus: tensor shapes, batching, CPU/GPU devices, neural networks, automatic differentiation, optimization, and saving/loading models.
  - Practice: build a small network that accepts a batch of observations and returns one action vector per observation.
  - Goal: explain the input/output dimensions and where each tensor lives before using a larger RL framework.

## 2. Regular MuJoCo fundamentals

- [ ] [MuJoCo — Python Documentation](https://mujoco.readthedocs.io/en/latest/python.html)
  - Focus: `MjModel` versus `MjData`, `qpos`, `qvel`, actuator inputs, simulation stepping, and state inspection.
  - Practice: load a one-joint model, apply PD control, and plot desired versus measured motion.
  - Goal: understand the simulation loop before adding learning or GPU batching.

## 3. Direct MuJoCo-Warp simulation

- [ ] [MuJoCo-Warp — Official Guide](https://mujoco.readthedocs.io/en/latest/mjwarp/)
  - Focus: model conversion, GPU-resident state, parallel worlds, supported features, and contact/constraint buffer capacity.
  - Goal: understand why MuJoCo-Warp is not a drop-in replacement for regular MuJoCo.

- [ ] [MuJoCo-Warp — Official Tutorial Notebook on GitHub](https://github.com/google-deepmind/mujoco_warp/blob/main/notebooks/tutorial.ipynb)
  - Focus: creating GPU models/data and manipulating batched simulation state.
  - Practice: simulate several copies of the same small model with different initial conditions or commands.
  - Goal: understand how each world's state and control are indexed independently.

### API reminder

| Operation | Regular MuJoCo | Direct MuJoCo-Warp |
| --- | --- | --- |
| Load XML | `mujoco.MjModel.from_xml_path(...)` | Same initial CPU loading step |
| Prepare model | Use the CPU `MjModel` | `mjw.put_model(cpu_model)` |
| Allocate state | `mujoco.MjData(cpu_model)` | `mjw.make_data(cpu_model, nworld=N)` |
| Advance physics | `mujoco.mj_step(model, data)` | `mjw.step(model, data)` |
| Joint positions | Typically a NumPy array shaped `(nq,)` | A Warp array shaped `(N, nq)` |

`N` is the number of parallel worlds. `mjw.make_data` takes the original CPU model, not the converted GPU model. Arrays and supported functions differ; copying GPU data back through NumPy every step can undermine throughput. Check the documentation and installed version rather than mechanically replacing module names.

## 4. Newton and our chosen simulation stack

- [ ] [NVIDIA — Accelerating Robot Learning With Isaac Lab and Newton](https://docs.nvidia.com/learning/physical-ai/getting-started-with-newton/latest/index.html)
  - **Primary hands-on course for this project.**
  - Focus: `ModelBuilder`, `Model`, `State`, `Control`, `Contacts`, solvers, simulation loops, robot control, CUDA graph capture, and an Isaac Lab RL pipeline.
  - The example uses a Franka arm rather than a quadruped, but the programming structure transfers.
  - Initially prioritize basic simulation, joint control, and training/playback. Cameras and deformable-object examples can wait.

### Keep the abstraction layers separate

- **Direct MuJoCo-Warp:** model/data/physics stepping.
- **Newton:** model builders, states, controls, and solvers such as `SolverMuJoCo`.
- **Isaac Lab:** robot assets, environments, observations, actions, rewards, resets, and training integration.
- **RSL-RL:** the learning algorithm implementation, including PPO.

We do not need to master CUDA or write custom Warp kernels before starting with Isaac Lab.

## 5. Building and evaluating Isaac Lab tasks

- [ ] [NVIDIA — Getting Started With Isaac Lab](https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-lab/latest/index.html)
  - Focus: task specification, observations, actions, rewards, resets, training, evaluation, and simulation-to-reality.
  - Goal: explain what the policy sees, what its outputs mean, and what counts as success before launching training.

- [ ] [IsaacLabTutorial — Official Companion GitHub Repository](https://github.com/isaac-sim/IsaacLabTutorial)
  - Use alongside the course to inspect actual task and agent configurations.
  - Focus on the state-based task first; camera-based policies add complexity and memory use.
  - Read example environment counts as workload-specific settings, not as a recommendation for our 8 GB GPU.

## 6. Reinforcement learning theory

- [ ] [Spinning Up — Key Concepts in RL](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html)
  - Focus: states, observations, actions, policies, trajectories, rewards, returns, and value functions.
  - Goal: distinguish simulation stepping, policy inference, experience collection, and learning updates.
  - Use this as a theory reference; its installation instructions are not our Isaac Lab setup instructions.

- [ ] [David Silver — Lecture 1: Introduction to Reinforcement Learning, YouTube](https://www.youtube.com/watch?v=2pWv7GOvuf0)
  - Original DeepMind lecture introducing the agent–environment loop and core RL ideas.
  - Useful for a structured theoretical foundation rather than a current framework coding walkthrough.

- [ ] [David Silver — Official Teaching Page and Full Course](https://www.davidsilver.uk/teaching/)
  - Includes lecture materials covering Markov decision processes, value functions, policy gradients, planning, and exploration.
  - Focus especially on the introductory concepts and policy-gradient material.
  - This is an older foundational course, not a modern PPO implementation tutorial.

## 7. PPO theory and the actual learner

- [ ] [Spinning Up — Proximal Policy Optimization](https://spinningup.openai.com/en/latest/algorithms/ppo.html)
  - Focus: policy updates, probability ratios, advantages, clipping, and why updates must be controlled.
  - Goal: explain what PPO changes in the neural network and why larger updates can damage a useful policy.

- [ ] [RSL-RL — PPO Implementation on GitHub](https://github.com/leggedrobotics/rsl_rl/blob/main/rsl_rl/algorithms/ppo.py)
  - Read after learning the basic PPO equations.
  - Trace experience collection, actor/critic outputs, action distributions, losses, and optimizer updates.
  - Follow the relevant rollout/storage code when studying return and advantage estimation.
  - Compare online code with our installed RSL-RL version before copying implementation details.

## 8. Additional video orientation

- [ ] [NVIDIA — Reinforcement Learning Fundamentals: Isaac Lab Office Hours, YouTube](https://www.youtube.com/watch?v=nxxzuDqJCzQ)
  - Official NVIDIA Omniverse video covering introductory RL concepts and Isaac Lab workflow/Q&A.
  - **Version warning:** published in 2025, before the 3.0 workflow discussed here. Learn the concepts, but do not copy its installation or API commands without checking current documentation.

## Practical milestones for our quadruped

- [ ] **One simulated joint:** apply PD control and inspect target versus measured position and velocity.
- [ ] **Several parallel copies:** understand batched tensor dimensions, per-world commands, and independent resets.
- [ ] **A small RL task:** define and explain observations, actions, rewards, termination, and success before training.
- [ ] **Our quadruped on flat ground:** validate frames, joint ordering/signs, mass, inertia, actuator gains/limits, and timing before learning.
- [ ] **Evaluation and deployment:** test multiple seeds, disturbances, and learned stops; compare exported-policy outputs with Python before hardware use.

## Skills checklist

- [ ] Python functions, classes, modules, configuration, and debugging.
- [ ] Linear algebra, derivatives/gradients, probability distributions, expectations, and basic optimization.
- [ ] PyTorch tensor shapes, batching, device placement, automatic differentiation, and model checkpoints.
- [ ] Rigid-body frames, joint conventions, contacts, actuator models, and physics/control time steps.
- [ ] RL task design: observations, actions, rewards, resets, termination versus time limits, and success metrics.
- [ ] Actor–critic methods, advantage estimation, PPO clipping, exploration, and update diagnostics.
- [ ] Reproducible experiments, held-out seeds, physical performance metrics, and regression tests—not reward alone.
- [ ] Sim-to-real: calibration, sensor availability/noise, delays, actuator behavior, randomization, and deployment timing.

## Version and setup cautions

- Our reference framework is an **Isaac Lab 3.0 checkout**. This does not mean the Newton or MuJoCo-Warp package version is also 3.0.
- Tutorials for Isaac Lab 2.x, Isaac Gym, and MJX/JAX may teach useful concepts but use different APIs and setup workflows.
- Documentation URLs containing `latest`, and GitHub URLs pointing at `main`, can change. Consult the installed source and matching documentation when reproducing an example.
- Do not replace the existing environment or follow every tutorial's install/sync commands blindly. Read first, then use a deliberate, version-controlled setup for experiments.
- A successful simulation or policy export does not by itself prove safe hardware operation.
