import datetime
import multiprocessing as mp

from omegaconf import DictConfig, OmegaConf
import wandb
import hydra
import jax
import orbax.checkpoint
from flax.training import orbax_utils
from jax.scipy.special import logsumexp
import os

from core.environments.base import Env
from core.utils.utils import dict_to_array

FILE_PATH = os.path.dirname(__file__)

from core.environments.mujoco.FurutaPendulum import FurutaPendulum
from core.rule_based_policy.furuta_controller import SwingUpAndBalanceCtrl
import numpy as np
from core.sampling.sbi_rollout_sampler import SimRolloutSampler, RealRolloutSampler
from core.sampling.rollout import rollout
from sbx import DDPG, DQN, PPO, SAC, TD3, TQC, CrossQ
from stable_baselines3.common.vec_env import SubprocVecEnv

from core.environment_wrappers.domain_randomizer import DomainRandomizer
from core.models.basic.category import CategoryDistribution


@hydra.main(
    version_base=None,
    config_path=os.path.join(FILE_PATH, "../configs"),
    config_name="config",
)
def main(cfg: DictConfig):
    if cfg.gpu:
        os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false"
    else:
        os.environ["CUDA_VISIBLE_DEVICES"] = ""
        os.environ["XLA_FLAGS"] = "--xla_force_host_platform_device_count=16"

    # Seeding
    rng_key = jax.random.PRNGKey(cfg.experiment.seed)

    # Loading the simulator/environment
    param_names = list(cfg.experiment.task.nominal_params.keys())

    sim_env = Env.create_instance(
         **cfg.experiment.task.environment_cfg, param_names=param_names
    )

    param_support = sim_env.param_support
    obs_dim = sim_env.obs_dim
    action_dim = sim_env.action_dim
    ground_truth = sim_env.get_sim_ground_truth()

    from stable_baselines3.common.monitor import Monitor
    # Simulate rollouts for a batch of domain parameters
    env_sim_fns = [
        lambda: Monitor(
            Env.create_instance(**cfg.experiment.task.environment_cfg, param_names=param_names)
        )
        for _ in range(cfg.experiment.num_workers)
    ]

    prior = CategoryDistribution(
        min_vals=param_support[0],
        max_vals=param_support[1],
        num_bins=cfg.experiment.CategoryDistribution_cfg.num_bins,
        log_scale=cfg.experiment.CategoryDistribution_cfg.log_scale,
    )

    #sample_params = prior.sample(rng_key=rng_key, sample_shape=(5000,))
    vec_env = SubprocVecEnv(env_sim_fns, start_method='spawn')
    #vec_env = DomainRandomizer(vec_env)
    #vec_env._buffer = sample_params
    #vec_env._ring_idx = 0

    import wandb
    wandb.login(key="bf00d1b2c38b2a7d8c8cc6642f8fad28d5284fa7")
    hyperparams = {
        "policy": "MlpPolicy",
        "n_steps": 4096,
        "ent_coef": 0.01,
        "batch_size": 256,
        "learning_rate": 1e-4,
        "clip_range": 0.2,
        "verbose": 1
    }

    logger = wandb.init(
        project="PPO",
        config=hyperparams,
        sync_tensorboard=True,
        monitor_gym=True,
    )

    from wandb.integration.sb3 import WandbCallback
    wandb_callback = WandbCallback(log="all")

    model = PPO(
        hyperparams["policy"],
        vec_env,
        verbose=hyperparams["verbose"],
        n_steps=hyperparams["n_steps"],
        ent_coef=hyperparams["ent_coef"],
        batch_size=hyperparams["batch_size"],
        learning_rate=hyperparams["learning_rate"],
        clip_range=hyperparams["clip_range"],
        tensorboard_log=f"runs",
        seed=0
    )

    model.learn(total_timesteps=2000_0000, progress_bar=True, callback=wandb_callback)

    from stable_baselines3.common.evaluation import evaluate_policy

    mean_reward, std_reward = evaluate_policy(
        model, vec_env, n_eval_episodes=10, deterministic=True
    )
    print(f"Evaluation result: Mean reward = {mean_reward:.2f} +/- {std_reward:.2f}")

    model.save("ppo_furuta_pendulum")

    vec_env.close()

    env = FurutaPendulum(render=True, param_names=param_names)
    model = PPO.load("ppo_furuta_pendulum")

    model.get_env()

    obs, _ = env.reset(options=ground_truth)
    for _ in range(50000):
        action, _ = model.predict(obs, deterministic=True)

        print(action)

        obs, reward, terminated, truncated, info = env.step(action)

        # time.sleep(0.05)

        if terminated or truncated:
            obs, _ = env.reset()





if __name__ == '__main__':
#     # policy = SwingUpAndBalanceCtrl(env.observation_space, env.action_space, env)
#     # policy = SwingUpAndBalanceCtrl(env)
#     import flax.linen as nn
#     # policy_kwargs = {
#     #     "net_arch": [64, 64],
#     #     "activation_fn": nn.relu,
#     #     "log_std_init": 0.0,
#     # }
#     #from core.utils.wandb_callback import WandbCallback

    main()

