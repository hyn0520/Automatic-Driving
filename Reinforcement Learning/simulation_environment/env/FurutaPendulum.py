import gymnasium as gym
from gymnasium import spaces
from typing import Any, Callable, Dict, List, Optional, Union
import numpy as np
from collections import OrderedDict

from sympy.physics.vector.printing import params

from core.environments.mujoco.base import MujocoSimEnv
from core.environments.base import register_env
import os
import mujoco
import time
from core.models.basic.uniform import UniformDistribution


# gravity_const=9.81,  # gravity [m/s**2]
# motor_resistance=8.4,  # motor resistance [Ohm]
# motor_back_emf=0.042,  # motor back-emf constant [V*s/rad]
# mass_rot_pole=0.095,  # rotary arm mass [kg]
# length_rot_pole=0.085,  # rotary arm length [m]
# damping_rot_pole=5e-6,  # rotary arm viscous damping [N*m*s/rad], original: 0.0015, identified: 5e-6
# mass_pend_pole=0.024,  # pendulum link mass [kg]
# length_pend_pole=0.129,  # pendulum link length [m]
# damping_pend_pole=1e-6,  # pendulum link viscous damping [N*m*s/rad], original: 0.0005, identified: 1e-6
# voltage_thold_neg=0,  # min. voltage required to move the servo in negative the direction [V]
# voltage_thold_pos=0,  # min. voltage required to move the servo in positive the direction [V]

NOMINAL_PARAMS = OrderedDict(
    gravity_const=[  # gravity [m/s**2]
        9.81,
        9.0,
        11,
    ],
    motor_resistance=[  # motor resistance [Ohm]
        8.4,
        7.0,
        9.0,
    ],
    motor_back_emf=[  # motor back-emf constant [V*s/rad]
        0.042,
        0.0,
        1.0,
    ],
    mass_rot_pole=[  # rotary arm mass [kg]
        0.095,
        0.080,
        0.100,
    ],
    length_rot_pole=[  # rotary arm length [m]
        0.085,
        0.080,
        0.090,
    ],
    damping_rot_pole=[  # rotary arm viscous damping [N*m*s/rad], original: 0.0015, identified: 5e-6
        5e-6,
        0.0,
        1e-3
    ],
    mass_pend_pole=[  # pendulum link mass [kg]
        0.024,
        0.020,
        0.030
    ],
    length_pend_pole=[  # pendulum link length [m]
        0.129,
        0.120,
        0.135
    ],
    damping_pend_pole=[  # pendulum link viscous damping [N*m*s/rad], original: 0.0005, identified: 1e-6
        1e-6,
        0.0,
        1e-3
    ],
    voltage_thold_neg=[  # min. voltage required to move the servo in negative the direction [V]
        0,
        0,
        0.1
    ],
    voltage_thold_pos=[  # min. voltage required to move the servo in negative the direction [V]
        0,
        0,
        0.1
    ],
)

@register_env
class FurutaPendulum(MujocoSimEnv):
    """
    Specific implementation of the Furuta Pendulum environment.
    """

    def __init__(
        self,
        model_path: str = os.path.join(os.path.dirname(__file__), "assets/FurutaPendulum_random.xml"),
        dt: float = 0.01,
        max_steps: int = 5000,
        param_names: List[str] | None = None,
        render: bool = False,
        state_des: Optional[np.ndarray] = None,
        Q: Optional[np.ndarray] = None,
        R: Optional[np.ndarray] = None,
    ):
        super().__init__(
            model_path=model_path,
            dt=dt,
            max_steps=max_steps,
            param_names=param_names,
            render=render,
        )

        # Set parameter ranges
        min_param_values = [NOMINAL_PARAMS[key][1] for key in self.param_names]
        max_param_values = [NOMINAL_PARAMS[key][2] for key in self.param_names]
        self.param_support = np.asarray([min_param_values, max_param_values])

        self.state_des = state_des if state_des is not None else np.array([0.0, np.pi, 0.0, 0.0])
        self.Q = Q if Q is not None else np.diag([1.0, 1.0, 2e-2, 5e-3])
        self.R = R if R is not None else np.diag([4e-3])


    @property
    def init_space(self) -> gym.spaces.Space:
        max_init_state = np.array([2.0, 1.0, 0.5, 0.5], dtype=np.float32) / 180 * np.pi  # [rad, rad, rad/s, rad/s]
        return spaces.Box(-max_init_state, max_init_state, dtype=np.float32)

    def _define_state_space(self)-> gym.spaces.Space:
        """
        Define the state limitation for the Furuta Pendulum..
        """
        max_state = np.array([115.0 / 180 * np.pi, 4 * np.pi, 20 * np.pi, 20 * np.pi], dtype=np.float32)
        return spaces.Box(-max_state, max_state, dtype=np.float32)

    def _define_observation_space(self) -> gym.spaces.Space:
        """
        Define the observation space for the Furuta Pendulum.
        Observations include angles and angular velocities.
        """
        max_obs = np.array([1.0, 1.0, 1.0, 1.0, 20 * np.pi, 20 * np.pi], dtype=np.float32)
        return spaces.Box(-max_obs, max_obs, dtype=np.float32)

    def _define_action_space(self) -> gym.spaces.Space:
        """
        Define the action space for the Furuta Pendulum.
        Actions are limited to a voltage range.
        """
        max_act = 3.5
        return spaces.Box(-max_act, max_act, dtype=np.float32)

    def get_prior(self):
        return UniformDistribution(min_val=self.param_support[0], max_val=self.param_support[1])

    def get_sim_ground_truth(self) -> Dict[str, Any]:
        return {key: NOMINAL_PARAMS[key][0] for key in self.param_names}

    def get_nominal_params(self) -> Dict[str, Any]:
        return {key: val[0] for key, val in NOMINAL_PARAMS.items()}


    def step(self, action: np.ndarray):
        """
        Perform one step in the environment.
        """
        motor_back_emf = self.domain_params["motor_back_emf"]
        motor_resistance = self.domain_params["motor_resistance"]

        if NOMINAL_PARAMS["voltage_thold_neg"][0] <= action <= NOMINAL_PARAMS["voltage_thold_pos"][0]:
            action = np.zeros_like(action)

        _, _, theta_dot, _ = self.state
        torque = (
                motor_back_emf * (float(action) - motor_back_emf * theta_dot) / motor_resistance
        )

        self.data.ctrl[:] = torque
        mujoco.mj_step(self.model, self.data)
        self.current_step += 1
        self.state = self._get_mujoco_state()

        if self.enable_render:
            self.viewer.render(self.data, record=False)
            time.sleep(self.dt)

        # Check termination
        terminated = self._is_terminal(self.state)
        truncated = self.current_step >= self.max_steps

        # Compute reward
        reward = self._compute_reward(self.state, action)

        return self._observe(), reward, terminated, truncated, {}

    def _observe(self):
        """ Return the observation of the current state. """
        return np.array([
            np.sin(self.state[0]),
            np.cos(self.state[0]),
            np.sin(self.state[1]),
            np.cos(self.state[1]),
            self.state[2],
            self.state[3]
        ])


    def _set_mujoco_state(self, state):
        """ Set the MuJoCo simulation state. """
        self.data.qpos[:] = state[:2]  # [theta, alpha]
        self.data.qvel[:] = state[2:]  # [theta_dot, alpha_dot]
        mujoco.mj_forward(self.model, self.data)

    def _get_mujoco_state(self):
        """ Get the current state from the MuJoCo simulation. """
        qpos = self.data.qpos[:]
        qvel = self.data.qvel[:]
        return np.concatenate([qpos, qvel])


    def _compute_reward(self, state, action) -> float:
        """
        Compute the reward for the current step.
        """
        state_error = state - self.state_des

        state_error[1] = np.fmod(state_error[1], 2 * np.pi)  # Map to [-2pi, 2pi]
        state_error[state_error > np.pi] -= 2 * np.pi
        state_error[state_error < -np.pi] += 2 * np.pi

        state_cost = state_error.T @ self.Q @ state_error
        action_cost = action.T @ self.R @ action
        return np.exp(-(state_cost + action_cost))

    def _create_mujoco_model(
            self,
            xml_model: str,
    ) -> str:
        length_pend_pole = self.domain_params["length_pend_pole"]
        length_rot_pole = self.domain_params["length_rot_pole"]
        xml_model = xml_model.replace("[0.13-length_pend_pole]", str(0.13 - length_pend_pole))
        xml_model = xml_model.replace("[0.0055+length_rot_pole]", str(0.0055 + length_rot_pole))

        return super()._create_mujoco_model(xml_model)