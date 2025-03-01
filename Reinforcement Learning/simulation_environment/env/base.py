import gymnasium as gym
from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, List, Optional, Union

import numpy as np
import mujoco
from mushroom_rl.utils.mujoco import MujocoViewer
import time
from core.environments.base import Env


class MujocoSimEnv(gym.Env, ABC, Env):
    """
    Base class for MuJoCo environments, inheriting from OpenAI Gym.
    Supports domain parameter randomization.
    """

    def __init__(
        self,
        model_path: str,
        dt: float,
        max_steps: int,
        param_names: List[str] | None = None,
        render: bool = False,
    ):
        """
        Initialize the simulator.

        :param model_path: Path to the Mujoco model file
        :param dt: Time step of the simulation
        :param max_steps: Max steps of simulation
        :param param_names: List of domain parameter names
        """
        super().__init__()
        self.model_path = model_path
        self.dt = dt
        self.max_steps = max_steps
        self.current_step = 0


        # Load default domain parameters
        self.domain_params = self.get_nominal_params()
        nominal_param_names = list(self.domain_params.keys())
        if param_names is None:
            self.param_names = nominal_param_names
        else:
            self.param_names = param_names
        assert all(name in nominal_param_names for name in self.param_names)

        with open(model_path, mode="r") as f:
            self.xml_file = f.read()

        # Create Mujoco model with default parameters
        xml_model = self._create_mujoco_model(self.xml_file)

        self.model = mujoco.MjModel.from_xml_string(xml_model) # type: ignore
        self.model.opt.timestep = self.dt
        self.data = mujoco.MjData(self.model)

        self.state = None
        self.pending_params =None
        self.state_space = self._define_state_space()
        self.observation_space = self._define_observation_space()
        self.action_space = self._define_action_space()

        self.obs_dim = self.observation_space.shape[0] if self.observation_space.shape else None
        self.action_dim = self.action_space.shape[0] if self.action_space.shape else None

        self.enable_render = render
        if self.enable_render:
            self.viewer = MujocoViewer(self.model, self.dt, start_paused=True)

    @property
    @abstractmethod
    def init_space(self) -> gym.spaces.Space:
        """Define the initial space for the simulator."""
        pass

    @abstractmethod
    def _define_state_space(self) -> gym.spaces.Space:
        """Define the observation space for the simulator."""
        pass

    @abstractmethod
    def _define_observation_space(self) -> gym.spaces.Space:
        """Define the observation space for the simulator."""
        pass

    @abstractmethod
    def _define_action_space(self) -> gym.spaces.Space:
        """Define the action space for the simulator."""
        raise NotImplementedError

    @abstractmethod
    def get_prior(self):
        """The prior of the assumed task."""
        raise NotImplementedError

    @abstractmethod
    def get_sim_ground_truth(self) -> Dict[str, Any]:
        raise NotImplementedError

    @abstractmethod
    def get_nominal_params(self) -> Dict[str, Any]:
        """Return the nominal domain parameters. To be implemented by subclasses."""
        raise NotImplementedError


    def reset(self, seed:Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)

        if options is not None and 'domain_params' in options:
            domain_params = options['domain_params']
            self._update_domain_params(domain_params)

        if self.pending_params is not None:
            self._update_domain_params(self.pending_params)
            self.pending_params = None

        """ Reset the environment to an initial state. """
        self.current_step = 0
        self.state = self.init_space.sample()
        self._set_mujoco_state(self.state)
        mujoco.mj_forward(self.model, self.data)

        return self._observe(), {}


    def _update_domain_params(self, params: np.ndarray):
        """ Update the domain parameters."""
        params_dict = self.params_to_dict(params)

        for k, v in params_dict.items():
            self.domain_params[k] = v

        xml_model = self._create_mujoco_model(
            xml_model=self.xml_file
        )
        self.model = mujoco.MjModel.from_xml_string(xml_model)  # type: ignore
        self.model.opt.timestep = self.dt
        self.data = mujoco.MjData(self.model)


    @abstractmethod
    def _observe(self):
        """ Return the observation of the current state. """
        raise NotImplementedError

    @abstractmethod
    def _set_mujoco_state(self, state):
        """ Set the MuJoCo simulation state. """
        raise NotImplementedError

    @abstractmethod
    def _get_mujoco_state(self):
        """ Get the current state from the MuJoCo simulation. """
        raise NotImplementedError


    def params_to_dict(self, params: np.ndarray) -> Dict[str, np.ndarray]:
        """Convert a parameter array to a dictionary with parameter names."""
        assert len(params) == len(self.param_names)
        return {key: params[i] for i, key in enumerate(self.param_names)}


    def _create_mujoco_model(
        self,
        xml_model: str
    ) -> str:
        """
        Adapt the XML model file with the parameters.

        :param xml_model: The XML model string
        :return: The adapted XML model
        """
        for key, val in self.domain_params.items():
            xml_model = xml_model.replace(f"[{key}]", str(val))
        return xml_model


    def _is_terminal(self, state):
        """ Check if the state is terminal. """
        state = state.astype(np.float32)
        state_oob = not self.state_space.contains(state)
        return state_oob

    def render(self):
        """ Render the simulation. """
        if self.enable_render:
            self.viewer.render(self.data, record=False)
            time.sleep(self.dt)
