from omni.isaac.core.scenes.scene import Scene
from omni.isaac.franka.tasks import PickPlace
from omni.isaac.franka.controllers import PickPlaceController
from .base import Base

class IsaacBase(Base):
    '''Isaac Sim Base Universe
    '''
    def __init__(self) -> None:
        super().__init__()
        print("Using Isaac Base")
        return

    def setup_scene(self, scene: Scene):
        scene.add_default_ground_plane()
        return

    async def setup_post_load(self):
        # self._world.add_physics_callback("sim_step", callback_fn=self.print_cube_info) #callback names have to be unique
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return


class FrankaUniverse(Base):
    '''Franka Manipulator Robot Universe
    '''
    def __init__(self) -> None:
        super().__init__()
        return

    def setup_scene(self, scene: Scene):
        world = self.get_world()
        # We add the task to the world here
        world.add_task(PickPlace(name="awesome_task"))
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        # The world already called the setup_scene from the task so
        # we can retrieve the task objects
        # Each defined task in the robot extensions
        # has set_params and get_params to allow for changing tasks during
        # simulation, {"task_param_name": "value": [value], "modifiable": [True/ False]}
        task_params = self._world.get_task("awesome_task").get_params()
        self._franka = self._world.scene.get_object(task_params["robot_name"]["value"])
        self._cube_name = task_params["cube_name"]["value"]
        self._controller = PickPlaceController(
            name="pick_place_controller",
            gripper_dof_indices=self._franka.gripper.dof_indices,
            robot_articulation=self._franka,
        )
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        # await self._world.play_async()
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        self._controller.reset()
        # await self._world.play_async()
        return

    def physics_step(self, step_size):
        # Gets all the tasks observations
        current_observations = self._world.get_observations()
        actions = self._controller.forward(
            picking_position=current_observations[self._cube_name]["position"],
            placing_position=current_observations[self._cube_name]["target_position"],
            current_joint_positions=current_observations[self._franka.name]["joint_positions"],
        )
        self._franka.apply_action(actions)
        if self._controller.is_done():
            self._world.pause()
        return

    def world_cleanup(self):
        return