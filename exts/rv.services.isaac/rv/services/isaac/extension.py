import carb
import asyncio
import omni.ext
import omni.ui as ui
from omni.services.core import main
from omni.isaac.core import World
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot

from rv.services.isaac.base_sample import BaseSample


def ping():
    return "pong"


class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

    def add_robot(self):
        world = self.get_world()
        # Use the find_nucleus_server instead of changing it every time
        # you configure a new server with /Isaac folder in it
        # assets_root_path = get_assets_root_path()
        assets_root_path = "omniverse://localhost/NVIDIA/Assets/Isaac/2022.1"
        if assets_root_path is None:
            # Use carb to log warnings, errors and infos in your application (shown on terminal)
            carb.log_error("Could not find nucleus server with /Isaac folder")
        asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        # This will create a new XFormPrim and point it to the usd file as a reference
        # Similar to how pointers work in memory
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Fancy_Robot")
        # Wrap the jetbot prim root under a Robot class and add it to the Scene
        # to use high level api to set/ get attributes as well as initializing
        # physics handles needed..etc.
        # Note: this call doesn't create the Jetbot in the stage window, it was already
        # created with the add_reference_to_stage
        jetbot_robot = world.scene.add(Robot(prim_path="/World/Fancy_Robot", name="fancy_robot"))
        # Note: before a reset is called, we can't access information related to an Articulation
        # because physics handles are not initialized yet. setup_post_load is called after
        # the first reset so we can do so there
        print("Num of degrees of freedom before first reset: " + str(jetbot_robot.num_dof)) # prints None

    def setup_scene(self, scene: Scene):
        scene.add_default_ground_plane()
        return

    async def setup_post_load(self):
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        return


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class IsaacServiceExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):

        frontend_port = carb.settings.get_settings().get_as_int("exts/omni.services.transport.server.http/port")
        print(f"[rv.services.isaac] Starting Isaac Sim Microservice at port {frontend_port}")

        self._sample = HelloWorld()

        main.register_endpoint("get", "/create_world_async", self._sample.load_world_async)
        main.register_endpoint("get", "/add_robot", self._sample.add_robot)

        self._window = ui.Window("My Window", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                ui.Label("Some Label")

                def on_click():
                    print("clicked!")
                    self._on_load_world()

                ui.Button("Click Me", clicked_fn=lambda: on_click())


    def _on_load_world(self):
        async def _on_load_world_async():
            await self._sample.load_world_async()
            await omni.kit.app.get_app().next_update_async()

        asyncio.ensure_future(_on_load_world_async())
        return


    def on_shutdown(self):
        main.deregister_endpoint("get", "/ping")
        main.deregister_endpoint("get", "/add_robot")
        print("[rv.services.isaac] MyExtension shutdown")
