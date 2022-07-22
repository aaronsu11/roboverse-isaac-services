# this file initializes the service endpoint and world
# library
import carb
import asyncio
# dependency
import omni.kit.app
import omni.ext
import omni.usd
import omni.timeline
import omni.ui as ui
from omni.services.core import main
from omni.isaac.core import World
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
# module
from .main.universes.base import Base
from .main.services import usd_service, sample_service, universe_service


class IsaacBase(Base):
    def __init__(self) -> None:
        super().__init__()
        print("Using Isaac Base")
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
    def on_startup(self, ext_id: str):

        self._ext_id = ext_id
        self._universe = IsaacBase()

        # debug
        frontend_port = carb.settings.get_settings().get_as_int("exts/omni.services.transport.server.http/port")
        print(f"[rv.services.isaac] Starting Isaac Sim Microservice at port {frontend_port}")

        # views
        self._window = ui.Window("Debug Window", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                ui.Label("Debug Button")

                def on_click():
                    print("clicked!")
                    self._on_load_world()

                ui.Button("Click Me", clicked_fn=lambda: on_click())

        # controllers
        self.start_services()
        return 

    def start_services(self):

        main.register_router(sample_service.router, prefix="/sample", tags=["sample"]) # sample, remove in prod
        main.register_router(usd_service.router, prefix="/usd", tags=["usd"])
        # main.register_router(universe_service.router, prefix="/universe", tags=["universe"])

        main.register_endpoint("get", "/initialize_universe", self.initialize_universe)
        main.register_endpoint("get", "/create_world_async", self._universe.load_world_async)
        main.register_endpoint("get", "/add_robot", self._universe.add_robot)
        return

    def on_shutdown(self):
        if self._universe._world is not None:
            self._universe._world_cleanup()
        self.shutdown_services()
        return

    def shutdown_services(self):

        main.deregister_router(sample_service.router, prefix="/sample")
        main.deregister_router(usd_service.router, prefix="/usd")
        
        main.deregister_endpoint("get", "/initialize_universe")
        main.deregister_endpoint("get", "/create_world_async")
        main.deregister_endpoint("get", "/add_robot")
        print("[rv.services.isaac] IsaacServiceExtension shutdown")
        return

    # @property
    # def universe(self):
    #     return self._universe

    async def initialize_universe(self):
        await self._universe.load_world_async()
        await omni.kit.app.get_app().next_update_async()
        self._universe._world.add_stage_callback("stage_event", self.on_stage_event)
        self._universe._world.add_timeline_callback("timeline_event", self.on_timeline_event)
        return

    def on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.CLOSED):
            if World.instance() is not None:
                self._universe._world_cleanup()
                self._universe._world.clear_instance()
        return

    def on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            pass
        return
