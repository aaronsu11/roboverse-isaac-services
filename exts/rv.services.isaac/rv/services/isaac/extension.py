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

# module
from .main.universes.isaac_base import IsaacBase, FrankaUniverse
from .main.services import isaac_service, usd_service, sample_service


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class IsaacServiceExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id: str):

        self._ext_id = ext_id
        # change this to change environment
        self._universe = FrankaUniverse()

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
        main.register_endpoint("get", "/load-universe", self.load_universe)
        main.register_endpoint("get", "/reset-universe", self.reset_universe)
        main.register_endpoint("get", "/clear-universe", self.clear_universe)
        main.register_endpoint("get", "/play-async", self.play)
        main.register_endpoint("get", "/pause-async", self.pause)
        # custom routes
        self.start_services()
        return 

    def start_services(self):
        main.register_router(sample_service.router, prefix="/sample", tags=["sample"]) # sample, remove in prod
        main.register_router(usd_service.router, prefix="/usd", tags=["usd"])
        main.register_router(isaac_service.router, prefix="/isaac", tags=["isaac"])
        return

    def on_shutdown(self):
        print("[rv.services.isaac] Shutting down IsaacServiceExtension")
        if self._universe._world is not None:
            self._universe._world_cleanup()

        main.deregister_endpoint("get", "/load-universe")
        main.deregister_endpoint("get", "/reset-universe")
        main.deregister_endpoint("get", "/clear-universe")
        main.deregister_endpoint("get", "/play-async")
        main.deregister_endpoint("get", "/pause-async")
        # shutdown custom routes
        self.shutdown_services()
        return

    def shutdown_services(self):
        main.deregister_router(sample_service.router, prefix="/sample")
        main.deregister_router(usd_service.router, prefix="/usd")
        main.deregister_router(isaac_service.router, prefix="/isaac")
        return

    # @property
    # def universe(self):
    #     return self._universe

    async def load_universe(self):
        await self._universe.load_world_async()
        await omni.kit.app.get_app().next_update_async()
        self._universe._world.add_stage_callback("stage_event", self.on_stage_event)
        self._universe._world.add_timeline_callback("timeline_event", self.on_timeline_event)
        return

    async def reset_universe(self):
        await self._universe.reset_async()
        await omni.kit.app.get_app().next_update_async()

    async def clear_universe(self):
        await self._universe.clear_async()
        await omni.kit.app.get_app().next_update_async()

    async def play(self):
        await self._universe._world.play_async()
        return

    async def pause(self):
        await self._universe._world.pause_async()
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
