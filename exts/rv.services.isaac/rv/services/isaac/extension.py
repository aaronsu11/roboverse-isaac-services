import carb
import asyncio
import omni.ext
import omni.ui as ui
from omni.services.core import main
from omni.isaac.core import World
from omni.isaac.core.scenes.scene import Scene

from rv.services.isaac.base_sample import BaseSample


def ping():
    return "pong"


class HelloWorld(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return

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
        print("[rv.services.isaac] MyExtension shutdown")
