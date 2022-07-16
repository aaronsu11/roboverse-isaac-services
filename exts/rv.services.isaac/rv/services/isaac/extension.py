import carb
import omni.ext
import omni.ui as ui
from omni.services.core import main

def ping():
    return "pong"
    
# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class IsaacServiceExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):

        frontend_port = carb.settings.get_settings().get_as_int("exts/omni.services.transport.server.http/port")
        print(f"[rv.services.isaac] Starting Isaac Sim Microservice at port {frontend_port}")

        main.register_endpoint("get", "/ping", ping)

        self._window = ui.Window("My Window", width=300, height=300)
        with self._window.frame:
            with ui.VStack():
                ui.Label("Some Label")

                def on_click():
                    print("clicked!")

                ui.Button("Click Me", clicked_fn=lambda: on_click())

    def on_shutdown(self):
        main.deregister_endpoint("get", "/ping")
        print("[rv.services.isaac] MyExtension shutdown")
