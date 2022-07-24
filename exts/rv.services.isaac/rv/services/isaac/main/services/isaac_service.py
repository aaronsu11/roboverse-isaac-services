import time
import carb
import asyncio
import numpy as np

from omni.services.core import routers
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.franka import Franka
# from ..models.usd_models import RequestDataModel, ResponseModel



router = routers.ServiceAPIRouter()

@router.get("/ping")
def ping():
    return "pong"

@router.get("/async-get")
async def async_get():
    await asyncio.sleep(3)
    return "success"

@router.get("/sync-get")
def sync_get():
    time.sleep(3)
    return "success"

@router.get("/add-ground")
async def add_ground():
    world = World.instance()
    world.scene.add_default_ground_plane()
    return "success"

@router.get("/add-cube")
async def add_cube():
    world = World.instance()
    fancy_cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/random_cube", # The prim path of the cube in the USD stage
            name="fancy_cube", # The unique name used to retrieve the object from the scene later on
            position=np.array([0, 0, 1.0]), # Using the current stage units which is in meters by default.
            size=np.array([0.5015, 0.5015, 0.5015]), # most arguments accept mainly numpy arrays.
            color=np.array([0, 0, 1.0]), # RGB channels, going from 0-1
        )
    )
    return "success"

@router.get("/inspect-cube")
async def inspect_cube():
    world = World.instance()
    cube = world.scene.get_object("fancy_cube")
    position, orientation = cube.get_world_pose()
    linear_velocity = cube.get_linear_velocity()
    # will be shown on terminal
    print("Cube position is : " + str(position))
    print("Cube's orientation is : " + str(orientation))
    print("Cube's linear velocity is : " + str(linear_velocity))
    return

@router.get("/add-robot")
def add_robot():
    world = World.instance()
    if World.instance() is None:
        print("World not initialized")
        return
    else:
        world.scene.add(Franka(prim_path="/World/Fancy_Franka", name="fancy_franka"))
        return
