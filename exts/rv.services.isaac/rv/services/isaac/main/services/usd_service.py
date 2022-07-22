import fastapi
import asyncio
import numpy as np

from pxr import UsdGeom, Usd, Sdf

from omni import usd as _usd
from omni.services.core import exceptions, routers
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

from ..models.usd_models import RequestDataModel, ResponseModel


router = routers.ServiceAPIRouter()

@router.post("/list-meshes")
async def list_meshes(usd_stage: str = fastapi.Body(..., embed=True)):

    success, error = await _usd.get_context().open_stage_async(usd_stage)
    if not success:
        raise exceptions.KitServicesBaseException(status_code=500, detail=f"Error opening stage: {error}")

    stage = _usd.get_context().get_stage()
    result = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            result[prim.GetName()] = str(prim.GetPrimPath())

    return result


@router.post("/create-sphere", response_model=ResponseModel)
async def create_sphere(req: RequestDataModel):
    stage = Usd.Stage.CreateNew(req.new_stage_location)

    root_layer = stage.GetRootLayer()
    work_layer = Sdf.Layer.FindOrOpen(req.reference_stage)
    root_layer.subLayerPaths.append(work_layer.identifier)

    UsdGeom.Sphere.Define(stage, '/world/sphere')

    root_layer.Save()

    return {"output_file": req.new_stage_location}

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
