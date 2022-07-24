import fastapi
import asyncio

from pxr import UsdGeom, Usd, Sdf

from omni import usd as _usd
from omni.services.core import exceptions, routers
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

