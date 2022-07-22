import time
import asyncio

from omni.services.core import routers

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