from pydantic import BaseModel


class RequestModel(BaseModel):
    """ Create sphere input model
    """
    x_pos: float
    y_pos: float

class ResponseModel(BaseModel):
    """ Create sphere response model
    """ 
    status: int
    message: str