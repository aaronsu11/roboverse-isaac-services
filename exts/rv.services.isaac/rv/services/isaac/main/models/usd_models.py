import pydantic


class RequestDataModel(pydantic.BaseModel):
    """ Create sphere input model
    """
    reference_stage: str
    new_stage_location: str

class ResponseModel(pydantic.BaseModel):
    """ Create sphere response model
    """ 
    output_file: str