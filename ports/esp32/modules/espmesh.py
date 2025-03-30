# espmesh module for MicroPython on ESP32
# MIT license; Copyright (c) 2025 Richard Liu @richyliu

from _espmesh import *


class ESPMesh(ESPMeshBase):
    def __init__(self):
        super().__init__()
