import os
from textx import language, metamodel_from_file

from .utils import get_mm


@language('rosbridge', '*.rbr')
def rosbridge_language():
    "rosbridgeml language"
    mm = get_mm()
    return mm
