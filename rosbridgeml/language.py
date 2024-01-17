import os
from textx import language, metamodel_from_file

from .utils import get_mm


@language('rosbridge', '*.rbr')
def rosbridge_language():
    "rosbridgeml language"
    mm = get_mm()

    # Here if necessary register object processors or scope providers
    # http://textx.github.io/textX/stable/metamodel/#object-processors
    # http://textx.github.io/textX/stable/scoping/

    return mm
