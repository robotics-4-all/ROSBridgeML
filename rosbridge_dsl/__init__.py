import os
from textx import language, metamodel_from_file

from .utils import get_mm

__version__ = "0.1.0.dev"


@language('rosbridge_dsl', '*.rbr')
def rosbridge_dsl_language():
    "rosbridge_dsl language"
    mm = get_mm()

    # Here if necessary register object processors or scope providers
    # http://textx.github.io/textX/stable/metamodel/#object-processors
    # http://textx.github.io/textX/stable/scoping/

    return mm
