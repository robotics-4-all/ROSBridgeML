import sys
from os import path, mkdir, getcwd, chmod

import textx
from textx import GeneratorDesc
import jinja2

from rosbridgeml.utils import build_model

_THIS_DIR = path.abspath(path.dirname(__file__))


# Initialize template engine.
jinja_env = jinja2.Environment(
    loader=jinja2.FileSystemLoader(path.join(_THIS_DIR, "..", "templates")),
    trim_blocks=True,
    lstrip_blocks=True
)


class GeneratorROS:
    bridge_tpl = jinja_env.get_template('ros_bridge.tpl')
    srcgen_folder = path.join(getcwd(), 'bridge_gen')
    python_version = 3

    @staticmethod
    def generate(model_fpath: str,
                 out_dir: str = None):
        # Create output folder
        if out_dir is None:
            out_dir = GeneratorROS.srcgen_folder
        else:
            out_dir = path.join(out_dir, 'gen')
        if not path.exists(out_dir):
            mkdir(out_dir)
        model, imports = build_model(model_fpath)
        gen_params = {
            'bridges': {
                'topic': {
                    'b2r': [],
                    'r2b': []
                },
                'service': {
                    'b2r': [],
                    'r2b': []
                }
            }
        }
        for br in model.bridges:
            if 'TopicBridge' in br.__class__.__name__:
                gen_params['bridges']['topic']['b2r'].append(br)
            elif 'ServiceBridge' in br.__class__.__name__:
                gen_params['bridges']['service']['b2r'].append(br)
        out_file = path.join(out_dir, "bridges_node.py")
        with open(out_file, 'w') as f:
            f.write(GeneratorROS.bridge_tpl.render(
                bridges=gen_params['bridges'],
                python_version=GeneratorROS.python_version))
        # Give execution permissions to the generated file
        chmod(out_file, 509)