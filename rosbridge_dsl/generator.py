import sys
from os import path, mkdir, getcwd, chmod

import textx
from textx import GeneratorDesc
import jinja2

from .utils import build_model

_THIS_DIR = path.abspath(path.dirname(__file__))


# Initialize template engine.
jinja_env = jinja2.Environment(
    loader=jinja2.FileSystemLoader(path.join(_THIS_DIR, 'templates')),
    trim_blocks=True,
    lstrip_blocks=True)


class Generator:
    @staticmethod
    def generate(model):
        raise NotImplementedError()


class GeneratorROS(Generator):
    bridge_tpl = jinja_env.get_template('ros/bridge.tpl')
    srcgen_folder = path.join(getcwd(), 'bridge_gen')
    python_version = 3

    @staticmethod
    def generate(model_fpath: str,
                 out_dir: str = None):
        # Create output folder
        if out_dir is None:
            out_dir = GeneratorROS.srcgen_folder
        else:
            out_dir = path.join(out_dir, 'bridge_gen')
        if not path.exists(out_dir):
            mkdir(out_dir)
        model, imports = build_model(model_fpath)
        gen_params = {
            'bridges': {
                'topic': {
                    'b2r': [],
                    'r2b': []
                },
                'rpc': []
            }
        }
        for br in model.bridges:
            if 'TopicBridgeB2R' in br.__class__.__name__:
                gen_params['bridges']['topic']['b2r'].append(br)
            elif 'TopicBridgeR2B' in br.__class__.__name__:
                gen_params['bridges']['topic']['r2b'].append(br)
            elif 'RPCBridge' in br.__class__.__name__:
                gen_params['bridges']['rpc'].append(br)
        print(gen_params)
        out_file = path.join(out_dir, "bridges_node.py")
        with open(out_file, 'w') as f:
            f.write(GeneratorROS.bridge_tpl.render(
                bridges=gen_params['bridges'],
                python_version=GeneratorROS.python_version))
        # Give execution permissions to the generated file
        chmod(out_file, 509)


class GeneratorROS2(Generator):
    bridge_tpl = jinja_env.get_template('ros2/bridge.tpl')
    srcgen_folder = path.join(getcwd(), 'bridge_gen')

    @staticmethod
    def generate(model_fpath: str, gen_imports: bool = True,
                 out_dir: str = None):
        raise NotImplementedError()


def _generator_ros_impl(metamodel, model, output_path, overwrite,
                        debug, **custom_args):
    # Some code that perform generation
    GeneratorROS.generate(model._tx_filename)


def _generator_ros2_impl(metamodel, model, output_path, overwrite,
                         debug, **custom_args):
    # Some code that perform generation
    GeneratorROS2.generate(model._tx_filename)


generator_ros = GeneratorDesc(
    language='rosbridge',
    target='ros',
    description='ROS-to-Broker communication bridges',
    generator=_generator_ros_impl)


generator_ros2 = GeneratorDesc(
    language='rosbridge',
    target='ros2',
    description='ROS2-to-Broker communication bridges',
    generator=_generator_ros2_impl)

