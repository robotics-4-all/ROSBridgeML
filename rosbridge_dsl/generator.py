import sys
from os import path, mkdir, getcwd

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

    @staticmethod
    def generate(model_fpath: str, gen_imports: bool = True,
                 out_dir: str = None):
        pass


class GeneratorROS2(Generator):
    bridge_tpl = jinja_env.get_template('ros2/bridge.tpl')
    srcgen_folder = path.join(getcwd(), 'bridge_gen')

    @staticmethod
    def generate(model_fpath: str, gen_imports: bool = True,
                 out_dir: str = None):
        pass


def _generator_ros_impl(metamodel, model, output_path, overwrite,
                        debug, **custom_args):
        # Some code that perform generation
    gen_imports = custom_args['gen_imports'] if 'gen_imports' in custom_args \
        else True
    GeneratorROS.generate(model._tx_filename, gen_imports=gen_imports)


def _generator_ros2_impl(metamodel, model, output_path, overwrite,
                         debug, **custom_args):
        # Some code that perform generation
    gen_imports = custom_args['gen_imports'] if 'gen_imports' in custom_args \
        else True
    GeneratorROS2.generate(model._tx_filename, gen_imports=gen_imports)


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

