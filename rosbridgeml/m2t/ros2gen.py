import sys
from os import path, mkdir, getcwd, chmod


import jinja2

from rosbridgeml.utils import build_model

_THIS_DIR = path.abspath(path.dirname(__file__))


# Initialize template engine.
jinja_env = jinja2.Environment(
    loader=jinja2.FileSystemLoader(path.join(_THIS_DIR, "..", 'templates')),
    trim_blocks=True,
    lstrip_blocks=True
)


class GeneratorROS2:
    bridge_tpl = jinja_env.get_template('ros2_bridge.tpl')
    srcgen_folder = path.join(getcwd(), 'gen')

    @staticmethod
    def generate(model_fpath: str, gen_imports: bool = True,
                 out_dir: str = None):
        if out_dir is None:
            out_dir = GeneratorROS2.srcgen_folder
        else:
            out_dir = path.join(out_dir, 'gen')
        if not path.exists(out_dir):
            mkdir(out_dir)
        model, imports = build_model(model_fpath)
        out_file = path.join(out_dir, "bridges_node.py")

        if model.rosSys.type != 'ROS2':
            print('[ERROR] - Did not found any ROS2 System definition!')
            return

        GeneratorROS2.report(model)

        with open(out_file, 'w') as f:
            f.write(GeneratorROS2.bridge_tpl.render(
                bridges=model.bridges))
        # Give execution permissions to the generated file
        chmod(out_file, 509)

    @staticmethod
    def report(model):
        print(f"[*] - ROS2 System: {model.rosSys.name}")
        for bridge in model.bridges:
            print(f'[*] - Bridge: Type={bridge.__class__.__name__},' + \
                  f' Direction={bridge.direction}, ROS_URI={bridge.rosURI},' + \
                  f' Broker_URI={bridge.brokerURI},' + \
                  f' Broker=<{bridge.broker.host}:{bridge.broker.port}>')
