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
    reqs_tpl = jinja_env.get_template('requirements.txt.tpl')
    srcgen_folder = path.join(getcwd(), 'gen')
    
    PY_DEPS = [
        ('commlib-py', '>=', '0.11.2'),
        ('ros-msg-transform', '>=', '0.1.0')
    ]

    @staticmethod
    def generate(model: object, out_dir: str = None):
        if not path.exists(out_dir):
            mkdir(out_dir)
        out_file = path.join(out_dir, f"{model.rosSys.name}_bridges.py")
        if model.rosSys.type != 'ROS':
            print('[ERROR] - Did not find any ROS System definition!')
            return
        GeneratorROS.report(model)
        code = GeneratorROS.bridge_tpl.render(model=model)
        for bridge in model.bridges:
            print(bridge.msgType.split('/')[1])
        with open(out_file, 'w') as f:
            f.write(code)
        # Give execution permissions to the generated file
        chmod(out_file, 509)
        GeneratorROS.gen_requirements(out_dir)
        
    @staticmethod
    def gen_requirements(out_dir):
        out_file = path.join(out_dir, "requirements.txt")
        contents = GeneratorROS.reqs_tpl.render(deps=GeneratorROS.PY_DEPS)
        with open(out_file, 'w') as f:
            f.write(contents)

    @staticmethod
    def report(model):
        print(f"[*] - ROS System: {model.rosSys.name}")
        for bridge in model.bridges:
            print(f'[*] - Bridge: Type={bridge.__class__.__name__},' + \
                  f' Direction={bridge.direction}, ROS_URI={bridge.rosURI},' + \
                  f' Broker_URI={bridge.brokerURI},' + \
                  f' Broker=<{model.broker.host}:{model.broker.port}>')
