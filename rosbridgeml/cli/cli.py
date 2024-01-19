import click
import os
from rich import print, pretty

from rosbridgeml.generator import GeneratorROS2, GeneratorROS
from rosbridgeml.utils import build_model

pretty.install()


def make_executable(path):
    mode = os.stat(path).st_mode
    mode |= (mode & 0o444) >> 2  # copy R bits to X
    os.chmod(path, mode)


@click.group()
@click.pass_context
def cli(ctx):
    ctx.ensure_object(dict)


@cli.command("validate", help="Model Validation")
@click.pass_context
@click.argument("model_path")
def validate(ctx, model_path):
    model = build_model(model_path)
    print("[*] Model validation success!!")


@cli.command("gen", help="Generate in Python")
@click.pass_context
@click.argument("generator")
@click.argument("model_path")
@click.option('-o', '--out-dir', required=False, type=str,
              default="gen", show_default=True)
def generate_py(ctx, generator, model_path, out_dir):
    model, imports = build_model(model_path)
    if generator in ('ros2', 'ROS2', 'Ros2'):
        GeneratorROS2.generate(model, out_dir)
    # with open(filepath, "w") as fp:
    #     fp.write(pycode)
    #     make_executable(filepath)
    print(f"[CLI] Generated Bridge: {out_dir}/")


def main():
    cli(prog_name="rosbridgeml")
