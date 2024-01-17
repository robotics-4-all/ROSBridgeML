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
def generate_py(ctx, generator, model_path):
    pycode = smauto_m2t(model_path)
    model = build_model(model_path)
    filepath = f"{model.rosSys.name}_bridge.py"
    with open(filepath, "w") as fp:
        fp.write(pycode)
        make_executable(filepath)
    print(f"[CLI] Generated Bridge: {filepath}")


def main():
    cli(prog_name="rosbridgeml")
