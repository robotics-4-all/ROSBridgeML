# RBML

A Domain-specific Language for Bridging ROS and ROS2 robotics systems to the IoT and
Smart Environments systems.

# Installation

## Virtual Environment

Create virtual environment:

```sh
python -m venv myenv
source myenv/bin/activate
```

Install the language:

```sh
pip install .
```

## Docker

Build the docker image (`rbml-api`):

```sh
make docker
```

or

```sh
docker build . -t <NAME>
```