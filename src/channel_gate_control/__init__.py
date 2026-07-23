from pydoover.docker import run_app

from .application import ChannelGateControlApplication


def main():
    """Run the application."""
    run_app(ChannelGateControlApplication())
