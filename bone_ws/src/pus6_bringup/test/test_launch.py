import ast
from pathlib import Path
import pytest


def test_launch_file_syntax():
    launch_path = Path(__file__).resolve().parent.parent / "launch" / "pus6_system.launch.py"
    assert launch_path.exists()
    source = launch_path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    assert tree is not None

    config_path = Path(__file__).resolve().parent.parent / "config" / "system_params.yaml"
    assert config_path.exists()
