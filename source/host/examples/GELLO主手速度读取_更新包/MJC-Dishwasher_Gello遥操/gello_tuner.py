"""Launch the standalone Tk parameter tuner for the dual-arm Gello reader."""

from __future__ import annotations

import argparse
from pathlib import Path
import tkinter as tk

from gello import GelloConfig, load_config
from gello.tk_app import GelloTunerApp


DEFAULT_CONFIG = Path(__file__).with_name("gello_config.json")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--check-config",
        action="store_true",
        help="validate the configuration without opening a window",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = load_config(args.config) if args.config.exists() else GelloConfig()
    if args.check_config:
        config.validate()
        print(f"Config valid: {args.config}")
        return

    root = tk.Tk()
    GelloTunerApp(root, args.config, config)
    root.mainloop()


if __name__ == "__main__":
    main()
