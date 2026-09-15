#!/usr/bin/env python3
"""Command-line frontend for KEYA V4.3 configuration mode."""

import argparse
from pathlib import Path
import sys

import serial

from keya.controller import KeyaController
from keya.parameters import PARAMETERS, format_value, parse_parameter, prepare_writes
from keya.protocol import make_ky, read_ky


def parameter_argument(text):
    try:
        return parse_parameter(text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def print_snapshot(records, only=None):
    print(f"{'No':>4}  {'Name':<34} {'Param':>12} {'RAM':>12} {'ROM':>12}")
    print("─" * 80)
    indices = [only] if only is not None else PARAMETERS
    for index in indices:
        values = []
        for bank in (0, 1, 2):
            raw = records.get((index, bank))
            values.append("—" if raw is None else format_value(index, raw))
        print(f"{index:04d}  {PARAMETERS[index].name:<34} {values[0]:>12} {values[1]:>12} {values[2]:>12}")


def build_parser():
    parser = argparse.ArgumentParser(description="KEYA V4.3 RS-232 configuration utility")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial device (default: /dev/ttyUSB0)")
    parser.add_argument("--debug", action="store_true", help="print raw TX/RX bytes")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("dump", help="read all Param/RAM/ROM values")
    get = commands.add_parser("get", help="read one parameter")
    get.add_argument("parameter", type=parameter_argument)
    write = commands.add_parser("write", aliases=["set"], help="write one parameter to RAM")
    write.add_argument("parameter", type=parameter_argument)
    write.add_argument("value")
    write.add_argument("--advanced", "--force", action="store_true", dest="advanced")
    burn = commands.add_parser("burn", help="store the complete current RAM image in EEPROM")
    burn.add_argument("--yes", action="store_true")
    export = commands.add_parser("export", help="save a Windows-compatible .ky backup")
    export.add_argument("path", type=Path)
    export.add_argument("--bank", choices=("RAM", "ROM"), default="ROM")
    inspect = commands.add_parser("inspect", help="inspect a .ky backup without opening a serial port")
    inspect.add_argument("path", type=Path)
    load = commands.add_parser("load", help="load a .ky backup into RAM; does not burn")
    load.add_argument("path", type=Path)
    load.add_argument("--advanced", action="store_true")
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    try:
        if args.command == "inspect":
            values = read_ky(args.path.read_bytes())
            print_snapshot({(index, 1): raw for index, raw in values.items() if index in PARAMETERS})
            unknown = sorted(set(values) - set(PARAMETERS))
            if unknown:
                print("Unknown parameter indices:", ", ".join(f"{index:04d}" for index in unknown))
            return

        values = None
        if args.command in ("write", "set"):
            values = prepare_writes({args.parameter: args.value}, args.advanced)
        elif args.command == "load":
            file_values = read_ky(args.path.read_bytes())
            unknown = sorted(set(file_values) - set(PARAMETERS))
            if unknown:
                raise ValueError(".ky contains unsupported parameters: " +
                                 ", ".join(f"{index:04d}" for index in unknown))
            values = prepare_writes(
                {index: format_value(index, raw) for index, raw in file_values.items()
                 if not PARAMETERS[index].read_only},
                args.advanced,
            )
        elif args.command == "burn" and not args.yes:
            parser.error("burn requires --yes; it stores the complete current RAM image")
        elif args.command == "export" and args.path.exists():
            parser.error(f"backup already exists: {args.path}")

        print(
            "Connect after normal controller startup. The manufacturer warns that reading at power-on can reset parameters.\n"
            "Power-cycle the controller after this configuration session, even if it was read-only.",
            file=sys.stderr,
        )
        with KeyaController(args.port, debug=args.debug) as controller:
            snapshot = controller.connect()
            if args.command == "dump":
                print_snapshot(snapshot)
            elif args.command == "get":
                print_snapshot(snapshot, args.parameter)
            elif args.command in ("write", "set", "load"):
                snapshot = controller.apply_ram(values, displayed=snapshot)
                print_snapshot(snapshot)
                print("RAM readback verified. EEPROM was not changed.")
            elif args.command == "burn":
                snapshot = controller.burn(displayed=snapshot)
                print_snapshot(snapshot)
                print("ROM readback verified. Power-cycle, reconnect, and read once more to verify retention.")
            elif args.command == "export":
                bank = 1 if args.bank == "RAM" else 2
                with args.path.open("xb") as stream:
                    stream.write(make_ky(snapshot, bank))
                print(f"{args.bank} backup saved: {args.path}")
    except (ValueError, OSError, TimeoutError, serial.SerialException) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
