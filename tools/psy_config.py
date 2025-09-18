"""Utilities for reading psyche host configuration files.

The helper centralises TOML parsing for ``provision`` scripts so that shell
helpers can rely on a single, well-tested implementation.  It provides both a
Python API and a minimal command line interface:

>>> from tools import psy_config
>>> cfg = psy_config.load_host_config(root="/opt/psyched", host="cerebellum")
>>> cfg.services[:2]
('ros', 'workspace')
>>> cfg.domain_id()
42

Run ``python -m tools.psy_config --help`` for CLI usage details.
"""

from __future__ import annotations

import argparse
import socket
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping

try:  # pragma: no cover - tomllib is available on Python >= 3.11.
    import tomllib  # type: ignore[attr-defined]
except ModuleNotFoundError:  # pragma: no cover
    import tomli as tomllib  # type: ignore[no-redef]

DEFAULT_DOMAIN_ID = 42
DEFAULT_RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"
DEFAULT_ROS_DISTRO = "jazzy"


class PsyConfigError(RuntimeError):
    """Raised when a host configuration file cannot be parsed."""


@dataclass(frozen=True, slots=True)
class HostConfig:
    """Parsed representation of a host configuration file."""

    root: Path
    host: str
    path: Path
    data: Mapping[str, Any]
    services: tuple[str, ...]

    def get(self, key: str, default: Any | None = None) -> Any | None:
        """Return a raw value from the underlying TOML data."""

        return self.data.get(key, default)

    def domain_id(self, default: int = DEFAULT_DOMAIN_ID) -> int:
        """Return the configured ROS domain ID."""

        return _coerce_int(self.data.get("domain_id"), default, key="domain_id")

    def rmw(self, default: str = DEFAULT_RMW_IMPLEMENTATION) -> str:
        """Return the configured RMW implementation identifier."""

        return _coerce_str(self.data.get("rmw"), default, key="rmw")

    def ros_distro(self, default: str = DEFAULT_ROS_DISTRO) -> str:
        """Return the configured ROS 2 distribution name."""

        return _coerce_str(self.data.get("ros_distro"), default, key="ros_distro")

    def ros_environment(self) -> dict[str, str]:
        """Return ROS-related environment variables as a mapping of strings."""

        return {
            "ROS_DISTRO": self.ros_distro(),
            "ROS_DOMAIN_ID": str(self.domain_id()),
            "RMW_IMPLEMENTATION": self.rmw(),
        }


def resolve_root(candidate: str | Path | None = None) -> Path:
    """Resolve the psyche root directory.

    If ``candidate`` is provided it takes precedence; otherwise the helper
    consults ``$PSY_ROOT`` (when set), falling back to the repository checkout
    or ``/opt/psyched`` as a last resort.
    """

    if candidate is not None:
        return Path(candidate).expanduser().resolve()

    env = _getenv("PSY_ROOT")
    if env:
        path = Path(env).expanduser().resolve()
        if (path / "provision" / "hosts").is_dir():
            return path

    script_root = Path(__file__).resolve().parent.parent
    if (script_root / "provision" / "hosts").is_dir():
        return script_root

    system_root = Path("/opt/psyched")
    if (system_root / "provision" / "hosts").is_dir():
        return system_root

    return script_root


def resolve_host_file(
    host: str | None = None,
    root: str | Path | None = None,
) -> Path:
    """Return the path to the host configuration file."""

    resolved_root = resolve_root(root)
    hostname = host or socket.gethostname()
    return resolved_root / "provision" / "hosts" / f"{hostname}.toml"


def load_host_config(
    path: str | Path | None = None,
    *,
    root: str | Path | None = None,
    host: str | None = None,
) -> HostConfig:
    """Load and parse a host configuration file."""

    host_path: Path
    host_name: str
    if path is not None:
        host_path = Path(path).expanduser().resolve()
        host_name = host or host_path.stem
        if root is None:
            try:
                derived_root = host_path.parents[2]
            except IndexError:  # pragma: no cover - defensive fallback
                derived_root = host_path.parent.parent
            root = derived_root
    else:
        host_path = resolve_host_file(host=host, root=root)
        host_name = host or host_path.stem

    resolved_root = resolve_root(root)

    data: dict[str, Any] = {}
    if host_path.is_file():
        try:
            data = tomllib.loads(host_path.read_text())
        except tomllib.TOMLDecodeError as exc:  # pragma: no cover - invalid TOML is rare
            raise PsyConfigError(f"Failed to parse {host_path}: {exc}") from exc
    services = _normalise_services(data.get("services"))
    return HostConfig(
        root=resolved_root,
        host=host_name,
        path=host_path,
        data=data,
        services=services,
    )


def _normalise_services(raw: Any) -> tuple[str, ...]:
    if raw is None:
        return tuple()
    if not isinstance(raw, Iterable) or isinstance(raw, (str, bytes, bytearray)):
        raise PsyConfigError("services must be an array of strings")
    services: list[str] = []
    for entry in raw:
        if not isinstance(entry, str):
            raise PsyConfigError("services entries must be strings")
        value = entry.strip()
        if value:
            services.append(value)
    return tuple(services)


def _coerce_int(value: Any, default: int, *, key: str) -> int:
    if value is None:
        return default
    if isinstance(value, bool):
        raise PsyConfigError(f"{key} cannot be boolean")
    if isinstance(value, int):
        return value
    try:
        return int(str(value), 10)
    except (TypeError, ValueError) as exc:  # pragma: no cover - defensive fallback
        raise PsyConfigError(f"{key} must be an integer") from exc


def _coerce_str(value: Any, default: str, *, key: str) -> str:
    candidate = default if value is None else value
    if isinstance(candidate, str):
        result = candidate.strip()
        if result:
            return result
        raise PsyConfigError(f"{key} cannot be empty")
    raise PsyConfigError(f"{key} must be a string")


def _getenv(name: str) -> str | None:
    from os import environ

    return environ.get(name)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", help="psyche installation root")
    parser.add_argument("--host", help="host name (defaults to current hostname)")
    parser.add_argument("--file", help="explicit path to a host TOML file")

    sub = parser.add_subparsers(dest="command", required=True)

    services = sub.add_parser("services", help="list configured services")
    services.add_argument(
        "--format",
        choices=("lines", "json"),
        default="lines",
        help="output format (default: lines)",
    )

    value = sub.add_parser("get", help="retrieve a configuration value")
    value.add_argument("key", help="configuration key to fetch")
    value.add_argument("--default", help="fallback value if the key is missing")

    return parser


def _run_services_command(config: HostConfig, *, fmt: str) -> int:
    if fmt == "json":
        import json

        print(json.dumps(list(config.services)))
    else:
        for service in config.services:
            print(service)
    return 0


def _run_get_command(config: HostConfig, *, key: str, default: str | None) -> int:
    if key == "domain_id":
        default_value = DEFAULT_DOMAIN_ID
        if default is not None:
            try:
                default_value = int(default, 10)
            except ValueError as exc:
                raise SystemExit(f"invalid integer default for domain_id: {default}") from exc
        value = config.domain_id(default=default_value)
        print(value)
        return 0

    if key == "rmw":
        fallback = default or DEFAULT_RMW_IMPLEMENTATION
        print(config.rmw(default=fallback))
        return 0

    if key == "ros_distro":
        fallback = default or DEFAULT_ROS_DISTRO
        print(config.ros_distro(default=fallback))
        return 0

    value = config.get(key, default=default)
    if value is None:
        return 0
    print(value)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    host_path = Path(args.file).expanduser().resolve() if args.file else None
    config = load_host_config(path=host_path, root=args.root, host=args.host)

    if args.command == "services":
        return _run_services_command(config, fmt=args.format)
    if args.command == "get":
        return _run_get_command(config, key=args.key, default=args.default)
    parser.error(f"Unknown command: {args.command}")
    return 2


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
