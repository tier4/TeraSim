from __future__ import annotations

import time
from contextlib import contextmanager
from typing import Any, Iterator


def get_profile(ctx: dict | None) -> dict[str, Any] | None:
    if not isinstance(ctx, dict):
        return None
    profile = ctx.get("cosim_profile")
    return profile if isinstance(profile, dict) else None


def add_timing(ctx: dict | None, path: str, elapsed_s: float) -> None:
    profile = get_profile(ctx)
    if profile is None:
        return

    node: dict[str, Any] = profile
    parts = path.split(".")
    for part in parts[:-1]:
        child = node.get(part)
        if not isinstance(child, dict):
            child = {}
            node[part] = child
        node = child

    leaf = parts[-1]
    try:
        node[leaf] = float(node.get(leaf, 0.0)) + float(elapsed_s)
    except (TypeError, ValueError):
        node[leaf] = float(elapsed_s)


def set_value(ctx: dict | None, path: str, value: Any) -> None:
    profile = get_profile(ctx)
    if profile is None:
        return

    node: dict[str, Any] = profile
    parts = path.split(".")
    for part in parts[:-1]:
        child = node.get(part)
        if not isinstance(child, dict):
            child = {}
            node[part] = child
        node = child
    node[parts[-1]] = value


@contextmanager
def timed(ctx: dict | None, path: str) -> Iterator[None]:
    if get_profile(ctx) is None:
        yield
        return

    start = time.perf_counter()
    try:
        yield
    finally:
        add_timing(ctx, path, time.perf_counter() - start)
