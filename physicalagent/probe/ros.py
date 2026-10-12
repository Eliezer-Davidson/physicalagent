"""ROS graph capture over rosbridge — generalises grab_everything.sh.

The original script used SSH, which needs credentials we will not have for
every robot. rosapi is reachable through the same WebSocket the adapter
already uses, so this works on any rosbridge robot with no shell access.
"""
from __future__ import annotations

import statistics

_GRAPH_SERVICES = (
    ("topics", "/rosapi/topics", "rosapi/Topics", "topics"),
    ("nodes", "/rosapi/nodes", "rosapi/Nodes", "nodes"),
    ("services", "/rosapi/services", "rosapi/Services", "services"),
)


def dump_graph(call_service) -> dict:
    """Topics, nodes and services, sorted.

    A robot without rosapi (or without ROS at all) records an error and an
    empty list rather than aborting -- a partial capture is still evidence.
    """
    out: dict = {}
    errors = []
    for key, service, type_, field in _GRAPH_SERVICES:
        try:
            response = call_service(service, type_, {}) or {}
            out[key] = sorted(response.get(field, []))
        except Exception as exc:
            out[key] = []
            errors.append(f"{service}: {exc}")
    if errors:
        out["error"] = "; ".join(errors)
    return out


def summarise_samples(readings: dict) -> dict:
    """Per-topic summary of a timed sample.

    Reports min/max (jitter) and first/last (drift) separately -- they are
    different faults and one statistic cannot distinguish them. Non-numeric
    samples are counted but not aggregated.
    """
    summary = {}
    for topic, values in readings.items():
        values = list(values)
        entry = {"n": len(values), "first": values[0] if values else None,
                 "last": values[-1] if values else None,
                 "min": None, "max": None, "median": None}
        numeric = [v for v in values if isinstance(v, (int, float))
                   and not isinstance(v, bool)]
        if numeric and len(numeric) == len(values):
            entry["min"] = min(numeric)
            entry["max"] = max(numeric)
            entry["median"] = statistics.median(numeric)
        summary[topic] = entry
    return summary
