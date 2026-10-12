"""ROS graph dump + telemetry summarising. The service call is injected."""

from physicalagent.probe.ros import dump_graph, summarise_samples


def fake_call(responses):
    def _call(name, type_, request):
        if name not in responses:
            raise RuntimeError(f"no such service {name}")
        return responses[name]
    return _call


GRAPH = {
    "/rosapi/topics": {"topics": ["/scan", "/odom_raw", "/ros_robot_controller/battery"]},
    "/rosapi/nodes": {"nodes": ["/pug_control", "/rosbridge_websocket"]},
    "/rosapi/services": {"services": ["/ros_robot_controller/bus_servo/get_state"]},
}


class TestDumpGraph:
    def test_it_collects_topics(self):
        assert "/scan" in dump_graph(fake_call(GRAPH))["topics"]

    def test_it_collects_nodes(self):
        assert "/pug_control" in dump_graph(fake_call(GRAPH))["nodes"]

    def test_it_collects_services(self):
        assert len(dump_graph(fake_call(GRAPH))["services"]) == 1

    def test_output_is_sorted_for_stable_diffs(self):
        """Captures get diffed between sessions -- unstable ordering is noise."""
        assert dump_graph(fake_call(GRAPH))["topics"] == sorted(GRAPH["/rosapi/topics"]["topics"])

    def test_a_failing_service_is_recorded_not_fatal(self):
        """A non-ROS robot has no rosapi. The capture must still complete."""
        out = dump_graph(fake_call({}))
        assert out["topics"] == []
        assert "error" in out

    def test_partial_graph_keeps_what_it_got(self):
        out = dump_graph(fake_call({"/rosapi/topics": {"topics": ["/scan"]}}))
        assert out["topics"] == ["/scan"]
        assert out["nodes"] == []


class TestSummariseSamples:
    def test_it_counts_samples(self):
        assert summarise_samples({"batt": [1, 2, 3]})["batt"]["n"] == 3

    def test_it_reports_min_and_max(self):
        s = summarise_samples({"batt": [11100, 11500, 11200]})["batt"]
        assert (s["min"], s["max"]) == (11100, 11500)

    def test_it_reports_the_median(self):
        assert summarise_samples({"batt": [1, 100, 2]})["batt"]["median"] == 2

    def test_it_keeps_first_and_last_for_drift(self):
        """First vs last is drift; min vs max is jitter. Both, separately."""
        s = summarise_samples({"batt": [10, 99, 20]})["batt"]
        assert (s["first"], s["last"]) == (10, 20)

    def test_an_empty_topic_reports_n_zero_not_a_crash(self):
        assert summarise_samples({"batt": []})["batt"]["n"] == 0

    def test_non_numeric_samples_are_counted_but_not_averaged(self):
        s = summarise_samples({"mode": ["ALIVE", "PAUSED"]})["mode"]
        assert s["n"] == 2
        assert s["median"] is None
