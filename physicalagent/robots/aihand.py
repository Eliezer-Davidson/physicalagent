"""AiHand — the HiWonder 6-DOF hand, as a component bolted to another robot.

This is deliberately NOT a RobotAdapter. The hand used here is mounted on the
TonyPi's arm, so it has no supervision story of its own: it cannot walk away, it
cannot fall over, and it is powered and e-stopped as part of the robot carrying
it. Making it a peer robot in the registry would mean inventing a second guard,
a second e-stop and a second safe state for one body — the fleet spec's own
argument for why AiHand is `tethered`. It therefore rides on its host adapter's
guard, e-stop and tier, and this class is only: clamp, encode, send.

NO FEEDBACK EXISTS. Not position, not load, not temperature (fleet spec: the
Feedback row for AiHand reads "none at all"). Every method here is write-only,
and there is no read side to add later without new hardware. Consequences that
shape this file:

  * the tier clamp is the only thing between the model and a stalled servo;
  * `rest()` commands an open pose rather than releasing torque, because the
    framing has no torque-disable and pretending otherwise would be a lie;
  * a send that fails RAISES. An earlier draft swallowed connection failures
    and degraded to no-hand silently, which is indistinguishable from success
    on a robot that cannot report its own state.
"""
from __future__ import annotations

from physicalagent.robots import aihand_codec as codec
from physicalagent.robots.aihand_link import AiHandLink
from physicalagent.robots.tonypi_servo import ServoTier


class AiHand:
    """Clamp, encode and send finger commands over one AiHandLink."""

    def __init__(self, link: AiHandLink, *, tier: ServoTier = ServoTier.LOCKED,
                 framing: str = codec.DEFAULT_FRAMING):
        self._link = link
        self._tier = tier
        # Validated eagerly: a typo'd framing must fail when the hand is built,
        # not on the first command in front of an audience.
        if framing not in codec.FRAMINGS:
            raise ValueError(
                f"unknown framing {framing!r}; choose from "
                f"{', '.join(sorted(codec.FRAMINGS))}")
        self._framing = framing

    @property
    def tier(self) -> ServoTier:
        return self._tier

    @property
    def framing(self) -> str:
        return self._framing

    @property
    def kind(self) -> str:
        return self._link.kind

    @property
    def framing_is_verified(self) -> bool:
        """Whether the wire framing in use has ever moved a real finger.

        False until a probe says otherwise — see aihand_codec.VERIFIED_FRAMING.
        The operator surface reports this so nobody reads an accepted command
        as a moved finger.
        """
        return codec.VERIFIED_FRAMING == self._framing

    def set_fingers(self, positions: dict[str, int],
                    move_time_ms: int | None = None) -> dict:
        """Move the named fingers. Unnamed fingers are left where they are.

        Names are resolved (and rejected) BEFORE anything is clamped or sent, so
        a single bad name cannot half-execute a multi-finger command.
        """
        if not positions:
            return {"fingers": {}, "sent": False}

        resolved = {codec.finger_id(name): value
                    for name, value in positions.items()}

        clamped: dict[int, int] = {}
        time_out = None
        for servo_id, value in resolved.items():
            pulse, t = codec.clamp_finger(value, self._tier, move_time_ms)
            clamped[servo_id] = pulse
            # One frame carries one move time; the tier floor is identical for
            # every servo in it, so taking the last is taking the only value.
            time_out = t

        self._link.send(codec.encode(clamped, time_out, framing=self._framing))
        by_name = {name: clamped[codec.finger_id(name)] for name in positions}
        return {"fingers": by_name, "move_time_ms": time_out,
                "framing": self._framing, "sent": True,
                "framing_verified": self.framing_is_verified}

    def rest(self, move_time_ms: int | None = None) -> dict:
        """The hand's safe pose: every finger commanded open.

        This is NOT a torque release. The framing has no torque-disable command,
        so the servos stay powered and holding — which is the honest behaviour
        to document, not to paper over. `lower_and_release` in the fleet spec is
        therefore only half-achievable on this hand today.
        """
        return self.set_fingers(
            {name: codec.PULSE_MIN for name in codec.FINGER_IDS},
            move_time_ms=move_time_ms)

    def close_fist(self, move_time_ms: int | None = None) -> dict:
        """Every finger commanded closed, clamped by tier."""
        return self.set_fingers(
            {name: codec.PULSE_MAX for name in codec.FINGER_IDS},
            move_time_ms=move_time_ms)

    def close(self) -> None:
        """Release the transport."""
        self._link.close()
