"""Pure thread-safe lifecycle rules for arm Action execution and shutdown."""

from dataclasses import dataclass
import threading
import time
from typing import Callable, Dict, Hashable, Tuple


class ArmLifecycleError(RuntimeError):
    """Base error for lifecycle gate failures."""


class ShutdownRequestedError(ArmLifecycleError):
    """A physical command was attempted after shutdown started."""


class PhysicalCommandOwnedError(ArmLifecycleError):
    """A different goal still owns a physical arm command."""


@dataclass(frozen=True)
class ArmLifecycleSnapshot:
    """Immutable counts used by shutdown diagnostics and unit tests."""

    shutdown_requested: bool
    admitted_count: int
    queued_count: int
    running_count: int
    physical_arms: Tuple[str, ...]


class ArmExecutionLifecycle:
    """Serialize shutdown against publications and account accepted callbacks.

    Goal admission, callback execution and physical-command ownership are
    deliberately separate. A dry-run can be admitted and running without ever
    becoming a physical owner.
    """

    def __init__(self) -> None:
        self._condition = threading.Condition(threading.RLock())
        self._shutdown_requested = False
        self._admitted: set[Hashable] = set()
        self._queued: set[Hashable] = set()
        self._running: set[Hashable] = set()
        self._physical_owners: Dict[str, Hashable] = {}

    def try_admit(self, request_token: Hashable) -> bool:
        """Reserve a pre-shutdown request until handle acceptance completes."""

        with self._condition:
            if self._shutdown_requested:
                return False
            self._admitted.add(request_token)
            self._condition.notify_all()
            return True

    def register_accepted(
        self, request_token: Hashable, goal_token: Hashable
    ) -> None:
        """Move an admitted request to the queued execute-callback set."""

        with self._condition:
            self._admitted.discard(request_token)
            self._queued.add(goal_token)
            self._condition.notify_all()

    def mark_running(self, goal_token: Hashable) -> None:
        """Move a queued callback to the running set."""

        with self._condition:
            self._queued.discard(goal_token)
            self._running.add(goal_token)
            self._condition.notify_all()

    def finish(self, goal_token: Hashable) -> None:
        """Remove a callback from lifecycle accounting, even after exceptions."""

        with self._condition:
            self._queued.discard(goal_token)
            self._running.discard(goal_token)
            self._condition.notify_all()

    def request_shutdown(self) -> bool:
        """Close admission/publication gates; return true only on first request."""

        with self._condition:
            first_request = not self._shutdown_requested
            self._shutdown_requested = True
            self._condition.notify_all()
            return first_request

    @property
    def shutdown_requested(self) -> bool:
        with self._condition:
            return self._shutdown_requested

    def require_running(self) -> None:
        """Fail promptly when an Action should stop normal processing."""

        with self._condition:
            if self._shutdown_requested:
                raise ShutdownRequestedError("arm motion server is shutting down")

    def publish_motion(
        self,
        arm: str,
        goal_token: Hashable,
        publisher: Callable[[], None],
    ) -> None:
        """Atomically gate and record a physical target publication.

        The publisher runs while holding the same lock used by
        :meth:`request_shutdown`. Therefore an old target either completes
        before shutdown is visible, or is rejected; it cannot pass the gate
        after the shutdown hold.
        """

        with self._condition:
            if self._shutdown_requested:
                raise ShutdownRequestedError("arm motion server is shutting down")
            owner = self._physical_owners.get(arm)
            if owner is not None and owner != goal_token:
                raise PhysicalCommandOwnedError(
                    f"{arm} arm still has physical command owner {owner!r}"
                )
            # Keep ownership latched if the middleware raises: publication may
            # already have crossed the process boundary and requires a hold.
            self._physical_owners[arm] = goal_token
            publisher()
            self._condition.notify_all()

    def publish_hold(
        self,
        arm: str,
        publisher: Callable[[], None],
        *,
        goal_token: Hashable | None = None,
    ) -> None:
        """Serialize a current-position hold with normal target publications."""

        with self._condition:
            owner = self._physical_owners.get(arm)
            if goal_token is not None and owner not in (None, goal_token):
                raise PhysicalCommandOwnedError(
                    f"{arm} arm physical command is owned by {owner!r}"
                )
            publisher()

    def release_physical(self, arm: str, goal_token: Hashable) -> bool:
        """Release only the matching owner after arrival or confirmed hold."""

        with self._condition:
            if self._physical_owners.get(arm) != goal_token:
                return False
            del self._physical_owners[arm]
            self._condition.notify_all()
            return True

    def has_physical_owner(self, arm: str, goal_token: Hashable) -> bool:
        with self._condition:
            return self._physical_owners.get(arm) == goal_token

    def physical_commands(self) -> Tuple[Tuple[str, Hashable], ...]:
        with self._condition:
            return tuple(sorted(self._physical_owners.items()))

    def discard_unhandled_admissions(self) -> int:
        """Drop requests whose goal response failed before handle acceptance.

        Call this only after the ROS executor reports that in-flight waitable
        callbacks have drained.
        """

        with self._condition:
            count = len(self._admitted)
            self._admitted.clear()
            self._condition.notify_all()
            return count

    def wait_for_idle(self, timeout_sec: float) -> bool:
        """Wait for admitted, queued and running callbacks to leave."""

        if timeout_sec < 0.0:
            raise ValueError("timeout_sec must be non-negative")
        deadline = time.monotonic() + timeout_sec
        with self._condition:
            while self._admitted or self._queued or self._running:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self._condition.wait(remaining)
            return True

    def snapshot(self) -> ArmLifecycleSnapshot:
        with self._condition:
            return ArmLifecycleSnapshot(
                shutdown_requested=self._shutdown_requested,
                admitted_count=len(self._admitted),
                queued_count=len(self._queued),
                running_count=len(self._running),
                physical_arms=tuple(sorted(self._physical_owners)),
            )
