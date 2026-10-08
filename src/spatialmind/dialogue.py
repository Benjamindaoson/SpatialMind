"""Session-level clarification and interaction controller for mobile-robot missions.

Missions execute synchronously in v0.1. This module does NOT implement
live mid-motion barge-in, voice streaming or user preference learning.
"""
from __future__ import annotations

from dataclasses import dataclass
from uuid import uuid4

from spatialmind.language import ClarificationNeeded
from spatialmind.models import TaskResult
from spatialmind.runtime import AgentRuntime


@dataclass(frozen=True)
class DialogueResponse:
    session_id: str
    kind: str
    reply: str
    result: TaskResult | None = None

    def to_dict(self) -> dict:
        return {
            "session_id": self.session_id,
            "kind": self.kind,
            "reply": self.reply,
            "result": self.result.to_dict() if self.result else None,
        }


class DialogueSession:
    def __init__(self, runtime: AgentRuntime, session_id: str | None = None):
        self.runtime = runtime
        self.session_id = session_id or uuid4().hex[:12]
        self.last_result: TaskResult | None = None
        self.pending_prompt: str | None = None
        self.history: list[tuple[str, str]] = []

    def say(self, text: str, *, max_actions: int = 80) -> DialogueResponse:
        text = text.strip()
        if not text:
            return self._answer("clarification", "Tell me which object to find or where to go.")
        lowered = text.lower()
        if lowered in {"status", "进度", "任务状态"}:
            if self.last_result:
                return self._answer(
                    "status", f"Last mission: {self.last_result.status.value}; "
                    f"{self.last_result.reason}; task {self.last_result.task_id}.",
                    self.last_result,
                )
            return self._answer("status", "No mission has been executed in this session.")
        if lowered in {"stop", "cancel", "取消", "停止"}:
            self.runtime.robot.stop()
            self.pending_prompt = None
            return self._answer(
                "acknowledgement",
                "Stop requested. No background mission is active in this synchronous prototype.",
            )
        # A follow-up is grounded as a new target only; previous ambiguous
        # language is not silently merged with a model's guess.
        try:
            self.runtime.interpreter.parse(text)
            self.pending_prompt = None
            result = self.runtime.run(text, max_actions=max_actions)
        except ClarificationNeeded as exc:
            self.pending_prompt = text
            return self._answer("clarification", str(exc))
        self.last_result = result
        if result.evidence_id is not None:
            reply = (
                f"Target verified by observation #{result.evidence_id}. "
                f"Task {result.task_id}: {result.status.value}."
            )
        else:
            reply = (
                f"Task {result.task_id}: {result.status.value}; "
                f"{result.reason}. Navigation actions: {result.steps}."
            )
        return self._answer("result", reply, result)

    def _answer(
        self, kind: str, reply: str, result: TaskResult | None = None
    ) -> DialogueResponse:
        self.history.append((kind, reply))
        return DialogueResponse(self.session_id, kind, reply, result)
