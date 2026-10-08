"""Opt-in, schema-validated OpenAI-compatible language grounding.

The LLM is never given direct motor control; only bounded TaskRequest fields
can cross into planning. No model is downloaded, no API is called by default.
"""
from __future__ import annotations

import json
import os
import time
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from spatialmind.language import ClarificationNeeded, OBJECT_ALIASES, ROOM_ALIASES
from spatialmind.models import TaskRequest

Transport = Callable[[dict[str, Any]], dict[str, Any]]

SYSTEM_PROMPT = """
You ground a user's mobile-robot mission into a small validated task schema.
Return ONLY one JSON object with keys kind, target, room.
kind: find | navigate.
If kind=find: target must be exactly "blue toolbox" or "red first aid kit".
If kind=navigate: target must be one of lab, storage, meeting, office.
room must be null or one of lab, storage, meeting, office.
Do not generate coordinates, direct velocity commands, tool invocations,
unapproved targets, safety overrides, or claims that physical action occurred.
If the instruction is ambiguous or unsupported, return
{"kind":"clarify","target":"explain what is missing","room":null}.
""".strip()


class OpenAICompatibleInterpreter:
    def __init__(
        self, *, model: str, base_url: str = "https://api.openai.com/v1",
        api_key: str | None = None, timeout: float = 25.0,
        transport: Transport | None = None,
    ) -> None:
        if not model:
            raise ValueError("model is required")
        base_url = base_url.rstrip("/")
        if not (base_url.startswith("https://") or base_url.startswith("http://localhost")
                or base_url.startswith("http://127.0.0.1")):
            raise ValueError("HTTPS is required except for localhost inference")
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        self.model, self.base_url = model, base_url
        self.api_key = api_key if api_key is not None else os.getenv("OPENAI_API_KEY", "")
        self.timeout, self.transport = timeout, transport
        self.last_metadata: dict[str, Any] = {}

    def _request(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not self.api_key:
            raise RuntimeError(
                "OPENAI_API_KEY missing. Set it to use a remote model, "
                "or use the default zero-key rule-based interpreter."
            )
        body = json.dumps(payload).encode("utf-8")
        request = Request(
            self.base_url + "/chat/completions", data=body,
            headers={
                "Authorization": "Bearer " + self.api_key,
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with urlopen(request, timeout=self.timeout) as response:
                return json.loads(response.read().decode("utf-8"))
        except (HTTPError, URLError, TimeoutError) as exc:
            raise RuntimeError(f"Model transport failed ({type(exc).__name__})") from exc

    def parse(self, instruction: str) -> TaskRequest:
        if not instruction.strip():
            raise ClarificationNeeded("Please provide a task instruction.")
        payload = {
            "model": self.model, "temperature": 0,
            "response_format": {"type": "json_object"},
            "max_tokens": 160,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": instruction[:1200]},
            ],
        }
        start = time.perf_counter()
        response = self.transport(payload) if self.transport else self._request(payload)
        duration_ms = round(1000 * (time.perf_counter() - start), 2)
        self.last_metadata = {
            "model": self.model, "latency_ms": duration_ms,
            "usage": response.get("usage", {}),
        }
        try:
            content = response["choices"][0]["message"]["content"]
            document = json.loads(content)
            kind = document["kind"]
            target = document["target"]
            room = document.get("room")
        except (KeyError, IndexError, TypeError, ValueError) as exc:
            raise ClarificationNeeded("Model returned an invalid task schema.") from exc
        if not isinstance(target, str) or not isinstance(kind, str):
            raise ClarificationNeeded("Model produced invalid target or action.")
        if room is not None and (not isinstance(room, str) or room not in ROOM_ALIASES):
            raise ClarificationNeeded("Model selected an unsupported semantic room.")
        if kind == "clarify":
            raise ClarificationNeeded(target[:240])
        if kind == "find" and target not in OBJECT_ALIASES:
            raise ClarificationNeeded("The requested object is not in the known catalog.")
        if kind == "navigate" and target not in ROOM_ALIASES:
            raise ClarificationNeeded("The destination is not a known room.")
        if kind not in {"find", "navigate"}:
            raise ClarificationNeeded("Model selected an unsupported operation.")
        if kind == "navigate":
            room = target
        return TaskRequest(kind, target, room)
