"""Conservative language grounding for the no-model reference policy.

This module intentionally does not claim general natural-language understanding.
A model-backed interpreter can replace it behind the TaskRequest contract.
"""

from __future__ import annotations

from spatialmind.models import TaskRequest


class ClarificationNeeded(ValueError):
    pass


ROOM_ALIASES = {
    "lab": ("lab", "laboratory", "实验室", "研发室"),
    "storage": ("storage", "warehouse", "仓库", "储藏室"),
    "meeting": ("meeting", "conference", "会议室"),
    "office": ("office", "办公室"),
}

OBJECT_ALIASES = {
    "blue toolbox": ("blue toolbox", "blue tool box", "蓝色工具箱", "蓝工具箱"),
    "red first aid kit": ("red first aid kit", "first aid kit", "急救箱", "医药箱"),
}


class RuleBasedInterpreter:
    def parse(self, text: str) -> TaskRequest:
        text = text.lower().strip()
        if not text:
            raise ClarificationNeeded("Please specify a destination or an object.")
        room = next(
            (key for key, aliases in ROOM_ALIASES.items() if any(a in text for a in aliases)),
            None,
        )
        target = next(
            (key for key, aliases in OBJECT_ALIASES.items() if any(a in text for a in aliases)),
            None,
        )
        if target:
            return TaskRequest("find", target, room)
        if "toolbox" in text or "工具箱" in text:
            raise ClarificationNeeded("Which toolbox? For example: blue toolbox.")
        if room:
            return TaskRequest("navigate", room, room)
        raise ClarificationNeeded(
            "I can currently find a blue toolbox or red first aid kit, "
            "or navigate to the lab, storage, meeting room or office."
        )
