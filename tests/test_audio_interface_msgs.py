"""Behavior specs for the audio/transcript interface message definitions.

These tests codify the agreed field layout for the streaming ASR stack so we
can iterate on the message descriptions using TDD. Each message file is parsed
as a flat list of ``type name`` declarations and constants; the expectations are
kept deliberately small so they act as living documentation instead of brittle
fixture dumps.
"""
from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Tuple

import pytest


REPO_ROOT = Path(__file__).resolve().parent.parent
MSG_DIR = REPO_ROOT / "src" / "psyche_interfaces" / "msg"


def _parse_msg_fields(msg_path: Path) -> Tuple[List[Tuple[str, str]], Dict[str, Tuple[str, str]]]:
    """Return (fields, constants) for the ``.msg`` definition.

    Comments and blank lines are ignored. Field order is preserved to ensure the
    definition matches our documentation verbatim, making it easy to spot
    regressions in future diffs.
    """

    fields: List[Tuple[str, str]] = []
    constants: Dict[str, Tuple[str, str]] = {}
    for raw_line in msg_path.read_text().splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        if "=" in line:
            lhs, rhs = line.split("=", 1)
            rhs = rhs.strip()
            lhs = lhs.strip()
            if " " not in lhs:
                pytest.fail(f"Malformed constant declaration '{raw_line}' in {msg_path.name}")
            constant_type, constant_name = lhs.split(" ", 1)
            constants[constant_name.strip()] = (constant_type.strip(), rhs)
            continue
        if " " not in line:
            pytest.fail(f"Expected '<type> <name>' line in {msg_path.name}: '{raw_line}'")
        field_type, field_name = line.rsplit(" ", 1)
        fields.append((field_type.strip(), field_name.strip()))
    return fields, constants


def _assert_msg_signature(msg_name: str, expected_fields: List[Tuple[str, str]], expected_constants=None) -> None:
    msg_path = MSG_DIR / msg_name
    assert msg_path.exists(), f"Expected {msg_name} to exist in {MSG_DIR}"
    fields, constants = _parse_msg_fields(msg_path)
    assert fields == expected_fields, f"Fields for {msg_name} differed"
    expected_constants = expected_constants or {}
    assert constants == expected_constants, f"Constants for {msg_name} differed"


@pytest.mark.parametrize(
    "msg_name, expected_fields, expected_constants",
    [
        (
            "AudioPCM.msg",
            [
                ("builtin_interfaces/Time", "stamp"),
                ("uint32", "sample_rate"),
                ("uint8", "channels"),
                ("uint8", "sample_format"),
                ("uint32", "frame_index"),
                ("uint8[]", "data"),
            ],
            {},
        ),
        (
            "AudioChunk.msg",
            [
                ("builtin_interfaces/Time", "stamp_start"),
                ("builtin_interfaces/Time", "stamp_end"),
                ("string", "segment_id"),
                ("uint32", "sequence"),
                ("uint32", "sample_rate"),
                ("uint8", "channels"),
                ("uint8[]", "wav_data"),
            ],
            {},
        ),
        (
            "AudioSegment.msg",
            [
                ("builtin_interfaces/Time", "stamp_start"),
                ("builtin_interfaces/Time", "stamp_end"),
                ("string", "segment_id"),
                ("uint32", "sample_rate"),
                ("uint8", "channels"),
                ("uint8[]", "wav_data"),
                ("bool", "cut_by_timeout"),
            ],
            {},
        ),
        (
            "TranscriptPartial.msg",
            [
                ("string", "segment_id"),
                ("uint32", "rev"),
                ("builtin_interfaces/Time", "stamp_emitted"),
                ("string", "text"),
                ("float32", "confidence"),
            ],
            {},
        ),
        (
            "TranscriptSegment.msg",
            [
                ("string", "segment_id"),
                ("builtin_interfaces/Time", "segment_start"),
                ("builtin_interfaces/Time", "segment_end"),
                ("string", "text"),
                ("float32", "confidence"),
            ],
            {},
        ),
        (
            "TranscriptWhole.msg",
            [
                ("builtin_interfaces/Time", "stamp_emitted"),
                ("uint32[]", "start_char"),
                ("uint32[]", "end_char"),
                ("string[]", "text"),
            ],
            {},
        ),
        (
            "TranscriptEvent.msg",
            [
                ("builtin_interfaces/Time", "stamp_emitted"),
                ("uint8", "kind"),
                ("string", "payload_json"),
            ],
            {
                "KIND_PARTIAL": ("uint8", "0"),
                "KIND_FINAL": ("uint8", "1"),
                "KIND_HQ_PATCH": ("uint8", "2"),
            },
        ),
    ],
)
def test_message_signatures_are_locked_down(msg_name, expected_fields, expected_constants):
    """Messages act as shared contracts; assert the committed schema."""

    _assert_msg_signature(msg_name, expected_fields, expected_constants)
