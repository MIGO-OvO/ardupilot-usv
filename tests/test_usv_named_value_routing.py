from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
ROUTING_CPP = REPO_ROOT / "libraries" / "GCS_MAVLink" / "MAVLink_routing.cpp"


def test_usv_named_value_float_is_consumed_locally_not_forwarded():
    text = ROUTING_CPP.read_text(encoding="utf-8", errors="ignore")

    expected_block = """    if (msg.msgid == MAVLINK_MSG_ID_NAMED_VALUE_FLOAT) {
        // process locally (cache in usv_payload), but don't forward to other links
        return true;
    }"""

    assert expected_block in text
    assert "[REMOVED BLOCK]" not in text
