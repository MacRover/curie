#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
import signal
import threading
import time
from dataclasses import asdict, dataclass
from typing import Any, Optional

import can
from foxglove_websocket.server import FoxgloveServer

CAN_FRAME_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "arbId": {"type": "integer"},
        "data": {"type": "array", "items": {"type": "integer"}},
        "channel": {"type": "string"},
        "dlc": {"type": "integer"},
        "isExtendedId": {"type": "boolean"},
        "isFd": {"type": "boolean"},
        "isErrorFrame": {"type": "boolean"},
        "timestampSec": {"type": "number"},
    },
    "required": [
        "arbId",
        "data",
        "channel",
        "dlc",
        "isExtendedId",
        "isFd",
        "isErrorFrame",
        "timestampSec",
    ],
}

# Spark MAX status frame bases, mask ignores 6-bit device id
DEVICE_ID_MASK = 0x3F
SPARKMAX_STATUS_BASES = {
    0x0205B800, 0x0205B840, 0x0205B880, 0x0205B8C0, 0x0205B900,
    0x0205B940, 0x0205B980, 0x0205B9C0, 0x0205BA00, 0x0205BA40,
}
SPARKMAX_BASE_MASK = 0x1FFFFFC0  # extended-id mask that zeros lower 6 bits


def parse_hex_id(s: str) -> int:
    s = s.strip()
    if s.lower().startswith("0x"):
        return int(s, 16)
    return int(s, 16) if any(c in "abcdefABCDEF" for c in s) else int(s)


@dataclass
class CanFrameMessage:
    arbId: int
    data: list[int]
    channel: str
    dlc: int
    isExtendedId: bool
    isFd: bool
    isErrorFrame: bool
    timestampSec: float


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Linux SocketCAN -> Foxglove WebSocket (JSON)")
    p.add_argument("--can-iface", default="can0", help="SocketCAN interface e.g. can0 or vcan0")
    p.add_argument("--topic", default=None, help="Foxglove topic (default: /<iface>/received_frames)")
    p.add_argument("--host", default="0.0.0.0", help="WebSocket bind host")
    p.add_argument("--port", type=int, default=8765, help="WebSocket bind port")
    p.add_argument("--print-every", type=int, default=10, help="Print every N frames (0 disables)")
    p.add_argument(
        "--receive-own",
        action="store_true",
        help="Receive frames sent by this socket (useful for local vcan tests)",
    )

    # ---- filtering controls ----
    p.add_argument(
        "--only-sparkmax-status",
        action="store_true",
        help="Only forward REV Spark MAX Status0..Status9 frames (drops everything else)",
    )
    p.add_argument(
        "--drop-arbid",
        action="append",
        default=[],
        metavar="HEX",
        help="Drop a specific arbitration ID (hex). Repeatable. Example: --drop-arbid 0x0205B801",
    )
    p.add_argument(
        "--pass-arbid",
        action="append",
        default=[],
        metavar="HEX",
        help="Allow ONLY these arbitration IDs (hex). Repeatable. If set, everything else is dropped.",
    )
    return p.parse_args()


def build_payload(msg: can.Message, channel_name: str) -> dict[str, Any]:
    return asdict(
        CanFrameMessage(
            arbId=int(msg.arbitration_id),
            data=[int(b) & 0xFF for b in msg.data],
            channel=channel_name,
            dlc=int(msg.dlc),
            isExtendedId=bool(msg.is_extended_id),
            isFd=bool(getattr(msg, "is_fd", False)),
            isErrorFrame=bool(getattr(msg, "is_error_frame", False)),
            timestampSec=float(getattr(msg, "timestamp", time.time())),
        )
    )


def should_forward(
    arb_id: int,
    *,
    only_sparkmax_status: bool,
    drop_ids: set[int],
    pass_ids: Optional[set[int]],
) -> bool:
    # strict allowlist wins
    if pass_ids is not None and len(pass_ids) > 0:
        return arb_id in pass_ids

    # explicit drop list next
    if arb_id in drop_ids:
        return False

    # sparkmax-only filter
    if only_sparkmax_status:
        base = arb_id & SPARKMAX_BASE_MASK
        return base in SPARKMAX_STATUS_BASES

    # default: forward all
    return True


async def main_async() -> int:
    args = parse_args()
    loop = asyncio.get_running_loop()

    # default topic follows iface
    if args.topic is None:
        args.topic = f"/{args.can_iface}/received_frames"

    # parse filters
    drop_ids = {parse_hex_id(x) for x in args.drop_arbid}
    pass_ids = {parse_hex_id(x) for x in args.pass_arbid} if args.pass_arbid else None

    stop = asyncio.Event()

    def _sigint(_sig=None, _frame=None):
        stop.set()

    signal.signal(signal.SIGINT, _sigint)
    signal.signal(signal.SIGTERM, _sigint)

    server = FoxgloveServer(host=args.host, port=args.port, name="Linux SocketCAN Bridge")
    server.start()

    chan_id = await server.add_channel(
        {
            "topic": args.topic,
            "encoding": "json",
            "schemaName": "CanFrameMessage",
            "schema": json.dumps(CAN_FRAME_JSON_SCHEMA),
            "schemaEncoding": "jsonschema",
        }
    )

    print(f"[bridge] Foxglove WS: ws://{args.host}:{args.port}")
    print(f"[bridge] Reading: {args.can_iface}")
    print(f"[bridge] Topic: {args.topic}")
    if args.only_sparkmax_status:
        print("[bridge] Filter: only Spark MAX status0..9")
    if drop_ids:
        print("[bridge] Filter: drop IDs:", ", ".join(f"0x{x:X}" for x in sorted(drop_ids)))
    if pass_ids:
        print("[bridge] Filter: pass ONLY IDs:", ", ".join(f"0x{x:X}" for x in sorted(pass_ids)))

    bus: can.BusABC = can.Bus(
        interface="socketcan",
        channel=args.can_iface,
        receive_own_messages=args.receive_own,
    )

    forwarded = 0
    dropped = 0

    def can_thread():
        nonlocal forwarded, dropped
        try:
            while not stop.is_set():
                msg = bus.recv(timeout=0.2)
                if msg is None:
                    continue

                arb_id = int(msg.arbitration_id)

                if not should_forward(
                    arb_id,
                    only_sparkmax_status=args.only_sparkmax_status,
                    drop_ids=drop_ids,
                    pass_ids=pass_ids,
                ):
                    dropped += 1
                    continue

                payload = build_payload(msg, args.can_iface)
                data = json.dumps(payload).encode("utf-8")
                ts = time.time_ns()

                asyncio.run_coroutine_threadsafe(
                    server.send_message(chan_id, ts, data),
                    loop,
                )

                forwarded += 1
                if args.print_every > 0 and forwarded % args.print_every == 0:
                    print(
                        f"[bridge] forwarded={forwarded} dropped={dropped} last_id=0x{arb_id:X}"
                    )
        finally:
            try:
                bus.shutdown()
            except Exception:
                pass

    t = threading.Thread(target=can_thread, daemon=True)
    t.start()

    await stop.wait()

    print(f"[bridge] stopping, total forwarded={forwarded} dropped={dropped}")
    server.close()
    await server.wait_closed()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main_async()))