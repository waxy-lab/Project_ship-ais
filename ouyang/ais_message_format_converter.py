#!/usr/bin/env python3
"""Convert this project's AIS MQTT payloads to Ouyang simulator format.

Input default:
  topic: drone/ais/data
  payload:
    {"id":"123456789","latitude":40.6,"longitude":-73.8,"sog":12,
     "cog":90,"heading":90,"msgTime":"2026-04-22T09:30:15+00:00"}

Output default:
  topic: usv/AisMessage
  payload:
    {"MSGTIME":"2026-4-22 17:30:15","MMSI":"123456789","LAT":40.6,
     "LON":-73.8,"SOG":12.0,"COG":90.0,"HEADING":90.0}
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
from datetime import datetime
from typing import Any

import paho.mqtt.client as mqtt


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 1883
DEFAULT_SOURCE_TOPIC = "drone/ais/data"
DEFAULT_TARGET_TOPIC = "usv/AisMessage"


def _local_msg_time() -> str:
    now = datetime.now()
    return f"{now.year}-{now.month}-{now.day} {now.hour:02d}:{now.minute:02d}:{now.second:02d}"


def _as_float(value: Any, default: float) -> float:
    if value is None:
        return default
    if isinstance(value, str) and value.strip().lower() in {
        "",
        "none",
        "null",
        "not available",
    }:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _as_mmsi(payload: dict[str, Any]) -> str:
    value = payload.get("id", payload.get("mmsi", payload.get("MMSI")))
    if value is None:
        return ""
    return str(value)


def convert_ais_payload(payload: dict[str, Any]) -> dict[str, Any]:
    """Convert known project AIS payload variants to required simulator schema."""
    latitude = payload.get("latitude", payload.get("LAT"))
    longitude = payload.get("longitude", payload.get("LON"))
    sog = payload.get("sog", payload.get("speed_over_ground", payload.get("SOG")))
    cog = payload.get("cog", payload.get("course_over_ground", payload.get("COG")))
    heading = payload.get("heading", payload.get("true_heading", payload.get("HEADING")))

    return {
        "MSGTIME": _local_msg_time(),
        "MMSI": _as_mmsi(payload),
        "LAT": round(_as_float(latitude, 0.0), 6),
        "LON": round(_as_float(longitude, 0.0), 6),
        "SOG": round(_as_float(sog, 0.0), 2),
        "COG": round(_as_float(cog, 360.0), 1),
        "HEADING": round(_as_float(heading, 511.0), 1),
    }


class AisMessageFormatConverter:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.published_count = 0
        self.error_count = 0

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION1,
            client_id=args.client_id,
        )
        if args.username:
            self.client.username_pw_set(args.username, args.password or None)
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

    def connect(self) -> None:
        self.client.connect(self.args.host, self.args.port, keepalive=60)

    def loop_forever(self) -> None:
        print(
            "AIS format converter running\n"
            f"  broker : {self.args.host}:{self.args.port}\n"
            f"  input  : {self.args.source_topic}\n"
            f"  output : {self.args.target_topic}",
            flush=True,
        )
        self.client.loop_forever()

    def stop(self) -> None:
        self.client.disconnect()
        print(
            f"\nStopped. published={self.published_count}, errors={self.error_count}",
            flush=True,
        )

    def _on_connect(self, client: mqtt.Client, _userdata: Any, _flags: dict, rc: int) -> None:
        if rc != 0:
            print(f"MQTT connect failed: rc={rc}", file=sys.stderr, flush=True)
            return
        client.subscribe(self.args.source_topic, qos=0)
        print(f"Connected. Subscribed to {self.args.source_topic}", flush=True)

    def _on_message(self, client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
        try:
            source_payload = json.loads(msg.payload.decode("utf-8"))
            if not isinstance(source_payload, dict):
                raise ValueError("source payload root must be a JSON object")

            target_payload = convert_ais_payload(source_payload)
            target_json = json.dumps(target_payload, ensure_ascii=False, separators=(",", ":"))
            result = client.publish(self.args.target_topic, target_json, qos=self.args.qos)

            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                raise RuntimeError(f"publish failed: rc={result.rc}")

            self.published_count += 1
            if self.args.verbose:
                print(target_json, flush=True)
            elif self.published_count % self.args.log_every == 1:
                print(
                    f"Published #{self.published_count}: "
                    f"MMSI={target_payload['MMSI']} "
                    f"LAT={target_payload['LAT']} LON={target_payload['LON']}",
                    flush=True,
                )
        except Exception as exc:
            self.error_count += 1
            print(f"Convert error: {exc}; raw={msg.payload!r}", file=sys.stderr, flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Subscribe to current AIS MQTT payloads and republish Ouyang simulator AIS format."
    )
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"MQTT broker host, default {DEFAULT_HOST}")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"MQTT broker port, default {DEFAULT_PORT}")
    parser.add_argument("--source-topic", default=DEFAULT_SOURCE_TOPIC, help=f"source topic, default {DEFAULT_SOURCE_TOPIC}")
    parser.add_argument("--target-topic", default=DEFAULT_TARGET_TOPIC, help=f"target topic, default {DEFAULT_TARGET_TOPIC}")
    parser.add_argument("--username", default="", help="MQTT username")
    parser.add_argument("--password", default="", help="MQTT password")
    parser.add_argument("--client-id", default="ais_ouyang_format_converter", help="MQTT client id")
    parser.add_argument("--qos", type=int, choices=[0, 1, 2], default=0, help="publish QoS, default 0")
    parser.add_argument("--verbose", action="store_true", help="print every converted JSON payload")
    parser.add_argument("--log-every", type=int, default=20, help="print progress every N messages")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    converter = AisMessageFormatConverter(args)

    def handle_signal(_signum: int, _frame: Any) -> None:
        converter.stop()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        converter.connect()
        converter.loop_forever()
    except KeyboardInterrupt:
        converter.stop()
    except Exception as exc:
        print(f"Fatal error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
