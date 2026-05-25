#!/usr/bin/env python3
from __future__ import annotations

import argparse
import signal
import time

from motor_control import CanFrame, Cia402Controlword, Cia402Protocol, SocketCanBus


AXIS_COUNT_PER_NODE = 2
NODE_IDS = (1, 2, 3)
WELCON_VOLTAGE_MODE = -11

SDO_ABORT_CODES = {
    0x05030000: "toggle bit not alternated",
    0x05040000: "SDO protocol timed out",
    0x05040001: "client/server command specifier invalid",
    0x06010000: "unsupported access to object",
    0x06010001: "attempt to read a write-only object",
    0x06010002: "attempt to write a read-only object",
    0x06020000: "object does not exist",
    0x06040041: "object cannot be mapped to PDO",
    0x06040042: "PDO length exceeded",
    0x06090011: "subindex does not exist",
    0x06090030: "value range exceeded",
    0x06090031: "value too high",
    0x06090032: "value too low",
    0x08000000: "general error",
    0x08000020: "data cannot be transferred or stored to the application",
    0x08000021: "data cannot be transferred or stored because of local control",
    0x08000022: "data cannot be transferred or stored in the present device state",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send WELCON Q-axis voltage commands over CANopen.")
    parser.add_argument("--channel", default="can0", help="SocketCAN channel, default: can0")
    parser.add_argument(
        "--axis",
        type=int,
        default=1,
        choices=(1, 2),
        help="Axis used when three voltage values are given, default: 1",
    )
    parser.add_argument("--limit-mv", type=int, default=10000, help="Absolute voltage limit in mV, default: 10000")
    parser.add_argument(
        "--hold-sec",
        type=float,
        default=None,
        help="Optional hold time for one-shot mode. If omitted, one-shot voltage remains until another command is sent.",
    )
    parser.add_argument("--rate-hz", type=float, default=None, help="Repeat voltage writes at this rate until stopped")
    parser.add_argument("--response-timeout", type=float, default=0.2, help="SDO response timeout in seconds")
    parser.add_argument("--zero-on-exit", action="store_true", help="Send 0 mV before exiting")
    parser.add_argument(
        "--object",
        choices=("target-analog", "q-axis"),
        default="target-analog",
        help="Voltage object to write, default: target-analog",
    )
    parser.add_argument(
        "--analog-channel",
        dest="analog_channel",
        type=int,
        default=1,
        choices=(1, 2),
        help="Target analog input voltage channel, default: 1",
    )
    parser.add_argument(
        "--nmt",
        choices=("start", "preop", "none"),
        default="start",
        help="NMT command before writes, default: start",
    )
    parser.add_argument(
        "--enable",
        action="store_true",
        help="Run CiA402 fault-reset and enable-operation sequence before voltage writes",
    )
    parser.add_argument("--read-status", action="store_true", help="Read statusword and mode display after enabling")
    parser.add_argument("--read-feedback", action="store_true", help="Read target voltage and current feedback after writes")
    parser.add_argument(
        "--mode-value",
        type=int,
        default=WELCON_VOLTAGE_MODE,
        help="Mode value to write to 0x6060/0x6860 before enabling, default: -11 for WELCON Voltage Mode",
    )
    parser.add_argument(
        "voltages_mv",
        type=int,
        nargs="+",
        metavar="MV",
        help=(
            "Either 3 values for node1..3 on --axis, or 6 values as "
            "node1_axis1 node1_axis2 node2_axis1 node2_axis2 node3_axis1 node3_axis2"
        ),
    )
    return parser.parse_args()


def clamp(value: int, limit: int) -> int:
    return max(-limit, min(limit, value))


def build_voltage_commands(args: argparse.Namespace) -> list[tuple[int, int, int]]:
    values = [clamp(v, args.limit_mv) for v in args.voltages_mv]

    if len(values) == len(NODE_IDS):
        return [(node_id, args.axis, voltage_mv) for node_id, voltage_mv in zip(NODE_IDS, values)]

    if len(values) == len(NODE_IDS) * AXIS_COUNT_PER_NODE:
        commands = []
        value_index = 0
        for node_id in NODE_IDS:
            for axis in range(1, AXIS_COUNT_PER_NODE + 1):
                commands.append((node_id, axis, values[value_index]))
                value_index += 1
        return commands

    raise ValueError("give either 3 voltages or 6 voltages")


def format_frame(frame: CanFrame) -> str:
    data = " ".join(f"{byte:02X}" for byte in frame.data)
    return f"{frame.can_id:03X} [{len(frame.data)}] {data}"


def wait_sdo_response(
    bus: SocketCanBus,
    protocol: Cia402Protocol,
    node_id: int,
    timeout: float,
) -> bool:
    deadline = time.monotonic() + timeout
    expected_can_id = 0x580 + node_id

    while time.monotonic() < deadline:
        frame = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if frame is None:
            break

        print(f"RX {format_frame(frame)}")
        if frame.can_id != expected_can_id:
            continue

        response = protocol.parse_sdo_response(frame)
        if response is None:
            print(f"  node {node_id}: non-SDO response")
            return False
        if response.abort_code is not None:
            reason = SDO_ABORT_CODES.get(response.abort_code, "unknown abort code")
            print(f"  node {node_id}: SDO abort 0x{response.abort_code:08X} ({reason})")
            return False

        print(f"  node {node_id}: SDO write acknowledged")
        return True

    print(f"  node {node_id}: timeout waiting for SDO response")
    return False


def send_voltages(
    bus: SocketCanBus,
    protocol: Cia402Protocol,
    commands: list[tuple[int, int, int]],
    response_timeout: float,
    object_name: str,
    analog_channel: int,
) -> list[tuple[int, int]]:
    acknowledged_axes = []
    for node_id, axis, voltage_mv in commands:
        if object_name == "q-axis":
            frame = protocol.make_q_axis_voltage_mv_sdo(node_id=node_id, voltage_mv=voltage_mv, axis=axis)
            object_label = "Q-Axis Voltage"
        else:
            frame = protocol.make_target_analog_input_voltage_mv_sdo(
                node_id=node_id,
                voltage_mv=voltage_mv,
                axis=axis,
                channel=analog_channel,
            )
            object_label = f"Target Analog Input Voltage{analog_channel}"

        print(f"TX node {node_id} axis {axis} {object_label}: {voltage_mv} mV -> {format_frame(frame)}")
        bus.send(frame)
        if wait_sdo_response(bus, protocol, node_id, response_timeout):
            acknowledged_axes.append((node_id, axis))
        else:
            print(f"  node {node_id} axis {axis}: ignored and continuing")
    return acknowledged_axes


def repeat_voltages(
    bus: SocketCanBus,
    protocol: Cia402Protocol,
    commands: list[tuple[int, int, int]],
    rate_hz: float,
    stop_requested,
    object_name: str,
    analog_channel: int,
) -> None:
    if rate_hz <= 0:
        raise ValueError("--rate-hz must be positive")

    period = 1.0 / rate_hz
    next_time = time.monotonic()
    loop_count = 0
    print(f"Repeating voltage commands at {rate_hz:g} Hz. Press Ctrl+C to stop.")

    while not stop_requested():
        for node_id, axis, voltage_mv in commands:
            if object_name == "q-axis":
                frame = protocol.make_q_axis_voltage_mv_sdo(node_id=node_id, voltage_mv=voltage_mv, axis=axis)
            else:
                frame = protocol.make_target_analog_input_voltage_mv_sdo(
                    node_id=node_id,
                    voltage_mv=voltage_mv,
                    axis=axis,
                    channel=analog_channel,
                )
            bus.send(frame)

        loop_count += 1
        if loop_count == 1 or loop_count % max(1, int(rate_hz)) == 0:
            print(f"  repeated {loop_count} cycles")

        next_time += period
        sleep_time = next_time - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            next_time = time.monotonic()


def send_sdo_with_log(
    bus: SocketCanBus,
    protocol: Cia402Protocol,
    node_id: int,
    label: str,
    frame: CanFrame,
    response_timeout: float,
) -> bool:
    print(f"TX node {node_id} {label} -> {format_frame(frame)}")
    bus.send(frame)
    return wait_sdo_response(bus, protocol, node_id, response_timeout)


def read_sdo_with_log(
    bus: SocketCanBus,
    protocol: Cia402Protocol,
    node_id: int,
    label: str,
    frame: CanFrame,
    response_timeout: float,
) -> int | None:
    print(f"TX node {node_id} {label} -> {format_frame(frame)}")
    bus.send(frame)
    deadline = time.monotonic() + response_timeout

    while time.monotonic() < deadline:
        rx = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if rx is None:
            break
        print(f"RX {format_frame(rx)}")
        if rx.can_id != 0x580 + node_id:
            continue
        response = protocol.parse_sdo_response(rx)
        if response is None:
            continue
        if response.abort_code is not None:
            reason = SDO_ABORT_CODES.get(response.abort_code, "unknown abort code")
            print(f"  node {node_id}: SDO abort 0x{response.abort_code:08X} ({reason})")
            return None
        print(f"  node {node_id}: {label} = {response.value}")
        return response.value

    print(f"  node {node_id}: timeout waiting for {label}")
    return None


def enable_axes(
    bus: SocketCanBus,
    protocol: Cia402Protocol,
    axes: list[tuple[int, int]],
    mode_value: int | None,
    response_timeout: float,
) -> None:
    for node_id, axis in axes:
        if mode_value is not None:
            send_sdo_with_log(
                bus,
                protocol,
                node_id,
                f"axis {axis} mode=0x{mode_value & 0xFF:02X}",
                protocol.make_axis_mode_sdo(node_id=node_id, axis=axis, mode=mode_value),
                response_timeout,
            )

        for label, controlword in (
            ("fault reset", Cia402Controlword.FAULT_RESET),
            ("shutdown", Cia402Controlword.SHUTDOWN),
            ("switch on", Cia402Controlword.SWITCH_ON),
            ("enable operation", Cia402Controlword.ENABLE_OPERATION),
        ):
            send_sdo_with_log(
                bus,
                protocol,
                node_id,
                f"axis {axis} {label}",
                protocol.make_axis_controlword_sdo(node_id=node_id, axis=axis, controlword=controlword),
                response_timeout,
            )
            time.sleep(0.02)


def read_axis_states(
    bus: SocketCanBus,
    protocol: Cia402Protocol,
    axes: list[tuple[int, int]],
    response_timeout: float,
) -> None:
    for node_id, axis in axes:
        read_sdo_with_log(
            bus,
            protocol,
            node_id,
            f"axis {axis} statusword",
            protocol.make_axis_statusword_read_sdo(node_id=node_id, axis=axis),
            response_timeout,
        )


def read_axis_feedback(
    bus: SocketCanBus,
    protocol: Cia402Protocol,
    commands: list[tuple[int, int, int]],
    analog_channel: int,
    object_name: str,
    response_timeout: float,
) -> None:
    for node_id, axis, _ in commands:
        if object_name == "q-axis":
            read_sdo_with_log(
                bus,
                protocol,
                node_id,
                f"axis {axis} q-axis voltage",
                protocol.make_q_axis_voltage_read_sdo(node_id=node_id, axis=axis),
                response_timeout,
            )
        else:
            read_sdo_with_log(
                bus,
                protocol,
                node_id,
                f"axis {axis} target analog voltage{analog_channel}",
                protocol.make_target_analog_input_voltage_read_sdo(
                    node_id=node_id,
                    axis=axis,
                    channel=analog_channel,
                ),
                response_timeout,
            )
        read_sdo_with_log(
            bus,
            protocol,
            node_id,
            f"axis {axis} actual current",
            protocol.make_actual_current_read_sdo(node_id=node_id, axis=axis),
            response_timeout,
        )
        read_sdo_with_log(
            bus,
            protocol,
            node_id,
            f"axis {axis} q-axis demand current",
            protocol.make_q_axis_demand_current_read_sdo(node_id=node_id, axis=axis),
            response_timeout,
        )
        read_sdo_with_log(
            bus,
            protocol,
            node_id,
            f"axis {axis} q-axis actual current",
            protocol.make_q_axis_actual_current_read_sdo(node_id=node_id, axis=axis),
            response_timeout,
        )
        read_sdo_with_log(
            bus,
            protocol,
            node_id,
            f"axis {axis} mode display",
            protocol.make_axis_mode_read_sdo(node_id=node_id, axis=axis),
            response_timeout,
        )


def main() -> int:
    args = parse_args()
    protocol = Cia402Protocol()
    try:
        commands = build_voltage_commands(args)
    except ValueError as exc:
        print(f"error: {exc}")
        return 2

    zero_commands = [(node_id, axis, 0) for node_id in NODE_IDS for axis in range(1, AXIS_COUNT_PER_NODE + 1)]
    stop_requested = False

    def request_stop(signum, frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    def is_stop_requested() -> bool:
        return stop_requested

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    with SocketCanBus(args.channel, receive_timeout=0.0) as bus:
        if args.nmt == "start":
            nmt_frame = protocol.make_nmt_start(0)
            print(f"TX NMT start all -> {format_frame(nmt_frame)}")
            bus.send(nmt_frame)
        elif args.nmt == "preop":
            nmt_frame = protocol.make_nmt_pre_operational(0)
            print(f"TX NMT pre-operational all -> {format_frame(nmt_frame)}")
            bus.send(nmt_frame)
        else:
            print("Skipping NMT command")

        if args.enable:
            axes_to_enable = sorted({(node_id, axis) for node_id, axis, _ in commands})
            enable_axes(bus, protocol, axes_to_enable, args.mode_value, args.response_timeout)
            if args.read_status:
                read_axis_states(bus, protocol, axes_to_enable, args.response_timeout)

        if args.rate_hz is None:
            send_voltages(
                bus,
                protocol,
                commands,
                args.response_timeout,
                args.object,
                args.analog_channel,
            )
            if args.read_feedback:
                read_axis_feedback(
                    bus,
                    protocol,
                    commands,
                    args.analog_channel,
                    args.object,
                    args.response_timeout,
                )
        else:
            repeat_voltages(
                bus,
                protocol,
                commands,
                args.rate_hz,
                is_stop_requested,
                args.object,
                args.analog_channel,
            )

        if args.hold_sec is not None and args.rate_hz is None:
            end_time = time.monotonic() + args.hold_sec
            while time.monotonic() < end_time and not stop_requested:
                time.sleep(0.01)

        if args.zero_on_exit:
            print("Returning all commanded voltages to 0 mV")
            send_voltages(
                bus,
                protocol,
                zero_commands,
                args.response_timeout,
                args.object,
                args.analog_channel,
            )
        else:
            print("Leaving commanded voltages active. Send a new command, or run with all zeros to clear.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
