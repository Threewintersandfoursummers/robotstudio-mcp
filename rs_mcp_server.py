#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobotStudio 6.08 MCP server.

Architecture
------------
    WorkBuddy  --(stdio, MCP JSON-RPC)-->  this file
    this file  --(loopback TCP, JSON lines)-->  RsMcpBridge.dll (add-in inside RobotStudio.exe)

The add-in is a RobotStudio "General" add-in that exposes a small command bus on
127.0.0.1 so an external process can drive stations, robots and RAPID programs.

Pure stdlib, no pip dependencies.
"""

import json
import os
import socket
import subprocess
import sys
import threading
import time

PORT_FILE = os.path.join(
    os.environ.get("LOCALAPPDATA", os.path.expanduser("~")), "RsMcpBridge", "port.txt"
)
BASE_PORT = 47950
PORT_COUNT = 12
RS_EXE = r"C:\Program Files (x86)\ABB Industrial IT\Robotics IT\RobotStudio 6.08\Bin64\RobotStudio.exe"


# --------------------------------------------------------------------------- #
# TCP client to the in-process bridge
# --------------------------------------------------------------------------- #
class BridgeError(Exception):
    pass


class Bridge(object):
    def __init__(self):
        self.sock = None
        self.port = None
        self.lock = threading.Lock()
        self.req_id = 0

    # -- connection ------------------------------------------------------- #
    def candidate_ports(self):
        ports = []
        try:
            with open(PORT_FILE, "r", encoding="utf-8-sig") as f:
                p = int(f.read().strip())
                ports.append(p)
        except Exception:
            pass
        for i in range(PORT_COUNT):
            p = BASE_PORT + i
            if p not in ports:
                ports.append(p)
        return ports

    def connect(self, timeout=3.0):
        if self.sock is not None:
            return
        last = None
        for port in self.candidate_ports():
            try:
                s = socket.create_connection(("127.0.0.1", port), timeout=timeout)
                s.settimeout(None)
                self.sock = s
                self.port = port
                return
            except Exception as e:  # noqa: BLE001
                last = e
        self.sock = None
        raise BridgeError(
            "Cannot reach the RobotStudio bridge on 127.0.0.1:%d-%d (%s). "
            "RobotStudio 6.08 must be running with the RsMcpBridge add-in installed; "
            "use rs_launch or start RobotStudio manually."
            % (BASE_PORT, BASE_PORT + PORT_COUNT - 1, last)
        )

    def close(self):
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass
        self.sock = None

    # -- request/response -------------------------------------------------- #
    def call(self, cmd, args=None, timeout=120):
        payload = {"cmd": cmd, "args": args or {}}
        with self.lock:
            self.connect()
            self.req_id += 1
            payload["id"] = self.req_id
            body = (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
            try:
                self.sock.settimeout(timeout)
                self.sock.sendall(body)
                buf = b""
                while not buf.endswith(b"\n"):
                    chunk = self.sock.recv(1 << 20)
                    if not chunk:
                        raise BridgeError("bridge closed the connection")
                    buf += chunk
            except BridgeError:
                self.close()
                raise
            except Exception as e:  # noqa: BLE001
                self.close()
                raise BridgeError("bridge I/O failed: %s" % e)

            try:
                res = json.loads(buf.decode("utf-8"))
            except Exception as e:  # noqa: BLE001
                raise BridgeError("bad response from bridge: %r (%s)" % (buf[:200], e))

            if not res.get("ok"):
                raise BridgeError(res.get("error") or "unknown bridge error")
            return res.get("result")


BRIDGE = Bridge()


# --------------------------------------------------------------------------- #
# Tool definitions
# --------------------------------------------------------------------------- #
def _pose_schema():
    return {
        "type": "object",
        "description": "Pose in the active work object: translation mm, rotation deg (Euler ZYX).",
        "properties": {
            "x": {"type": "number", "description": "mm"},
            "y": {"type": "number", "description": "mm"},
            "z": {"type": "number", "description": "mm"},
            "rx": {"type": "number", "description": "degrees about X"},
            "ry": {"type": "number", "description": "degrees about Y"},
            "rz": {"type": "number", "description": "degrees about Z"},
        },
        "required": ["x", "y", "z", "rx", "ry", "rz"],
    }


TOOLS = [
    {
        "name": "rs_status",
        "description": "Check the RobotStudio link: bridge version, active station, simulation state, running process.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "rs_launch",
        "description": "Start RobotStudio 6.08 (if it is not running yet) and wait until the bridge answers. Optionally open a .rsstn station file.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "station": {"type": "string", "description": "Optional absolute path to a .rsstn file to open."},
                "timeout_s": {"type": "integer", "description": "Seconds to wait for startup, default 90."},
            },
        },
    },
    {
        "name": "rs_new_station",
        "description": "Create a new empty station. Optionally drop a robot from the ABB component library into it (e.g. robot='IRB1200_7_70_STD_02').",
        "inputSchema": {
            "type": "object",
            "properties": {
                "name": {"type": "string"},
                "robot": {"type": "string", "description": "Robot library name or absolute .rslib path."},
            },
        },
    },
    {
        "name": "rs_open_station",
        "description": "Open an existing .rsstn station file.",
        "inputSchema": {
            "type": "object",
            "properties": {"path": {"type": "string"}},
            "required": ["path"],
        },
    },
    {
        "name": "rs_save_station",
        "description": "Save the active station. With 'path' it saves a copy there (SaveAs), otherwise saves in place.",
        "inputSchema": {
            "type": "object",
            "properties": {"path": {"type": "string"}},
        },
    },
    {
        "name": "rs_station_info",
        "description": "Summarise the active station: name, file, robots, RAPID tasks, controllers, simulation state.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "rs_list_robot_library",
        "description": "List robot models available in the RobotStudio component library (341 models shipped). Filter with 'pattern', e.g. 'IRB1200'.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "pattern": {"type": "string"},
                "limit": {"type": "integer", "description": "default 100"},
            },
        },
    },
    {
        "name": "rs_add_robot",
        "description": "Add a robot (mechanism) from the ABB library into the active station.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "robot": {"type": "string", "description": "Library name like 'IRB1200_7_70_STD_02' or an absolute .rslib path."},
                "name": {"type": "string", "description": "Optional instance name."},
            },
            "required": ["robot"],
        },
    },
    {
        "name": "rs_list_robots",
        "description": "List robots/mechanisms in the station with joint count, RAPID task and current joint values.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "rs_get_joints",
        "description": "Read current joint values of a robot, in degrees and radians.",
        "inputSchema": {
            "type": "object",
            "properties": {"robot": {"type": "string"}},
        },
    },
    {
        "name": "rs_set_joints",
        "description": "Jog a robot to the given joint values (moves the 3D model immediately). Units default to degrees.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "robot": {"type": "string"},
                "joints": {"type": "array", "items": {"type": "number"}},
                "units": {"type": "string", "enum": ["deg", "rad"], "description": "default deg"},
            },
            "required": ["joints"],
        },
    },
    {
        "name": "rs_home",
        "description": "Move a robot to its home position.",
        "inputSchema": {"type": "object", "properties": {"robot": {"type": "string"}}},
    },
    {
        "name": "rs_joint_limits",
        "description": "Show joint limits (degrees) and joint types of a robot.",
        "inputSchema": {"type": "object", "properties": {"robot": {"type": "string"}}},
    },
    {
        "name": "rs_fk",
        "description": "Forward kinematics: joint values (deg) -> TCP pose (mm/deg) of the flange.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "robot": {"type": "string"},
                "joints": {"type": "array", "items": {"type": "number"}, "description": "degrees; defaults to current"},
            },
        },
    },
    {
        "name": "rs_ik",
        "description": "Inverse kinematics: TCP pose (mm/deg) -> joint values (deg). Falls back to a built-in damped-least-squares solver when RobotStudio has no controller.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "robot": {"type": "string"},
                "pose": _pose_schema(),
                "seed": {"type": "array", "items": {"type": "number"}, "description": "optional seed joint values (deg)"},
            },
            "required": ["pose"],
        },
    },
    {
        "name": "rs_reach",
        "description": "Ask RobotStudio whether a pose is reachable by the robot.",
        "inputSchema": {
            "type": "object",
            "properties": {"robot": {"type": "string"}, "pose": _pose_schema()},
            "required": ["pose"],
        },
    },
    {
        "name": "rs_list_tasks",
        "description": "List RAPID tasks with their active tool/work object, targets, routines and data declarations.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "rs_list_targets",
        "description": "List robtargets of a RAPID task with pose and configuration.",
        "inputSchema": {"type": "object", "properties": {"task": {"type": "string"}}},
    },
    {
        "name": "rs_add_target",
        "description": "Create a robtarget in the RAPID task (adds both the robtarget data and its graphical target).",
        "inputSchema": {
            "type": "object",
            "properties": {
                "name": {"type": "string"},
                "pose": _pose_schema(),
                "wobj": {"type": "string", "description": "work object name, default wobj0"},
                "task": {"type": "string"},
            },
            "required": ["name", "pose"],
        },
    },
    {
        "name": "rs_add_routine",
        "description": "Create an empty RAPID path routine (PROC) in the task.",
        "inputSchema": {
            "type": "object",
            "properties": {"name": {"type": "string"}, "task": {"type": "string"}},
            "required": ["name"],
        },
    },
    {
        "name": "rs_add_move",
        "description": "Append a motion instruction to a routine. motion: J (MoveJ/MoveAbsJ), L (MoveL), C (MoveC via target). speed like 'v300' or '[100,25,25,25]', zone like 'fine' or 'z10'.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "routine": {"type": "string"},
                "target": {"type": "string", "description": "robtarget (or jointtarget for AbsJ) name"},
                "motion": {"type": "string", "enum": ["J", "L", "C", "AbsJ"], "description": "default L"},
                "speed": {"type": "string"},
                "zone": {"type": "string"},
                "tool": {"type": "string"},
                "wobj": {"type": "string"},
                "via": {"type": "string", "description": "circle point for Motion C"},
                "task": {"type": "string"},
            },
            "required": ["target"],
        },
    },
    {
        "name": "rs_show_code",
        "description": "Render the RAPID program currently modelled in the station as source text (MODULE / robtargets / PROC bodies).",
        "inputSchema": {"type": "object", "properties": {"task": {"type": "string"}}},
    },
    {
        "name": "rs_sim_state",
        "description": "Simulation state, speed and current time.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "rs_sim_control",
        "description": "Control simulation: action = start | stop | pause | resume | step | speed. Requires a virtual controller (RobotWare) for motion simulation.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "pause", "resume", "step", "speed"]},
                "value": {"type": "number", "description": "speed factor for action=speed"},
            },
            "required": ["action"],
        },
    },
    {
        "name": "rs_io_list",
        "description": "List station I/O signals with type, direction and value.",
        "inputSchema": {"type": "object", "properties": {}},
    },
    {
        "name": "rs_io_set",
        "description": "Set an I/O signal value (true/false for digital, number for analog).",
        "inputSchema": {
            "type": "object",
            "properties": {
                "name": {"type": "string"},
                "value": {"type": "string", "description": "true|false or a number"},
            },
            "required": ["name", "value"],
        },
    },
    {
        "name": "rs_raw",
        "description": "Escape hatch: send any low-level bridge command directly (e.g. debug.transform, rs.reflect). Use for diagnostics.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "cmd": {"type": "string"},
                "args": {"type": "object"},
            },
            "required": ["cmd"],
        },
    },
]


# --------------------------------------------------------------------------- #
# Tool dispatch
# --------------------------------------------------------------------------- #
def text_result(obj):
    if isinstance(obj, str):
        return {"content": [{"type": "text", "text": obj}]}
    return {"content": [{"type": "text", "text": json.dumps(obj, ensure_ascii=False, indent=2)}]}


def error_result(msg):
    return {"content": [{"type": "text", "text": "Error: " + str(msg)}], "isError": True}


def tool_launch(args):
    timeout = int(args.get("timeout_s") or 90)
    station = args.get("station")

    running = False
    try:
        BRIDGE.call("ping", timeout=2)
        running = True
    except Exception:
        running = False

    if not running:
        if not os.path.exists(RS_EXE):
            raise BridgeError("RobotStudio.exe not found at " + RS_EXE)
        creation = 0
        if hasattr(subprocess, "DETACHED_PROCESS"):
            creation = subprocess.DETACHED_PROCESS | subprocess.CREATE_NEW_PROCESS_GROUP
        subprocess.Popen([RS_EXE], creationflags=creation, close_fds=True)

    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            info = BRIDGE.call("ping", timeout=3)
            break
        except Exception as e:  # noqa: BLE001
            last = e
            BRIDGE.close()
            time.sleep(2)
    else:
        raise BridgeError("RobotStudio did not answer within %ss (%s)" % (timeout, last))

    already = info
    if station:
        info = BRIDGE.call("station.open", {"path": station})
        return {"bridge": already, "opened": info}
    return {"bridge": already}


def dispatch(name, args):
    args = args or {}

    if name == "rs_status":
        try:
            info = BRIDGE.call("ping", timeout=5)
            info["reachable"] = True
            return text_result(info)
        except Exception as e:  # noqa: BLE001
            return text_result({"reachable": False, "hint": str(e)})

    if name == "rs_launch":
        return text_result(tool_launch(args))

    mapping = {
        "rs_new_station": ("station.new", lambda a: {"name": a.get("name"), "robot": a.get("robot")}),
        "rs_open_station": ("station.open", lambda a: {"path": a.get("path")}),
        "rs_save_station": ("station.save", lambda a: {"path": a.get("path")}),
        "rs_station_info": ("station.info", lambda a: {}),
        "rs_list_robot_library": ("lib.robots", lambda a: {"pattern": a.get("pattern"),
                                                           "limit": a.get("limit") or 100}),
        "rs_add_robot": ("robot.add", lambda a: {"robot": a.get("robot"), "name": a.get("name")}),
        "rs_list_robots": ("robot.list", lambda a: {}),
        "rs_get_joints": ("robot.joints.get", lambda a: {"robot": a.get("robot")}),
        "rs_set_joints": ("robot.joints.set", lambda a: {"robot": a.get("robot"),
                                                         "joints": a.get("joints"),
                                                         "units": a.get("units")}),
        "rs_home": ("robot.home", lambda a: {"robot": a.get("robot")}),
        "rs_joint_limits": ("robot.limits", lambda a: {"robot": a.get("robot")}),
        "rs_fk": ("robot.fk", lambda a: {"robot": a.get("robot"), "joints": a.get("joints")}),
        "rs_ik": ("robot.ik", lambda a: {"robot": a.get("robot"), "pose": a.get("pose"),
                                         "seed": a.get("seed")}),
        "rs_reach": ("robot.reach", lambda a: {"robot": a.get("robot"), "pose": a.get("pose")}),
        "rs_list_tasks": ("rapid.tasks", lambda a: {}),
        "rs_list_targets": ("rapid.targets", lambda a: {"task": a.get("task")}),
        "rs_add_target": ("rapid.add_target", lambda a: {"name": a.get("name"), "pose": a.get("pose"),
                                                         "wobj": a.get("wobj"), "task": a.get("task")}),
        "rs_add_routine": ("rapid.add_routine", lambda a: {"name": a.get("name"), "task": a.get("task")}),
        "rs_add_move": ("rapid.add_move", lambda a: {"routine": a.get("routine"), "target": a.get("target"),
                                                     "motion": a.get("motion"), "speed": a.get("speed"),
                                                     "zone": a.get("zone"), "tool": a.get("tool"),
                                                     "wobj": a.get("wobj"), "via": a.get("via"),
                                                     "task": a.get("task")}),
        "rs_show_code": ("rapid.code", lambda a: {"task": a.get("task")}),
        "rs_sim_state": ("sim.state", lambda a: {}),
        "rs_io_list": ("io.list", lambda a: {}),
        "rs_io_set": ("io.set", lambda a: {"name": a.get("name"), "value": a.get("value")}),
    }

    if name == "rs_sim_control":
        action = (args.get("action") or "").lower()
        if action not in ("start", "stop", "pause", "resume", "step", "speed"):
            return error_result("action must be start|stop|pause|resume|step|speed")
        cmd = "sim." + action
        params = {}
        if action == "speed":
            params["value"] = args.get("value") or 1.0
        return text_result(BRIDGE.call(cmd, params))

    if name == "rs_raw":
        cmd = args.get("cmd")
        if not cmd:
            return error_result("cmd required")
        return text_result(BRIDGE.call(cmd, args.get("args") or {}))

    if name not in mapping:
        return error_result("unknown tool: " + name)

    cmd, build = mapping[name]
    params = {k: v for k, v in build(args).items() if v is not None}
    return text_result(BRIDGE.call(cmd, params))


# --------------------------------------------------------------------------- #
# MCP stdio plumbing
# --------------------------------------------------------------------------- #
FRAMING = {"mode": None}


def send(obj):
    data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
    if FRAMING["mode"] == "lsp":
        sys.stdout.buffer.write(b"Content-Length: %d\r\n\r\n" % len(data))
        sys.stdout.buffer.write(data)
    else:
        sys.stdout.buffer.write(data + b"\n")
    sys.stdout.buffer.flush()


def recv():
    while True:
        raw = sys.stdin.buffer.readline()
        if not raw:
            return None
        line = raw.decode("utf-8", "replace").strip()
        if line == "":
            continue
        if line.lower().startswith("content-length:"):
            FRAMING["mode"] = "lsp"
            clen = int(line.split(":", 1)[1].strip())
            while True:
                h = sys.stdin.buffer.readline()
                if not h:
                    return None
                if h.strip() == b"":
                    break
            body = sys.stdin.buffer.read(clen)
            return json.loads(body.decode("utf-8", "replace"))
        FRAMING["mode"] = "newline"
        return json.loads(line)


def main():
    sys.stderr.write("robotstudio-mcp: stdio ready\n")
    sys.stderr.flush()

    while True:
        msg = recv()
        if msg is None:
            break

        method = msg.get("method")
        mid = msg.get("id")

        if method == "initialize":
            send({
                "jsonrpc": "2.0", "id": mid,
                "result": {
                    "protocolVersion": msg.get("params", {}).get("protocolVersion", "2024-11-05"),
                    "capabilities": {"tools": {"listChanged": False}},
                    "serverInfo": {"name": "robotstudio", "version": "1.0.0"},
                },
            })
        elif method in ("notifications/initialized", "initialized", "notifications/cancelled"):
            continue
        elif method == "tools/list":
            send({"jsonrpc": "2.0", "id": mid, "result": {"tools": TOOLS}})
        elif method == "tools/call":
            params = msg.get("params") or {}
            name = params.get("name")
            arguments = params.get("arguments") or {}
            try:
                result = dispatch(name, arguments)
            except BridgeError as e:
                result = error_result(e)
            except Exception as e:  # noqa: BLE001
                result = error_result("%s: %s" % (type(e).__name__, e))
            send({"jsonrpc": "2.0", "id": mid, "result": result})
        elif method == "ping":
            send({"jsonrpc": "2.0", "id": mid, "result": {}})
        elif mid is not None:
            send({"jsonrpc": "2.0", "id": mid,
                  "error": {"code": -32601, "message": "Method not found: %s" % method}})


if __name__ == "__main__":
    main()
