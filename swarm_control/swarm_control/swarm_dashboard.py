#!/usr/bin/env python3
"""
Swarm Control Web Dashboard

ROS 2 node that serves a live web dashboard on http://localhost:8080.
Uses Python stdlib http.server + Server-Sent Events (SSE) for real-time
push -- no external dependencies required.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Dict

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

_lock = threading.Lock()
_state: Dict = {
    'mission': 'UNKNOWN',
    'uptime': 0,
    'drones': {},
    'frontiers': {},
    'events': [],
    'stats': {
        'total_assignments': 0,
        'goals_reached': 0,
        'goals_failed': 0,
        'drone_state_hz': 0.0,
        'frontier_hz': 0.0,
        'assignment_hz': 0.0,
    },
}

# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------


class _RateTracker:
    """Sliding-window message rate estimator."""

    def __init__(self, window: float = 5.0):
        self._stamps: deque[float] = deque()
        self._window = window

    def tick(self) -> None:
        now = time.time()
        self._stamps.append(now)
        cutoff = now - self._window
        while self._stamps and self._stamps[0] < cutoff:
            self._stamps.popleft()

    def hz(self) -> float:
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        if span < 1e-6:
            return 0.0
        return (len(self._stamps) - 1) / span


class DashboardNode(Node):
    def __init__(self):
        super().__init__('swarm_dashboard')
        self._start = time.time()
        self._events: deque[str] = deque(maxlen=50)
        self._total_assign = 0
        self._goals_reached = 0
        self._goals_failed = 0
        self._drone_rate = _RateTracker()
        self._frontier_rate = _RateTracker()
        self._assign_rate = _RateTracker()
        self._drones: Dict = {}
        self._frontiers: Dict = {}
        self._mission = 'UNKNOWN'
        self._goal_start_times: Dict[str, float] = {}

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.create_subscription(String, '/swarm/drone_states', self._drone_cb, qos)
        self.create_subscription(PoseStamped, '/swarm/frontiers', self._frontier_cb, qos)
        self.create_subscription(String, '/swarm/assignments', self._assign_cb, qos)
        self.create_subscription(
            String, '/swarm/mission_state', self._mission_cb,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=1),
        )
        self.create_timer(0.5, self._push)
        self.get_logger().info('Dashboard node started – http://localhost:8080')

    def _ts(self) -> str:
        return time.strftime('%H:%M:%S')

    def _drone_cb(self, msg: String) -> None:
        self._drone_rate.tick()
        try:
            parts = msg.data.split(':')
            if len(parts) < 4:
                return
            rid, fsm = parts[0], parts[1]
            px, py, pyaw = parts[2].split(',')
            assign = parts[3] if parts[3] != 'None' else None

            prev = self._drones.get(rid, {})
            prev_fsm = prev.get('state', '')
            if prev_fsm and prev_fsm != fsm:
                self._events.append(f'{self._ts()} {rid}: {prev_fsm} -> {fsm}')
                if fsm == 'AVAILABLE' and prev_fsm == 'EXECUTING_GOAL':
                    self._goals_reached += 1
                    if rid in self._goal_start_times:
                        del self._goal_start_times[rid]
                if fsm == 'RECOVERY' and prev_fsm == 'EXECUTING_GOAL':
                    self._goals_failed += 1
                    if rid in self._goal_start_times:
                        del self._goal_start_times[rid]
                if fsm == 'EXECUTING_GOAL' and prev_fsm != 'EXECUTING_GOAL':
                    self._goal_start_times[rid] = time.time()

            self._drones[rid] = {
                'state': fsm,
                'x': float(px),
                'y': float(py),
                'yaw': float(pyaw),
                'assignment': assign,
                'updated': time.time(),
            }
        except (ValueError, IndexError):
            pass

    def _frontier_cb(self, msg: PoseStamped) -> None:
        self._frontier_rate.tick()
        fid = f"f_{int(msg.pose.position.x)}_{int(msg.pose.position.y)}"
        self._frontiers[fid] = {
            'cx': msg.pose.position.x,
            'cy': msg.pose.position.y,
            'size': int(msg.pose.orientation.w),
            'updated': time.time(),
        }

    def _assign_cb(self, msg: String) -> None:
        self._assign_rate.tick()
        try:
            parts = msg.data.split(':')
            if len(parts) >= 4:
                rid, fid = parts[0], parts[1]
                cx, cy = float(parts[2]), float(parts[3])
                self._total_assign += 1
                self._events.append(
                    f'{self._ts()} Assign {rid} -> {fid} ({cx:.1f},{cy:.1f})'
                )
        except (ValueError, IndexError):
            pass

    def _mission_cb(self, msg: String) -> None:
        old = self._mission
        self._mission = msg.data
        if old != msg.data:
            self._events.append(f'{self._ts()} Mission: {old} -> {msg.data}')

    def _push(self) -> None:
        """Update the shared state dict consumed by the HTTP server."""
        now = time.time()
        active_frontiers = {
            fid: f for fid, f in self._frontiers.items()
            if now - f['updated'] < 15.0
        }

        with _lock:
            _state['mission'] = self._mission
            _state['uptime'] = int(now - self._start)
            _state['drones'] = dict(self._drones)
            _state['frontiers'] = active_frontiers
            _state['events'] = list(self._events)
            _state['stats'] = {
                'total_assignments': self._total_assign,
                'goals_reached': self._goals_reached,
                'goals_failed': self._goals_failed,
                'drone_state_hz': round(self._drone_rate.hz(), 1),
                'frontier_hz': round(self._frontier_rate.hz(), 1),
                'assignment_hz': round(self._assign_rate.hz(), 2),
            }

# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------

_HTML = r"""<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Swarm Dashboard</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
  background:#0d1117;color:#c9d1d9;padding:12px;font-size:14px}
h1{font-size:18px;color:#58a6ff;margin-bottom:8px}
.header{display:flex;gap:20px;align-items:center;padding:10px 16px;
  background:#161b22;border:1px solid #30363d;border-radius:8px;margin-bottom:12px;flex-wrap:wrap}
.header .item{display:flex;flex-direction:column;align-items:center;min-width:80px}
.header .label{font-size:11px;color:#8b949e;text-transform:uppercase}
.header .value{font-size:20px;font-weight:700}
.mission-state{padding:4px 10px;border-radius:4px;font-weight:700}
.ms-RUNNING{background:#238636;color:#fff}
.ms-IDLE,.ms-READY{background:#1f6feb;color:#fff}
.ms-TAKEOFF_WINDOW_A,.ms-TAKEOFF_WINDOW_B{background:#9e6a03;color:#fff}
.ms-ABORT,.ms-EMERGENCY{background:#da3633;color:#fff}
.ms-COMPLETE{background:#388bfd;color:#fff}
.panels{display:grid;grid-template-columns:1fr 1fr;gap:12px}
@media(max-width:900px){.panels{grid-template-columns:1fr}}
.panel{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:12px;overflow:auto}
.panel h2{font-size:13px;color:#8b949e;text-transform:uppercase;margin-bottom:8px;letter-spacing:.5px}
table{width:100%;border-collapse:collapse;font-size:13px}
th{text-align:left;color:#8b949e;font-weight:600;padding:4px 8px;border-bottom:1px solid #30363d}
td{padding:4px 8px;border-bottom:1px solid #21262d}
.state{padding:2px 6px;border-radius:3px;font-size:12px;font-weight:600;display:inline-block}
.s-AVAILABLE{background:#238636;color:#fff}
.s-EXECUTING_GOAL{background:#8957e5;color:#fff}
.s-BOOT,.s-PREFLIGHT{background:#484f58;color:#c9d1d9}
.s-ARMED,.s-TAKING_OFF,.s-STAGING{background:#9e6a03;color:#fff}
.s-RECOVERY{background:#da3633;color:#fff}
.s-EMERGENCY{background:#f85149;color:#fff}
.s-RETURNING,.s-LANDING,.s-LANDED{background:#1f6feb;color:#fff}
.events{max-height:260px;overflow-y:auto;font-family:'SF Mono',Menlo,monospace;font-size:12px;line-height:1.6}
.events div{padding:2px 0;border-bottom:1px solid #21262d}
.perf-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.perf-card{background:#0d1117;border:1px solid #30363d;border-radius:6px;padding:10px;text-align:center}
.perf-card .pv{font-size:22px;font-weight:700;color:#58a6ff}
.perf-card .pl{font-size:11px;color:#8b949e;margin-top:2px}
.warn{color:#d29922}.err{color:#f85149}
.ok{color:#3fb950}
</style>
</head><body>
<h1>Swarm Control Dashboard</h1>
<div class="header" id="hdr"></div>
<div class="panels">
  <div class="panel"><h2>Drones</h2><table><thead><tr>
    <th>Name</th><th>State</th><th>Position</th><th>Assignment</th><th>In-State</th>
  </tr></thead><tbody id="dtbl"></tbody></table></div>
  <div class="panel"><h2>Performance</h2><div class="perf-grid" id="perf"></div></div>
  <div class="panel"><h2>Frontiers <span id="fcnt"></span></h2><table><thead><tr>
    <th>ID</th><th>Center</th><th>Size</th>
  </tr></thead><tbody id="ftbl"></tbody></table></div>
  <div class="panel"><h2>Event Log</h2><div class="events" id="elog"></div></div>
</div>
<script>
const es=new EventSource('/api/stream');
es.onmessage=e=>{try{render(JSON.parse(e.data))}catch(x){}};
es.onerror=()=>{document.title='Dashboard (disconnected)'};

function render(d){
  document.title='Swarm Dashboard';
  // Header
  const mc='ms-'+(d.mission||'UNKNOWN');
  let h=`<div class="item"><span class="label">Mission</span>
    <span class="mission-state ${mc}">${d.mission}</span></div>
    <div class="item"><span class="label">Uptime</span><span class="value">${fmtTime(d.uptime)}</span></div>
    <div class="item"><span class="label">Drones</span><span class="value">${Object.keys(d.drones).length}</span></div>
    <div class="item"><span class="label">Frontiers</span><span class="value">${Object.keys(d.frontiers).length}</span></div>
    <div class="item"><span class="label">Assigned</span><span class="value">${d.stats.total_assignments}</span></div>
    <div class="item"><span class="label">Reached</span><span class="value ok">${d.stats.goals_reached}</span></div>
    <div class="item"><span class="label">Failed</span><span class="value ${d.stats.goals_failed?'err':''}">${d.stats.goals_failed}</span></div>`;
  document.getElementById('hdr').innerHTML=h;

  // Drones
  const now=Date.now()/1000;
  let dt='';
  Object.keys(d.drones).sort().forEach(rid=>{
    const dr=d.drones[rid];
    const age=dr.updated?Math.round(now-dr.updated):'-';
    const sc='s-'+dr.state;
    dt+=`<tr><td><b>${rid}</b></td>
      <td><span class="state ${sc}">${dr.state}</span></td>
      <td>(${dr.x.toFixed(1)}, ${dr.y.toFixed(1)})</td>
      <td>${dr.assignment||'-'}</td>
      <td>${age}s</td></tr>`;
  });
  document.getElementById('dtbl').innerHTML=dt;

  // Performance
  const s=d.stats;
  const dhz=s.drone_state_hz, fhz=s.frontier_hz, ahz=s.assignment_hz;
  const dhzCls=dhz<1?'err':dhz<3?'warn':'ok';
  const fhzCls=fhz<0.5?'err':fhz<1?'warn':'ok';
  document.getElementById('perf').innerHTML=`
    <div class="perf-card"><div class="pv ${dhzCls}">${dhz}</div><div class="pl">Drone State Hz</div></div>
    <div class="perf-card"><div class="pv ${fhzCls}">${fhz}</div><div class="pl">Frontier Hz</div></div>
    <div class="perf-card"><div class="pv">${ahz}</div><div class="pl">Assignment Hz</div></div>
    <div class="perf-card"><div class="pv ok">${s.goals_reached}</div><div class="pl">Goals Reached</div></div>
    <div class="perf-card"><div class="pv ${s.goals_failed?'err':''}">${s.goals_failed}</div><div class="pl">Goals Failed</div></div>
    <div class="perf-card"><div class="pv">${s.total_assignments}</div><div class="pl">Total Assigned</div></div>`;

  // Frontiers
  const fkeys=Object.keys(d.frontiers).sort((a,b)=>(d.frontiers[b].size||0)-(d.frontiers[a].size||0)).slice(0,10);
  document.getElementById('fcnt').textContent=`(${Object.keys(d.frontiers).length} active)`;
  let ft='';
  fkeys.forEach(fid=>{
    const f=d.frontiers[fid];
    ft+=`<tr><td>${fid}</td><td>(${f.cx.toFixed(1)}, ${f.cy.toFixed(1)})</td><td>${f.size}</td></tr>`;
  });
  document.getElementById('ftbl').innerHTML=ft;

  // Events
  let el='';
  (d.events||[]).slice(-30).reverse().forEach(e=>{el+=`<div>${esc(e)}</div>`});
  document.getElementById('elog').innerHTML=el;
}

function fmtTime(s){const m=Math.floor(s/60);return m?m+'m '+(s%60)+'s':s+'s'}
function esc(s){const d=document.createElement('div');d.textContent=s;return d.innerHTML}
</script>
</body></html>"""


class _Handler(BaseHTTPRequestHandler):
    """Serves the dashboard HTML and SSE stream."""

    def log_message(self, fmt, *args):
        pass  # Suppress default access logs

    def do_GET(self):
        if self.path == '/api/stream':
            self._handle_sse()
        elif self.path == '/api/state':
            self._handle_json()
        else:
            self._handle_html()

    def _handle_html(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.end_headers()
        self.wfile.write(_HTML.encode())

    def _handle_json(self):
        with _lock:
            payload = json.dumps(_state)
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(payload.encode())

    def _handle_sse(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        try:
            while True:
                with _lock:
                    payload = json.dumps(_state)
                self.wfile.write(f'data: {payload}\n\n'.encode())
                self.wfile.flush()
                time.sleep(0.5)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()

    node.declare_parameter('host', '0.0.0.0')
    node.declare_parameter('port', 8080)
    host = node.get_parameter('host').value
    port = node.get_parameter('port').value

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    server = HTTPServer((host, port), _Handler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    node.get_logger().info(f'Web dashboard serving on http://localhost:{port}')

    try:
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
