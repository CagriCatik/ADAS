"""Local, read-only interactive 2D scenario viewer using Python's HTTP server.

The frontend previews a fully generated and validated plan; it does not
simulate sensors or provide remote control of a real vehicle.
"""
from __future__ import annotations
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from importlib.resources import files
from urllib.parse import urlsplit
from .planner import plan
from .scenarios import SCENARIOS
from .ultrasound_12 import scan_12
from .sensor_demo import CASES,build_case_payload

_HTML = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ADAS · Planar Parking Workbench</title>
<style>
:root{font-family:system-ui,sans-serif;color:#e2e8f0;background:#0b1220}
body{max-width:1250px;margin:auto;padding:24px}
h1{font-weight:650}header{display:flex;gap:16px;align-items:center;flex-wrap:wrap}
select,button,input{font:inherit;padding:8px;background:#1e293b;color:#f8fafc;border:1px solid #475569;border-radius:6px}
main{display:grid;grid-template-columns:minmax(0,1fr) 280px;gap:18px}
canvas{width:100%;aspect-ratio:16/10;background:#0f172a;border:1px solid #334155;border-radius:10px}
aside{background:#19263b;padding:16px;border-radius:10px}
#sensorGrid{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:10px}
.sensor{background:#0f172a;border:1px solid #334155;border-radius:5px;padding:7px;font:12px monospace}
.sensor.hit{border-color:#f59e0b;color:#fbbf24}.sensor.clear{color:#94a3b8}.sensor.fail{border-color:#f43f5e;color:#f43f5e}
@media(max-width:750px){main{grid-template-columns:1fr}}
</style></head><body><h1>ADAS · 2D Planar Parking Workbench</h1>
<header><label>Scenario <select id="scene"></select></label>
<button id="run">Load & replay</button><button id="play">Pause</button>
<label>Step <input id="seek" type="range" min="0" value="0"></label><label><input id="beams" type="checkbox" checked> Ultrasound FoV (12 ch)</label></header>
<p>Sensor-first simulation: survey → measured gap → size/quality decision → plan only when permitted. One highlighted TDMA transmitter; interference is a measurement fault, not overlapping FoV.</p>
<main><canvas id="world" width="1000" height="625"></canvas>
<aside><h3>Telemetry</h3><pre id="stats">Select a scenario.</pre><h3>Ultrasonic channels (6 front / 6 rear)</h3><div id="sensorGrid"></div><p style="font-size:12px;color:#94a3b8">Idealized ray-cast echoes; not calibrated OEM sensor data.</p></aside></main>
<script>
const $ = id=>document.getElementById(id);
const ctx=$("world").getContext("2d");
let data=null,frame=0,playing=true;
const camera={ppm:29,centerX:6,centerY:0};
function map(x,y){return [500+(x-camera.centerX)*camera.ppm,312-(y-camera.centerY)*camera.ppm]}
function polygon(points,fill,stroke="#334155"){ctx.beginPath();points.forEach((p,i)=>{const q=map(...p);i?ctx.lineTo(...q):ctx.moveTo(...q)});ctx.closePath();ctx.fillStyle=fill;ctx.fill();ctx.strokeStyle=stroke;ctx.lineWidth=2;ctx.stroke()}
function rect(cx,cy,yaw,length,width,fill,stroke){const c=Math.cos(yaw),s=Math.sin(yaw);polygon([[length/2,width/2],[length/2,-width/2],[-length/2,-width/2],[-length/2,width/2]].map(([x,y])=>[cx+c*x-s*y,cy+s*x+c*y]),fill,stroke)}
function vehicle(p,steer){
 const v=data.vehicle,c=Math.cos(p.yaw),s=Math.sin(p.yaw);
 const shift=v.length/2-v.rear_axle_from_center;
 rect(p.x+shift*c,p.y+shift*s,p.yaw,v.length,v.width,"#155e75","#38bdf8");
 for(const axle of [0,v.wheelbase])for(const side of [-.8,.8]){
   const x=p.x+axle*c-side*s,y=p.y+axle*s+side*c;
   rect(x,y,p.yaw+(axle?steer:0),.60,.225,"#080d1c","#f8fafc");
 }
}
function sensors(p){ 
 const echoes=data.ultrasound[frame]||[];
 if($("beams").checked && data.tx_mode==="tdma")echoes.filter((e,i)=>i===frame%12).forEach(e=>{
   const start=e.heading_rad-e.opening_angle_rad/2;
   const end=e.heading_rad+e.opening_angle_rad/2;
   const n=16;
   const left=map(e.origin_x,e.origin_y);
   ctx.beginPath();ctx.moveTo(...left);
   for(let k=0;k<=n;k++){
     const a=start+(end-start)*k/n;
     const pt=map(e.origin_x+e.max_range_m*Math.cos(a),e.origin_y+e.max_range_m*Math.sin(a));
     ctx.lineTo(...pt);
   }
   ctx.closePath();
   ctx.fillStyle=e.status==="DETECTED"?"rgba(245,158,11,0.13)":"rgba(56,189,248,0.07)";
   ctx.fill();
   ctx.strokeStyle="rgba(56,189,248,0.22)";ctx.lineWidth=1;ctx.stroke();
   ctx.beginPath();ctx.arc(...left,3,0,Math.PI*2);ctx.fillStyle="#38bdf8";ctx.fill();
   if(e.status==="DETECTED"&&e.hit_x!==null){
     const hit=map(e.hit_x,e.hit_y);
     ctx.beginPath();ctx.moveTo(...left);ctx.lineTo(...hit);
     ctx.strokeStyle="#f59e0b";ctx.lineWidth=2;ctx.stroke();
     ctx.beginPath();ctx.arc(...hit,4,0,Math.PI*2);ctx.fillStyle="#fbbf24";ctx.fill();
   }
 });
 $("sensorGrid").replaceChildren(...echoes.map(e=>{
   const div=document.createElement("div");
   div.className="sensor "+(e.status==="DETECTED"?"hit":(e.status==="DROPOUT"||e.status==="INTERFERENCE")?"fail":"clear");
   div.textContent=e.id+"  "+(e.range_m===null?"--":e.range_m.toFixed(2)+" m");
   div.title=e.status;
   return div;
 }));
}
function draw(){
 ctx.clearRect(0,0,1000,625);ctx.fillStyle="#0f172a";ctx.fillRect(0,0,1000,625);
 if(!data)return;
 data.obstacles.forEach(o=>rect(o.x,o.y,o.yaw,o.length,o.width,"#64748b","#94a3b8"));
 sensors(data.poses[frame]);
 const g=data.goal;if(g && frame>=data.survey_samples-1)rect(g.x,g.y,g.yaw,data.vehicle.length,data.vehicle.width,"#064e3b","#10b981");
 ctx.strokeStyle="#d95926";ctx.lineWidth=2;ctx.beginPath();
 data.poses.slice(0,frame+1).forEach((p,i)=>{let [x,y]=map(p.x,p.y);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke();
 const p=data.poses[frame],cmd=data.commands[Math.min(frame,data.commands.length-1)];
 vehicle(p,cmd?cmd.steer:0);
 const finishedScan=frame>=data.survey_samples-1;
 const decision=finishedScan?data.status:"SEARCHING";
 const details=finishedScan?data.reason:"collecting ultrasonic range observations";
 const slotText=finishedScan && data.slot?data.slot.certified_length_m.toFixed(2)+" m":"--";
 $("stats").textContent="Case: "+data.name+"\\nState: "+decision+
 "\\nReason: "+details+"\\nPhase: "+data.phases[frame]+
 "\\nTime: "+data.times[frame].toFixed(2)+"s"+
 "\\nSlot: "+slotText+"\\nTX mode: "+data.tx_mode+
 "\\nSpeed: "+(cmd?cmd.velocity.toFixed(2):0)+"m/s"+
 "\\nSteer: "+(cmd?(cmd.steer*180/Math.PI).toFixed(1):0)+"deg"+
 "\\nFrame: "+(frame+1)+"/"+data.poses.length;
}
async function load(){
 $("stats").textContent="Planning...";
 const r=await fetch("/api/scenario/"+encodeURIComponent($("scene").value));
 if(!r.ok){$("stats").textContent="Planning failed";return}
 data=await r.json();frame=0;$("seek").max=Math.max(0,data.poses.length-1);$("seek").value=0;playing=true;$("play").textContent="Pause";draw();
}
async function init(){
 const names=await (await fetch("/api/scenarios")).json();
 names.forEach(x=>{let o=document.createElement("option");o.textContent=x;o.value=x;$("scene").appendChild(o)});
 await load();
}
$("beams").onchange=draw;
$("run").onclick=load;$("play").onclick=()=>{playing=!playing;$("play").textContent=playing?"Pause":"Play"};
$("seek").oninput=()=>{frame=Number($("seek").value);playing=false;$("play").textContent="Play";draw()};
setInterval(()=>{if(!playing||!data||frame>=data.poses.length-1)return;frame=Math.min(data.poses.length-1,frame+3);$("seek").value=frame;draw()},80);
init().catch(e=>$("stats").textContent=String(e));
</script></body></html>"""

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/":
            body = _HTML.encode("utf-8");ctype="text/html; charset=utf-8"
        elif path == "/api/scenarios":
            body=json.dumps(list(CASES)).encode();ctype="application/json"
        elif path.startswith("/api/scenario/"):
            name=path.removeprefix("/api/scenario/")
            if name not in CASES:
                self.send_error(404, "Unknown case");return
            report=build_case_payload(name)
            frames=report["frames"]
            report["poses"]=[f["pose"] for f in frames]
            report["ultrasound"]=[f["echoes"] for f in frames]
            report["commands"]=[{"steer":f["front_steer_rad"],"velocity":f["velocity_m_s"]}
                                for f in frames]
            report["times"]=[f["time_s"] for f in frames]
            report["phases"]=[f["stage"] for f in frames]
            report["goal"]=report["slot"]["goal"] if report["slot"] else None
            report["step_dt"]=0.
            body=json.dumps(report).encode()
            ctype="application/json"
        else:
            self.send_error(404);return
        self.send_response(200)
        self.send_header("Content-Type",ctype)
        self.send_header("Content-Length",str(len(body)))
        self.send_header("Cache-Control","no-store")
        self.end_headers();self.wfile.write(body)

def main():
    import argparse
    p=argparse.ArgumentParser()
    p.add_argument("--host",default="127.0.0.1")
    p.add_argument("--port",type=int,default=8766)
    args=p.parse_args()
    httpd=ThreadingHTTPServer((args.host,args.port),Handler)
    print(f"ADAS viewer: http://{args.host}:{args.port}/",flush=True)
    try:httpd.serve_forever()
    except KeyboardInterrupt:pass
    finally:httpd.server_close()

if __name__ == "__main__":
    main()
