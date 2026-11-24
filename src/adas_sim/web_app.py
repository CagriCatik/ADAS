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

_HTML = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ADAS · Planar Parking Workbench</title>
<style>
:root{font-family:system-ui,sans-serif;color:#e2e8f0;background:#0b1220}
body{max-width:1250px;margin:auto;padding:24px}
h1{font-weight:650}header{display:flex;gap:16px;align-items:center;flex-wrap:wrap}
select,button,input{font:inherit;padding:8px;background:#1e293b;color:#f8fafc;border:1px solid #475569;border-radius:6px}
main{display:grid;grid-template-columns:minmax(0,1fr) 240px;gap:18px}
canvas{width:100%;aspect-ratio:16/10;background:#eaf0f5;border-radius:10px}
aside{background:#19263b;padding:16px;border-radius:10px}
@media(max-width:750px){main{grid-template-columns:1fr}}
</style></head><body><h1>ADAS · 2D Planar Parking Workbench</h1>
<header><label>Scenario <select id="scene"></select></label>
<button id="run">Load & replay</button><button id="play">Pause</button>
<label>Step <input id="seek" type="range" min="0" value="0"></label></header>
<p>Simulated plans only · fixed camera 29 px/m · rear-axle pose · four-wheel vehicle drawing</p>
<main><canvas id="world" width="1000" height="625"></canvas>
<aside><h3>Telemetry</h3><pre id="stats">Select a scenario.</pre></aside></main>
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
 rect(p.x+shift*c,p.y+shift*s,p.yaw,v.length,v.width,"#c7d2fe","#2563eb");
 for(const axle of [0,v.wheelbase])for(const side of [-.8,.8]){
   const x=p.x+axle*c-side*s,y=p.y+axle*s+side*c;
   rect(x,y,p.yaw+(axle?steer:0),.60,.225,"#1e293b","#f8fafc");
 }
}
function draw(){
 ctx.clearRect(0,0,1000,625);ctx.fillStyle="#eaf0f5";ctx.fillRect(0,0,1000,625);
 if(!data)return;
 data.obstacles.forEach(o=>rect(o.x,o.y,o.yaw,o.length,o.width,"#94a3b8"));
 const g=data.goal;rect(g.x,g.y,g.yaw,data.vehicle.length,data.vehicle.width,"#bbf7d0","#16a34a");
 ctx.strokeStyle="#f97316";ctx.lineWidth=2;ctx.beginPath();
 data.poses.slice(0,frame+1).forEach((p,i)=>{let [x,y]=map(p.x,p.y);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke();
 const p=data.poses[frame],cmd=data.commands[Math.min(frame,data.commands.length-1)];
 vehicle(p,cmd?cmd.steer:0);
 $("stats").textContent="Scenario: "+data.name+"\nStatus: "+data.status+
 "\nTime: "+(frame*data.step_dt).toFixed(2)+"s\nSpeed: "+(cmd?cmd.velocity.toFixed(2):0)+"m/s"+
 "\nSteer: "+(cmd?(cmd.steer*180/Math.PI).toFixed(1):0)+"deg\nFrame: "+(frame+1)+"/"+data.poses.length;
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
            body=json.dumps(list(SCENARIOS)).encode();ctype="application/json"
        elif path.startswith("/api/scenario/"):
            name=path.removeprefix("/api/scenario/")
            if name not in SCENARIOS:
                self.send_error(404, "Unknown scenario");return
            scene=SCENARIOS[name]
            route=plan(scene)
            body=json.dumps({"name":name,"status":route.reason,"step_dt":route.step_dt,
                             "poses":[asdict(p) for p in route.poses],
                             "commands":[asdict(c) for c in route.commands],
                             "vehicle":asdict(scene.vehicle),
                             "obstacles":[asdict(o) for o in scene.obstacles],
                             "goal":asdict(scene.goal)}).encode()
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
