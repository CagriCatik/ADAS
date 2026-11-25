"""Acoustic pulse scheduling and interference quality flags.

Overlapping visual FoVs alone do NOT cause interference. Simulated crosstalk
requires overlapping transmit/echo time windows on neighboring channels. It
produces INVALID data instead of fabricating parking-space measurements.
"""
from __future__ import annotations
from dataclasses import dataclass,replace
from .ultrasound_12 import Echo

@dataclass(frozen=True)
class Pulse:
    channel:str
    start_s:float

@dataclass(frozen=True)
class AcousticSchedule:
    sound_speed_m_s:float=343.
    range_m:float=4.5
    guard_s:float=.003

    def flight_window_s(self)->float:
        if self.sound_speed_m_s<=0 or self.range_m<=0 or self.guard_s<0:
            raise ValueError("invalid acoustic schedule")
        return 2*self.range_m/self.sound_speed_m_s+self.guard_s

    def tdma(self,ids):
        period=self.flight_window_s()
        return tuple(Pulse(cid,i*period) for i,cid in enumerate(ids))

    def simultaneous(self,ids):
        return tuple(Pulse(cid,0.) for cid in ids)

def invalidate_interference(echoes:tuple[Echo,...],pulses:tuple[Pulse,...],
                            schedule:AcousticSchedule=AcousticSchedule())->tuple[Echo,...]:
    times={p.channel:p.start_s for p in pulses}
    if len(times)!=len(pulses):
        raise ValueError("duplicate transmitter")
    invalid=set()
    for i,e in enumerate(echoes):
        if e.id not in times:continue
        for j,other in enumerate(echoes):
            if i==j or other.id not in times:continue
            if e.id[0]!=other.id[0]:continue
            if abs(int(e.id[1:])-int(other.id[1:]))>1:continue
            if abs(times[e.id]-times[other.id])<schedule.flight_window_s():
                invalid.update((e.id,other.id))
    return tuple(replace(e,status="INTERFERENCE",range_m=None,hit_x=None,hit_y=None)
                 if e.id in invalid else e for e in echoes)
