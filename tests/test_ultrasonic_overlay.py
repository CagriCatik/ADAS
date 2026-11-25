"""Visual regression tests for 12 permanently shown ultrasonic channels."""
from dataclasses import asdict
import pytest
from adas_sim.geometry import Pose
from adas_sim.ultrasound_12 import scan_12
from adas_sim.ultrasonic_overlay import UltrasonicOverlay,status_table

def echoes():
    return [asdict(e) for e in scan_12(Pose(0,0,0),())]

def test_12_mounted_markers_and_optional_fields_of_view():
    from matplotlib import pyplot as plt
    fig,ax=plt.subplots()
    try:
        overlay=UltrasonicOverlay(ax)
        measurements=echoes()
        overlay.update(measurements,active_id="F03")
        assert len(overlay.channels)==12
        assert [c.label.get_text() for c in overlay.channels]==[
            *(f"F{i:02}" for i in range(1,5)),
            *(f"R{i:02}" for i in range(1,5)), "SLF", "SRF", "SLR", "SRR"]
        assert all(c.marker.get_visible() for c in overlay.channels)
        assert all(c.footprint.get_visible() for c in overlay.channels)
        assert sum(c.marker.get_linewidth()>1 for c in overlay.channels)==1
        overlay.update(measurements,active_id="SRR",show_fovs=False)
        assert all(c.marker.get_visible() for c in overlay.channels)
        assert all(not c.footprint.get_visible() for c in overlay.channels)
    finally:
        plt.close(fig)

def test_interference_cannot_create_an_echo_pulse():
    from matplotlib import pyplot as plt
    fig,ax=plt.subplots()
    try:
        overlay=UltrasonicOverlay(ax)
        measurements=echoes()
        measurements[0]["status"]="INTERFERENCE"
        measurements[0]["range_m"]=None
        overlay.update(measurements,active_id=None)
        assert not overlay.active_field.get_visible()
        assert not overlay.echo_point.get_visible()
        assert "X-TALK" in status_table(measurements,None)[0]
    finally:
        plt.close(fig)

def test_incomplete_sensor_frame_is_rejected():
    from matplotlib import pyplot as plt
    fig,ax=plt.subplots()
    try:
        with pytest.raises(ValueError,match="12 distinct"):
            UltrasonicOverlay(ax).update(echoes()[:11],active_id="F01")
    finally:
        plt.close(fig)
