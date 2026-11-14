"""The surviving single-step helpers: bounded envelope, telemetry, stationarity.

prepare_step (depth/obstacle/candidate gating) was deleted with the D435i chain.
"""
import unittest
from vlx_async.fusion_step import StepLimits,stationary_evidence,validate_telemetry


def telemetry(now=1_000_000_000, boot='boot', **changes):
    data=dict(status='running',host_boot_id=boot,timestamp_ns=now,
              sport=dict(received_monotonic_ns=now,mode=3,error_code=0,
                         position=[0.,0.,0.],rpy=[0.,0.,0.],velocity=[0.,0.,0.],yaw_speed=0.),
              low=dict(received_monotonic_ns=now,mode_machine=1),
              remote_axes=dict(lx=0.,ly=0.,rx=0.,ry=0.,keys=0))
    data.update(changes)
    return data


class FusionStepTests(unittest.TestCase):
    def test_stationarity_requires_time_and_real_stillness(self):
        s=dict(position=[0,0,0],velocity=[0,0,0],rpy=[0,0,0],yaw_speed=0.)
        history=[(i*100_000_000,s) for i in range(25)]
        self.assertTrue(stationary_evidence(history))
        self.assertFalse(stationary_evidence(history[:10]))
        self.assertFalse(stationary_evidence(history[:-1]+[(history[-1][0],dict(s,velocity=[.1,0,0]))]))

    def test_remote_input_and_stale_telemetry_reject(self):
        now=1_000_000_000
        self.assertTrue(validate_telemetry(telemetry(now),now,'boot',StepLimits()))
        bad=[telemetry(now,remote_axes=dict(lx=0.,ly=0.,rx=0.,ry=0.,keys=1)),
             telemetry(now-300_000_000),
             telemetry(now,status='finished'),
             telemetry(now,boot='other')]
        for data in bad:
            with self.assertRaises(ValueError):validate_telemetry(data,now,'boot',StepLimits())

    def test_cannot_expand_step_bounds(self):
        for change in [dict(duration_s=3),dict(max_speed_mps=.1),dict(max_displacement_m=.3)]:
            with self.assertRaises(ValueError):StepLimits(**change)

    def test_nonfinite_angular_velocity_rejected(self):
        now=1_000_000_000
        data=telemetry(now)
        data['sport'].update(position=[0.,0.,0.],velocity=[0.,0.,0.],yaw_speed=float('nan'))
        with self.assertRaisesRegex(ValueError,'invalid_yaw_speed'):
            validate_telemetry(data,now,'boot',StepLimits())
        samples=[(i*100_000_000,data['sport']) for i in range(25)]
        self.assertFalse(stationary_evidence(samples))


if __name__=='__main__':unittest.main()
