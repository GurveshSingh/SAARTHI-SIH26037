#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import Twist
import numpy as np

_RAW_TABLE = [
    (1701, +0.0049694,  +0.0116746),
    (1711, +0.2274574,  +0.0979008),
    (1721, +0.1737052,  +0.1362236),
    (1731, +0.4751986,  +0.2596646),
    (1741, +0.1476481,  +0.0829669),
    (1751, +0.3071875,  +0.2271002),
    (1761, +0.4833515,  +0.3657181),
    (1771, +0.6804709,  +0.2407416),
    (1781, +0.5793779,  +0.4451320),
    (1791, +1.0248756,  +0.5320357),
    (1801, +1.1032788,  +0.5747312),
    (1811, +1.0711821,  +0.6207493),
    (1821, +1.2955393,  +0.6905345),
    (1831, +1.1781380,  +0.7251384),
    (1841, +1.0658966,  +0.7950879),
    (1851, +1.4582719,  +0.8642207),
    (1861, +1.4239148,  +0.8996943),
    (1871, +1.7182883,  +0.9620392),
    (1881, +1.5694072,  +1.0192515),
    (1891, +1.9333812,  +1.0672085),
    (1901, +1.9507320,  +1.1195627),
    (1911, +1.9800199,  +1.1534857),
    (1921, +2.0167842,  +1.1996787),
    (1931, +2.0566902,  +1.2736796),
    (1941, +2.2430718,  +1.3122753),
    (1951, +2.3966208,  +1.3747363),
    (1961, +2.4894938,  +1.4250761),
    (1971, +2.3577766,  +1.4557761),
    (1981, +2.5835660,  +1.4988349),
    (1991, +2.6570659,  +1.5663777),
    (2001, +2.7864964,  +1.6056126),
    (2011, +2.6721101,  +1.5999318),
    (2021, +2.7697933,  +1.6120573),
    (2031, +2.5703838,  +1.6245543),
    (2041, +2.7050591,  +1.5916198),
    (2051, +2.7829666,  +1.6222680),
    (2061, +2.6038892,  +1.6164773),
    (2071, +2.8462231,  +1.6217815),
    (2081, +2.7862916,  +1.6369924),
    (2091, +2.7955372,  +1.6103469),
    (2101, +2.5281668,  +1.6286706),
]

_PWM_NEUTRAL = 1501
_PWM_MIN_POS = 1701
_PWM_MAX     = 2101


def _build_segments():
    pwms = [r[0] for r in _RAW_TABLE]
    vels = [r[2] for r in _RAW_TABLE]

    mono_pwm = [pwms[0]]
    mono_vel = [vels[0]]
    running_max = vels[0]

    for p, v in zip(pwms[1:], vels[1:]):
        if v > running_max:
            running_max = v
            mono_pwm.append(p)
            mono_vel.append(v)

    segs = []
    for i in range(len(mono_pwm) - 1):
        segs.append((
            mono_vel[i],
            mono_vel[i + 1],
            mono_pwm[i],
            mono_pwm[i + 1],
        ))
    return segs

_SEGMENTS = _build_segments()
_VEL_MIN  = _SEGMENTS[0][0]
_VEL_MAX  = _SEGMENTS[-1][1]


def vel_to_pwm(angular_z: float) -> int:
    vel = float(angular_z)

    if vel <= _VEL_MIN:
        return _PWM_MIN_POS
    if vel >= _VEL_MAX:
        return _PWM_MAX

    lo, hi = 0, len(_SEGMENTS) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if _SEGMENTS[mid][1] < vel:
            lo = mid + 1
        else:
            hi = mid

    vel_lo, vel_hi, pwm_lo, pwm_hi = _SEGMENTS[lo]

    if vel_hi == vel_lo:
        return int(round(pwm_lo))

    t   = (vel - vel_lo) / (vel_hi - vel_lo)
    pwm = pwm_lo + t * (pwm_hi - pwm_lo)
    return int(round(float(np.clip(pwm, _PWM_MIN_POS, _PWM_MAX))))


def _linear_vel_to_pwm_gain(vel: float,
                             slope: float,
                             intercept: float,
                             pwm_max: int,
                             pwm_min: int) -> int:
    gain = slope * vel + intercept

    if gain > pwm_max - 1500:
        gain = pwm_max - 1500

    if gain < 0:
        gain = 0

    return int(round(gain))


class CmdVelToPwmNode(Node):

    def __init__(self):
        super().__init__('az_vel_to_pwm')

        self.declare_parameter('pwm_channel',      0)
        self.declare_parameter('deadband_vel',     0.03)
        self.declare_parameter('publish_rate',     50.0)
        self.declare_parameter('invert_direction', False)

        self._channel  = self.get_parameter('pwm_channel').value
        self._deadband = self.get_parameter('deadband_vel').value
        self._rate_hz  = self.get_parameter('publish_rate').value
        self._invert   = self.get_parameter('invert_direction').value

        self.declare_parameter('ch_throttle',        1)
        self.declare_parameter('pwm_min',            1051)
        self.declare_parameter('pwm_max',            1951)
        self.declare_parameter('pwm_neutral',        1501)
        self.declare_parameter('slope_linear',       314)
        self.declare_parameter('intercept_linear',   140)
        self.declare_parameter('linear_vel_max',     1.0)

        self._ch_throttle        = self.get_parameter('ch_throttle').value
        self._pwm_min            = self.get_parameter('pwm_min').value
        self._pwm_max            = self.get_parameter('pwm_max').value
        self._pwm_neutral        = self.get_parameter('pwm_neutral').value
        self._slope_linear       = self.get_parameter('slope_linear').value
        self._intercept_linear   = self.get_parameter('intercept_linear').value
        self._linear_vel_max     = self.get_parameter('linear_vel_max').value

        self.declare_parameter('cmd_timeout', 0.2)
        self._cmd_timeout = self.get_parameter('cmd_timeout').value

        self._latest_pwm: int          = _PWM_NEUTRAL
        self._latest_throttle_pwm: int = self._pwm_neutral
        self._last_cmd_time: float     = 0.0

        self._rc_pub = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            10
        )

        self._cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel1',
            self._cmd_vel_callback,
            10
        )

        self._timer = self.create_timer(
            1.0 / self._rate_hz,
            self._publish_rc
        )

        self.get_logger().info(
            f'[az_vel_to_pwm] ready | '
            f'steering CH{self._channel} deadband={self._deadband} rad/s | '
            f'throttle CH{self._ch_throttle} slope={self._slope_linear} intercept={self._intercept_linear} | '
            f'rate={self._rate_hz} Hz | '
            f'cmd_timeout={self._cmd_timeout} s | '
            f'PWM stop={_PWM_NEUTRAL} pos_start={_PWM_MIN_POS} max={_PWM_MAX} | '
            f'{len(_SEGMENTS)} piecewise segments'
        )

    def _cmd_vel_callback(self, msg: Twist):
        self._last_cmd_time = time.monotonic()

        pwm_gain_x = _linear_vel_to_pwm_gain(
            abs(msg.linear.x),
            self._slope_linear,
            self._intercept_linear,
            self._pwm_max,
            self._pwm_min,
        )

        if msg.linear.x > 0.0:
            self._latest_throttle_pwm = 1500 + pwm_gain_x
        elif msg.linear.x < 0.0:
            self._latest_throttle_pwm = 1500 - pwm_gain_x
        elif msg.linear.x < 10 ** (-3):
            self._latest_throttle_pwm = 1500
        else:
            self._latest_throttle_pwm = 1500

        az = msg.angular.z

        if self._invert:
            az = -az

        if abs(az) < self._deadband:
            self._latest_pwm = _PWM_NEUTRAL

        elif az >= 0.0:
            self._latest_pwm = vel_to_pwm(az)

        else:
            pos_pwm = vel_to_pwm(-az)
            delta   = pos_pwm - _PWM_MIN_POS
            self._latest_pwm = max(1000, _PWM_NEUTRAL - delta)

    def _publish_rc(self):
        if self._last_cmd_time == 0.0:
            return

        if (time.monotonic() - self._last_cmd_time) > self._cmd_timeout:
            return

        msg      = OverrideRCIn()
        channels = [65535] * 18
        channels[self._channel]     = self._latest_pwm
        channels[self._ch_throttle] = self._latest_throttle_pwm
        msg.channels = channels
        self._rc_pub.publish(msg)


def _print_diagnostic_table():
    print(f"\n{'Vel (rad/s)':>14}  {'PWM':>6}")
    print("-" * 25)
    for v in np.linspace(0.0, _VEL_MAX + 0.1, 40):
        print(f"{v:>14.6f}  {vel_to_pwm(v):>6}")

    print(f"\n{len(_SEGMENTS)} piecewise segments (vel_lo → vel_hi : pwm_lo → pwm_hi):")
    for i, (vl, vh, pl, ph) in enumerate(_SEGMENTS):
        print(f"  [{i:02d}]  {vl:.6f} → {vh:.6f}  :  {pl} → {ph}")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPwmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import sys
    if '--diag' in sys.argv:
        _print_diagnostic_table()
    else:
        main()
