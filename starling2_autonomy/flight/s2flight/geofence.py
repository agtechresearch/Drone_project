"""지오펜스 — v14 DroneController._geofence_check 를 상태 객체로 분리. 판정 논리는 동일하다.

두 겹으로 본다.
  1) 추종 오차 펜스 (주 방어선, 항상 켜짐)
     실제 위치가 명령 setpoint 에서 얼마나 벌어졌는가. 미션이 홈에서 얼마나 멀리
     가든 무관하게 "위치 폭주"만 잡아낸다.
  2) 홈 기준 절대 펜스 (기본 꺼짐)
     운용 구역 자체를 벗어났는가. max_home_dist / max_alt_agl 로 켠다.

speed 미지정 이동은 setpoint 가 목적지로 즉시 점프한다. 이때 추종 오차가 커지는 것은
정상이므로, 새 setpoint 에 수렴할 때까지 1) 만 유예한다(jump_thresh / jump_timeout).
단발 튐으로 착륙시키지 않도록 violation_sec 동안 연속 위반일 때만 발동한다.
"""

import math

from .util import log


class Geofence(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.armed = False
        self._violation_since = None
        self._last_target = None
        self._jump_pending = False
        self._jump_since = 0.0

    def arm(self, target):
        """offboard 가 실제로 붙은 다음에 부른다. target = (n, e, d)."""
        self._last_target = tuple(target)
        self._violation_since = None
        self._jump_pending = False
        self.armed = not self.cfg.no_geofence

    def disarm(self):
        self.armed = False

    def describe(self):
        c = self.cfg
        return ("track h<={:.2f}m v<={:.2f}m, persist {:.1f}s, home_dist<={}, alt_agl<={}".format(
            c.max_horizontal_dev, c.max_alt_deviation, c.geofence_violation_sec,
            "{:.2f}m".format(c.max_home_dist) if c.max_home_dist > 0 else "off",
            "{:.2f}m".format(c.max_alt_agl) if c.max_alt_agl > 0 else "off"))

    def check(self, now, target, actual, home):
        """위반 사유 문자열을 반환한다. 펜스 안이면 None.

        target, actual, home 은 모두 NED (n, e, d) 튜플이다.
        """
        cfg = self.cfg
        if cfg.no_geofence or not self.armed:
            return None

        tgt_key = (target[0], target[1], target[2])
        if self._last_target is not None:
            jump = math.sqrt(
                (tgt_key[0] - self._last_target[0]) ** 2
                + (tgt_key[1] - self._last_target[1]) ** 2
                + (tgt_key[2] - self._last_target[2]) ** 2)
            if jump > cfg.geofence_jump_thresh:
                self._jump_pending = True
                self._jump_since = now
                self._violation_since = None
        self._last_target = tgt_key

        max_h = cfg.max_horizontal_dev
        max_v = cfg.max_alt_deviation
        h_dev = math.sqrt((actual[0] - target[0]) ** 2 + (actual[1] - target[1]) ** 2)
        v_dev = abs(actual[2] - target[2])

        track_reason = None
        if h_dev > max_h:
            track_reason = "tracking horizontal {:.2f}m > {:.2f}m".format(h_dev, max_h)
        elif v_dev > max_v:
            track_reason = "tracking altitude {:.2f}m > {:.2f}m".format(v_dev, max_v)

        if self._jump_pending:
            if track_reason is None:
                self._jump_pending = False          # 새 setpoint 로 수렴 완료
            elif now - self._jump_since > cfg.geofence_jump_timeout:
                return ("GEOFENCE: failed to converge within {:.0f}s after setpoint jump "
                        "({})".format(cfg.geofence_jump_timeout, track_reason))
            else:
                track_reason = None                 # 유예 중

        # 절대 펜스는 유예 중에도 계속 감시한다.
        abs_reason = None
        if cfg.max_home_dist > 0.0:
            home_dist = math.sqrt((actual[0] - home[0]) ** 2 + (actual[1] - home[1]) ** 2)
            if home_dist > cfg.max_home_dist:
                abs_reason = "home distance {:.2f}m > {:.2f}m".format(home_dist, cfg.max_home_dist)
        if abs_reason is None and cfg.max_alt_agl > 0.0:
            agl = home[2] - actual[2]               # NED: down 이 줄면 상승
            if agl > cfg.max_alt_agl:
                abs_reason = "altitude AGL {:.2f}m > {:.2f}m".format(agl, cfg.max_alt_agl)

        reason = track_reason or abs_reason
        if reason is None:
            self._violation_since = None
            return None

        if self._violation_since is None:
            self._violation_since = now
            log("       [geofence] WARN {} (must persist {:.1f}s to trigger)".format(
                reason, cfg.geofence_violation_sec))
            return None
        if now - self._violation_since < cfg.geofence_violation_sec:
            return None
        return "GEOFENCE: {} sustained {:.1f}s".format(reason, cfg.geofence_violation_sec)
