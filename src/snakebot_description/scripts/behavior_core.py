"""Simulation-time biological behavior, pulse tongue, and mapped navigation."""

from collections import deque
import math
import random


BODY_JOINTS = [f'joint_{i}' for i in range(1, 9)]
ALL_JOINTS = BODY_JOINTS + ['head_pitch_joint', 'tongue_joint']


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


class Blend:
    """Quintic transition with zero endpoint velocity/acceleration."""
    def __init__(self, start, target, now, duration=1.0):
        self.start, self.target, self.time = dict(start), dict(target), now
        self.duration = duration

    def sample(self, now):
        t = clamp((now - self.time)/self.duration, 0.0, 1.0)
        weight = t*t*t*(10.0 + t*(-15.0 + 6.0*t))
        derivative = 30.0*t*t*(1.0-t)*(1.0-t)/self.duration if t < 1 else 0.0
        values = {k: self.start[k] + (v-self.start[k])*weight for k, v in self.target.items()}
        rates = {k: (v-self.start[k])*derivative for k, v in self.target.items()}
        return values, rates


class FlickTrain:
    """3–5 explicit 0.1 / 0.05 / 0.1 s pulses within a two-second alert."""
    def __init__(self, rng):
        self.count = rng.randint(3, 5)
        self.pulses = []
        self.pauses = []
        time = 0.0
        budget = 1.95 - self.count*0.25
        for index in range(self.count):
            self.pulses.append((time, rng.uniform(0.06, 0.12)))
            if index < self.count-1:
                remaining = self.count-index-2
                pause = rng.uniform(0.1, min(0.3, budget-remaining*0.1))
                self.pauses.append(pause)
                budget -= pause
                time += 0.25 + pause

    def sample(self, elapsed):
        for start, length in self.pulses:
            t = elapsed-start
            if 0 <= t < 0.1:
                return length*t/0.1, length/0.1, 'EXTEND'
            if 0.1 <= t < 0.15:
                return length, 0.0, 'HOLD'
            if 0.15 <= t < 0.25:
                return length*(0.25-t)/0.1, -length/0.1, 'RETRACT'
        return 0.0, 0.0, 'PAUSE'


class Biology:
    SPECIAL = ('COILED', 'WRAP')
    TARGETS = {
        'ROAM': {'amplitude': 0.4, 'curvature': 0.0, 'head': 0.0, 'speed': 1.0, 'frequency': 0.7, 'look': 0.0},
        'REST': {'amplitude': 0.2, 'curvature': 0.08, 'head': -0.1, 'speed': 0.0, 'frequency': 0.7, 'look': 0.0},
        'ALERT': {'amplitude': 0.35, 'curvature': 0.0, 'head': 0.8, 'speed': 0.0, 'frequency': 0.7, 'look': 0.0},
        'LOW_CRAWL': {'amplitude': 0.32, 'curvature': 0.0, 'head': -0.2, 'speed': 0.5, 'frequency': 0.7, 'look': 0.0},
        'COILED': {'amplitude': 0.0, 'curvature': 0.65, 'head': 0.2, 'speed': 0.0, 'frequency': 0.7, 'look': 0.0},
        'WRAP': {'amplitude': 0.6, 'curvature': 0.0, 'head': 0.65, 'speed': 1.0, 'frequency': 1.0, 'look': 1.0},
    }

    def __init__(self, seed=42, frequency=0.7, transition=0.3):
        self.rng = random.Random(seed)
        self.flick_rng = random.Random(seed+1009)
        self.frequency, self.transition = frequency, transition
        self.state, self.started, self.until = 'ROAM', None, math.inf
        self.next_choice = None
        self.blend = self.head_blend = None
        self.flick = None
        self.alert_number = 0
        self.transition_duration = transition

    def choose(self):
        draw = self.rng.random()
        if draw < 0.1:
            return 'COILED' if self.rng.random() < 0.3 else 'REST'
        if draw < 0.4:
            return 'ALERT'
        if draw < 0.6:
            return 'LOW_CRAWL'
        return 'ROAM'

    def sample(self, now):
        values, rates = self.blend.sample(now)
        head, head_rate = self.head_blend.sample(now)
        values.update(head)
        rates.update(head_rate)
        return values, rates

    def enter(self, state, now):
        initial = self.sample(now)[0] if self.blend else {k: 0.0 for k in self.TARGETS['ROAM']}
        duration = 1.0 if state in self.SPECIAL or self.state in self.SPECIAL or self.blend is None else self.transition
        target = dict(self.TARGETS[state])
        if state != 'WRAP':
            target['frequency'] = self.frequency
        self.state, self.started = state, now
        self.transition_duration = duration
        self.blend = Blend(initial, target, now, duration)
        # A 0.3 s 0.8-rad pitch change would exceed the URDF's 3 rad/s limit.
        head_duration = max(duration, 1.875*abs(target['head']-initial['head'])/2.6)
        self.head_blend = Blend({'head': initial['head']}, {'head': target['head']}, now, head_duration)
        lifetime = self.rng.uniform(4.0, 6.0) if state == 'COILED' else {
            'REST': 2.0, 'ALERT': 2.0, 'LOW_CRAWL': 2.0, 'WRAP': 4.0}.get(state, math.inf)
        self.until = now+lifetime
        self.flick = None
        if state == 'ALERT':
            self.flick = FlickTrain(self.flick_rng)
            self.alert_number += 1

    def update(self, now, hold_choices=False):
        if self.blend is None:
            self.enter('ROAM', now)
            self.next_choice = now+self.rng.uniform(4.0, 8.0)
        if self.state in self.SPECIAL:
            if now >= self.until:
                self.enter('ROAM', now)
                self.next_choice = now+self.rng.uniform(4.0, 8.0)
        elif now >= self.until:
            self.enter('ROAM', now)
        elif now >= self.next_choice and not hold_choices:
            self.enter(self.choose(), now)
            self.next_choice = now+self.rng.uniform(4.0, 8.0)
        values, rates = self.sample(now)
        tongue, tongue_rate, phase = (0.0, 0.0, 'RETRACTED')
        if self.state == 'ALERT' and self.flick:
            tongue, tongue_rate, phase = self.flick.sample(now-self.started)
        return values, rates, tongue, tongue_rate, phase


def ease(value, rate, target, dt, duration=0.3):
    """Continuous retargetable ease-in-out, ~96% settled after duration."""
    omega = 5.0/duration
    delta = value-target
    temp = (rate+omega*delta)*dt
    decay = math.exp(-omega*dt)
    return target+(delta+temp)*decay, (rate-omega*temp)*decay


def local_centers(joints):
    points = [(0.0, 0.0, 0.105)]
    yaw, pitch = joints.get('joint_1', 0.0), joints.get('head_pitch_joint', 0.0)
    reach = 0.11*math.cos(pitch)
    points.append((0.095+reach*math.cos(yaw), reach*math.sin(yaw), 0.13))
    x, y, a = -0.095, 0.0, 0.0
    for index in range(2, 9):
        a += joints.get(f'joint_{index}', 0.0)
        points.append((x-0.075*math.cos(a), y-0.075*math.sin(a), 0.105))
        x -= 0.17*math.cos(a)
        y -= 0.17*math.sin(a)
    return points


class Navigation:
    """Mapped full-body avoidance and a radius-aware four-second WRAP arc."""
    def __init__(self, obstacles, seed=42, linear_min=0.1, linear_max=0.35,
                 angular_max=0.6, dodge_linear=-0.15, dodge_angular=1.5):
        self.obstacles = [(float(o['x']), float(o['y']), float(o['radius'])) for o in obstacles]
        self.rng = random.Random(seed+31)
        self.linear_min, self.linear_max, self.angular_max = linear_min, linear_max, angular_max
        self.dodge_linear, self.dodge_angular = dodge_linear, dodge_angular
        self.state, self.until, self.cooldown = 'ROAM', 0.0, 0.0
        self.v, self.w, self.sign = 0.25, 0.0, 1
        self.pillar = self.obstacles[0]
        self.pose, self.joints = None, {}
        self.future_joints = None
        self.now = 0.0
        self.history = deque()
        self.expected = self.command_v = 0.0
        self.recovering = False
        self.recovery_count = self.wrap_count = self.wrap_completed = self.wrap_aborted = 0
        self.blocked_since = None
        self.actual_distance = 0.0
        self.nearest_distance = self.minimum_clearance = math.inf
        self.base_tree_distance = math.inf
        self.wrap_radius = 1.6
        self.stage = ''

    def points(self, pose=None, joints=None):
        x, y, a = self.pose if pose is None else pose
        c, s = math.cos(a), math.sin(a)
        return [(x+c*px-s*py, y+s*px+c*py, r)
                for px, py, r in local_centers(self.joints if joints is None else joints)]

    def clearance(self, pose=None, joints=None):
        return min(math.hypot(x-ox, y-oy)-r-ore for x, y, r in self.points(pose, joints)
                   for ox, oy, ore in self.obstacles)

    def predicted(self, v, w):
        x, y, a = self.pose
        result = self.clearance()
        for t in (0.1, 0.25, 0.5, 0.8):
            pose = ((x+v*t*math.cos(a), y+v*t*math.sin(a), a) if abs(w)<1e-6 else
                    (x+v/w*(math.sin(a+w*t)-math.sin(a)), y-v/w*(math.cos(a+w*t)-math.cos(a)), a+w*t))
            joints = self.future_joints(self.now+t) if self.future_joints else None
            result = min(result, self.clearance(pose, joints))
            if self.state.startswith('WRAP'):
                # User's center-distance floor; full-body clearance is stricter.
                result = min(result, math.hypot(pose[0]-self.pillar[0], pose[1]-self.pillar[1])-0.5)
        return result

    def recover(self, now):
        if self.state.startswith('WRAP'):
            self.wrap_aborted += 1
        self.state, self.recovering, self.until = 'DODGE', True, now+2.0
        self.recovery_count += 1
        self.sign = max((-1, 1), key=lambda s: self.predicted(self.dodge_linear, s*self.dodge_angular))
        self.history.clear()
        self.blocked_since = None

    def _track_motion(self, now, dt):
        x, y, _ = self.pose
        self.expected += abs(self.command_v)*dt
        self.history.append((now, x, y, self.expected))
        while len(self.history)>1 and self.history[1][0] <= now-3.0:
            self.history.popleft()
        if len(self.history)>1 and self.history[-1][0]-self.history[0][0]>=2.99:
            self.actual_distance = sum(math.hypot(b[1]-a[1], b[2]-a[2])
                                       for a, b in zip(self.history, list(self.history)[1:]))
            if (not self.recovering and self.expected-self.history[0][3]>0.15
                    and self.actual_distance<0.1):
                self.recover(now)

    def _wrap_command(self, now):
        x, y, a = self.pose
        dx, dy = x-self.pillar[0], y-self.pillar[1]
        r, angle = math.hypot(dx, dy), math.atan2(dy, dx)
        self.base_tree_distance = r
        if self.state=='WRAP_SETUP':
            if now>=self.until or r<0.5:
                self.recover(now)
                return self.dodge_linear, self.sign*self.dodge_angular
            if self.stage=='BACKOFF':
                if r>=self.wrap_radius+0.03:
                    self.stage='ALIGN'
                else:
                    error=wrap(angle+math.pi-a)
                    return self.dodge_linear*max(0.0, math.cos(error)), clamp(2*error, -0.6, 0.6)
            error=wrap(angle+self.sign*math.pi/2-a)
            if abs(error)<0.12:
                self.state, self.until='WRAP', now+4.0
                self.wrap_count += 1
            else:
                return 0.0, clamp(2*error, -0.6, 0.6)
        if self.state=='WRAP':
            if now>=self.until:
                self.state, self.until='WRAP_EXIT', now+7.0
                self.wrap_completed += 1
            else:
                radial=2.0*(self.wrap_radius-r)
                heading=math.atan2(self.sign*dx/r+radial*dy/r, -self.sign*dy/r+radial*dx/r)
                error=wrap(heading-a)
                v=self.linear_max*max(0.0, math.cos(error))
                return v, clamp(self.sign*v/max(r, 0.5)+2*error, -0.8, 0.8)
        if self.state=='WRAP_EXIT':
            if now>=self.until or r>self.wrap_radius+0.45:
                self.state, self.until, self.cooldown='ROAM', now+2.0, now+8.0
            else:
                error=wrap(angle-a)
                return 0.2*max(0.0, math.cos(error)), clamp(2*error, -1.0, 1.0)
        return self.v, self.w

    def step(self, now, dt, speed_factor):
        self.now = now
        x, y, a = self.pose
        self._track_motion(now, dt)
        hx, hy, _ = self.points()[1]
        nearest=min(self.obstacles, key=lambda o: math.hypot(hx-o[0], hy-o[1])-o[2])
        # Near-tree thresholds are surface gaps, not center distances.
        self.nearest_distance=math.hypot(hx-nearest[0], hy-nearest[1])-nearest[2]
        self.minimum_clearance=self.clearance()
        if not self.state.startswith('WRAP'):
            self.base_tree_distance=math.hypot(x-nearest[0], y-nearest[1])
        if self.recovering:
            if now<self.until:
                return self.dodge_linear, self.sign*self.dodge_angular
            self.recovering=False
            self.state, self.until, self.cooldown='ROAM', now+2.0, now+1.0
            self.history.clear()
        if speed_factor<0.01 and not self.state.startswith('WRAP'):
            return 0.0, 0.0
        if self.state.startswith('WRAP'):
            v, w=self._wrap_command(now)
        else:
            if self.nearest_distance<0.3 and self.state!='DODGE':
                self.state, self.until, self.sign='DODGE', now+1.5, self.rng.choice((-1, 1))
            elif self.state in ('CIRCLE', 'DODGE') and now>=self.until:
                self.state, self.until, self.cooldown='ROAM', now+2.0, now+1.0
            elif self.state=='ROAM':
                if self.nearest_distance<1.3 and now>=self.cooldown:
                    self.state, self.until, self.pillar='AVOID', now+8.0, nearest
                elif now>=self.until:
                    self.v=self.rng.uniform(self.linear_min, self.linear_max)
                    self.w=self.rng.uniform(-self.angular_max, self.angular_max)
                    self.until=now+self.rng.uniform(2.0, 4.0)
            if self.state=='AVOID':
                if self.nearest_distance<0.6:
                    if self.rng.random()<0.5 and self.base_tree_distance>=0.5:
                        self.pillar=nearest
                        self.wrap_radius=max(0.5, nearest[2]+1.15+0.18)
                        self.sign=self.rng.choice((-1, 1))
                        self.state, self.stage, self.until='WRAP_SETUP', 'BACKOFF', now+16.0
                        v, w=self._wrap_command(now)
                    else:
                        self.state, self.until, self.sign='DODGE', now+1.5, self.rng.choice((-1, 1))
                        v, w=self.dodge_linear, self.sign*self.dodge_angular
                elif now>=self.until:
                    self.state, self.until, self.sign='CIRCLE', now+4.0, self.rng.choice((-1, 1))
                    v, w=0.0, self.sign*0.6
                else:
                    error=wrap(math.atan2(nearest[1]-y, nearest[0]-x)-a)
                    v, w=0.18*max(0.0, math.cos(error)), clamp(2*error, -0.6, 0.6)
            elif self.state=='DODGE':
                v, w=self.dodge_linear, self.sign*self.dodge_angular
            elif self.state=='CIRCLE':
                dx, dy=x-self.pillar[0], y-self.pillar[1]
                r=max(math.hypot(dx, dy), 1e-6)
                radial=1.8*(1.5-r)
                heading=math.atan2(self.sign*dx/r+radial*dy/r, -self.sign*dy/r+radial*dx/r)
                error=wrap(heading-a)
                v=0.25*max(0.0, math.cos(error))
                w=clamp(self.sign*v/1.5+error, -0.6, 0.6)
            elif self.state!='AVOID':
                v, w=self.v, self.w
                if abs(x)>13 or abs(y)>13:
                    error=wrap(math.atan2(-y, -x)-a)
                    v, w=0.2*max(0.0, math.cos(error)), clamp(error, -0.6, 0.6)
            if not self.state.startswith('WRAP'):
                v, w=v*speed_factor, w*speed_factor
        # Anticipate moving joints as well as the moving base.
        blocked=False
        if self.predicted(v, w)<0.1:
            if self.state=='WRAP':
                self.wrap_aborted += 1
                self.state, self.until='WRAP_EXIT', now+7.0
            v=0.0
            w=max((-0.6, 0.6), key=lambda rate:self.predicted(0.0, rate))
            if self.predicted(0.0, w)<0.04:
                w, blocked=0.0, True
        if blocked:
            if self.blocked_since is None:
                self.blocked_since=now
            elif now-self.blocked_since>=3:
                self.recover(now)
                return self.dodge_linear, self.sign*self.dodge_angular
        else:
            self.blocked_since=None
        return v, w
