"""NAO6 H25600 low-level control study: the RKneePitch extension phase.

The whole-robot reference is ../urdf/nao6_simplified.urdf (21 independent
moving coordinates). The RViz match uses its collision-visual derivative.
The right knee rotates around local +Z from RHipPitch_link to RKneePitch_link.
This scalar model uses the URDF angle bounds and datasheet motor/gear specs.
Electrical, load and feedback parameters are study assumptions. The model
simulates a joint phase; whole-body balance and foot-ball contact are separate.

Run: python3 knee_model.py [output_directory]  (Python 3.8+, standard library).
Exports nominal.csv, disturbed.csv and summary.json. Each trial has 401
samples including the initial state. The high-level planner assigns regions;
joint references are inputs supplied by the local motion layer.
"""
import csv
import json
import math
import argparse
from pathlib import Path

# Type 5 motor and KneePitch gearbox, H25600 datasheet p. 6.
GEAR = 130.85
RPM = 8700.0
STALL_NM = 0.065
CONT_NM = 0.0178
ENCODER_STEP = math.radians(0.1)
# Offline assumptions. V is a motor test reference, not the battery voltage.
V = 12.0
ETA = 0.72
KE = V / (RPM * 2 * math.pi / 60)
KT = KE
R = V * KT / STALL_NM
L = R * 0.00055
DRIVE = ETA * GEAR * KT
J = 0.015  # lumped inertia on the output side, kg m^2
B = 0.035  # viscous damping, N m s/rad
G = 0.12   # equivalent gravity coefficient, N m
KP = 20.0  # V/rad
KD = 0.18  # V s/rad
CONTROL_PERIOD = 0.005
DT = 0.0001
DURATION = 2.0
START, END = 0.25, 0.85
Q0, Q1 = 0.62, 0.28
LOAD_START, LOAD_END, LOAD_NM = 0.48, 0.80, 0.36
TOL, DWELL, DEADLINE = 0.015, 0.12, 1.50
QMIN, QMAX = -0.0923279, 2.11255


def reference(t):
    fraction = min(1.0, max(0.0, (t - START) / (END - START)))
    return Q0 + (Q1 - Q0) * fraction**2 * (3 - 2 * fraction)


def simulate(load, dt=DT):
    period = round(CONTROL_PERIOD / dt)
    q, speed = Q0, 0.0
    current = G * math.sin(q) / DRIVE  # equilibrium at the initial pose
    rows = []
    for k in range(round(DURATION / dt) + 1):
        t = round(k * dt, 9)
        torque = load if LOAD_START <= t < LOAD_END else 0.0
        if k % period == 0:
            ref = reference(t)
            sensed = round(q / ENCODER_STEP) * ENCODER_STEP
            # Quantised position; speed is ideal in this offline model.
            voltage = KP * (ref - sensed) - KD * speed + R * G * math.sin(sensed) / DRIVE
            voltage = max(-V, min(V, voltage))
            rows.append(dict(t_s=t, ref_rad=ref, sensed_rad=sensed, q_rad=q,
                             speed_rad_s=speed, error_rad=ref-sensed, volts=voltage,
                             amps=current, load_Nm=torque))
        if k == round(DURATION / dt):
            break
        # Exact electrical update for a held voltage and frozen back EMF.
        steady = (voltage - KE * GEAR * speed) / R
        decay = math.exp(-dt * R / L)
        avg = steady + (current - steady) * (1 - decay) * L / (dt * R)
        current = steady + (current - steady) * decay
        # Extension decreases q, so a positive external torque resists extension.
        accel = (DRIVE * avg - B * speed - G * math.sin(q) + torque) / J
        speed += dt * accel
        q += dt * speed
        if not QMIN <= q <= QMAX:
            raise ValueError("Trajectory left the URDF knee range")
    return rows


def metrics(rows, deadline=DEADLINE):
    since = None
    completed = None
    for row in rows:
        t = row['t_s']
        if t > deadline:
            break
        if t < END or abs(row['error_rad']) > TOL:
            since = None
        else:
            if since is None:
                since = t
            if t - since >= DWELL - 1e-9:
                completed = t
                break
    return dict(peak_error_rad=max(abs(r['error_rad']) for r in rows),
                rms_error_rad=math.sqrt(sum(r['error_rad']**2 for r in rows)/len(rows)),
                completed_at_s=completed, status='complete' if completed is not None else 'timeout')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("output_directory", nargs="?", type=Path,
                        default=Path("knee_results"))
    out = parser.parse_args().output_directory
    out.mkdir(parents=True, exist_ok=True)
    results = {}
    finer_check = {}
    traces = {}
    for name, load in [('nominal', 0.0), ('disturbed', LOAD_NM)]:
        rows = simulate(load)
        traces[name] = rows
        results[name] = metrics(rows)
        fine = simulate(load, DT / 2)
        max_diff = max(abs(a['q_rad'] - b['q_rad']) for a, b in zip(rows, fine))
        assert max_diff < ENCODER_STEP
        assert metrics(fine)['status'] == results[name]['status']
        finer_check[name] = max_diff
        with (out / (name + '.csv')).open('w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    for a, b in zip(traces['nominal'], traces['disturbed']):
        assert a['ref_rad'] == b['ref_rad']
        if a['t_s'] < LOAD_START:
            assert a == b
    assert metrics(traces['nominal'], END + DWELL / 2)['status'] == 'timeout'
    assert results['disturbed']['peak_error_rad'] > results['nominal']['peak_error_rad']
    (out / 'summary.json').write_text(json.dumps(dict(cases=results,
        half_step_max_angle_change_rad=finer_check),indent=2),encoding='utf-8')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
