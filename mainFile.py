import opensim as osim
import math
import os, csv
import numpy as np

# === Simulation parameters (defaults) ===
BW_N = 700.0       # body weight (N)
T_PEAK_S = 0.060   # time to reach peak force (s)
RESID_FRAC = 0.50  # residual force fraction of BW
SIM_T = 0.20       # total simulation time (s)

# 3-D impact direction (radians)
ANGLE_X = 0.0
ANGLE_Y = 0.0
ANGLE_Z = 0.0

RESULTS_PATH = os.path.join(os.path.dirname(__file__), "day2_runs.csv")

# ----------------------------------------------------------------- building model function
def build_model():
    model = osim.Model()
    model.setName("Day2_Ankle_3D")
    ground = model.getGround()

    # --- Bodies ---
    tibia = osim.Body("tibia", 3.0, osim.Vec3(0), osim.Inertia(0.02,0.02,0.02))
    talus = osim.Body("talus", 0.2, osim.Vec3(0), osim.Inertia(0.001,0.001,0.001))
    calc  = osim.Body("calcaneus", 0.3, osim.Vec3(0), osim.Inertia(0.002,0.002,0.002))
    fore  = osim.Body("forefoot", 0.2, osim.Vec3(0), osim.Inertia(0.001,0.001,0.001))
    for b in (tibia, talus, calc, fore):
        model.addBody(b)

    # --- Joints ---
    model.addJoint(osim.WeldJoint("tibia_ground",
        ground, osim.Vec3(0), osim.Vec3(0), tibia, osim.Vec3(0), osim.Vec3(0)))

    TT = osim.PinJoint("TT", tibia, osim.Vec3(0), osim.Vec3(0),
                       talus, osim.Vec3(0), osim.Vec3(0))
    TT.upd_coordinates(0).setName("tt_pfdf")
    model.addJoint(TT)

    ST = osim.PinJoint("ST", talus, osim.Vec3(0), osim.Vec3(0),
                       calc, osim.Vec3(0), osim.Vec3(0))
    ST.upd_coordinates(0).setName("st_inv_ev")
    model.addJoint(ST)

    MT = osim.PinJoint("MT", calc, osim.Vec3(0), osim.Vec3(0),
                       fore, osim.Vec3(0), osim.Vec3(0))
    MT.upd_coordinates(0).setName("mt_flex_ext")
    model.addJoint(MT)

    # --- Ligament-like range limits ---
    model.addForce(osim.CoordinateLimitForce(
        "tt_pfdf", 30*math.pi/180, 1e3, -20*math.pi/180, 1e3, 5, 0.05))
    model.addForce(osim.CoordinateLimitForce(
        "st_inv_ev", 30*math.pi/180, 1e3, -20*math.pi/180, 1e3, 5, 0.05))
    model.addForce(osim.CoordinateLimitForce(
        "mt_flex_ext", 15*math.pi/180, 500, -15*math.pi/180, 500, 3, 0.05))

    # --- Muscle-equivalent spring with built-in viscous damping ---
    pf_spring = osim.SpringGeneralizedForce()
    pf_spring.set_coordinate("tt_pfdf")
    pf_spring.set_stiffness(800.0)      # Nm/rad
    pf_spring.set_rest_length(0.0)      # rad (rest angle offset)
    pf_spring.set_viscosity(10.0)       # Nms/rad  << acts as a linear damper
    model.addForce(pf_spring)

    # --- Prescribed 3-D ground-reaction impulse on forefoot ---
    force = osim.PrescribedForce("forefoot_force", fore)
    force.set_pointIsGlobal(True)
    force.set_forceIsGlobal(True)

    times = [0.0, T_PEAK_S, SIM_T]
    Fy = [0.0, 2.0*BW_N, RESID_FRAC*BW_N]
    fn_y = osim.PiecewiseLinearFunction()
    for t, v in zip(times, Fy):
        fn_y.addPoint(t, v)

    # Direction vector (normalize)
    dir_x, dir_y, dir_z = math.sin(ANGLE_X), math.cos(ANGLE_Y), math.sin(ANGLE_Z)
    norm = math.sqrt(dir_x**2 + dir_y**2 + dir_z**2)
    dir_x, dir_y, dir_z = dir_x/norm, dir_y/norm, dir_z/norm

    fn_x = osim.PiecewiseLinearFunction()
    fn_z = osim.PiecewiseLinearFunction()
    for t, v in zip(times, Fy):
        fn_x.addPoint(t, v*dir_x)
        fn_z.addPoint(t, v*dir_z)

    funcs = osim.FunctionSet()
    funcs.cloneAndAppend(fn_x)
    funcs.cloneAndAppend(fn_y)
    funcs.cloneAndAppend(fn_z)
    force.set_forceFunctions(funcs)
    model.addForce(force)

    model.finalizeConnections()
    return model

# ----------------------------------------------------------------- run once function
def run_once():
    model = build_model()
    state = model.initSystem()

    cs = model.updCoordinateSet()
    cs.get("tt_pfdf").setValue(state, -5*math.pi/180)
    cs.get("st_inv_ev").setValue(state, -2*math.pi/180)
    cs.get("mt_flex_ext").setValue(state, 0)

    manager = osim.Manager(model)
    state.setTime(0.0)
    manager.setIntegratorAccuracy(1e-3)
    manager.initialize(state)
    final_state = manager.integrate(SIM_T)

    tt = cs.get("tt_pfdf").getValue(final_state)
    st = cs.get("st_inv_ev").getValue(final_state)
    mt = cs.get("mt_flex_ext").getValue(final_state)

    failed = (
        (tt > 30*math.pi/180) or (tt < -20*math.pi/180) or
        (abs(st) > 30*math.pi/180) or (abs(mt) > 15*math.pi/180)
    )

    row = {
        "t_peak_ms": int(T_PEAK_S*1000),
        "residual_frac": RESID_FRAC,
        "angle_x_deg": int(math.degrees(ANGLE_X)),
        "angle_y_deg": int(math.degrees(ANGLE_Y)),
        "angle_z_deg": int(math.degrees(ANGLE_Z)),
        "tt_deg_final": tt*180/math.pi,
        "st_deg_final": st*180/math.pi,
        "mt_deg_final": mt*180/math.pi,
        "failed": int(failed)
    }

    print(row)
    os.makedirs(os.path.dirname(RESULTS_PATH), exist_ok=True)
    write_header = not os.path.exists(RESULTS_PATH)
    with open(RESULTS_PATH, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=row.keys())
        if write_header: w.writeheader()
        w.writerow(row)

# ----------------------------------------------------------------- multiple run_once() iterations
def run_batch():
    # start with a tiny grid to validate runtime, then expand
    t_peaks = [0.04, 0.06]
    residuals = [0.25, 0.50]
    angles_deg = [0, 25]

    total = len(t_peaks)*len(residuals)*len(angles_deg)**3
    count = 0
    for tp in t_peaks:
        for rf in residuals:
            for ax in angles_deg:
                for ay in angles_deg:
                    for az in angles_deg:
                        count += 1
                        global T_PEAK_S, RESID_FRAC, ANGLE_X, ANGLE_Y, ANGLE_Z
                        T_PEAK_S, RESID_FRAC = tp, rf
                        ANGLE_X, ANGLE_Y, ANGLE_Z = map(math.radians, [ax, ay, az])
                        print(f"[{count}/{total}] tp={tp:.3f}s rf={rf:.2f} angles=({ax},{ay},{az})")
                        run_once()


if __name__ == "__main__":
    run_batch()
