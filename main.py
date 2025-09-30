import opensim as osim
import math

def build_model():
    model = osim.Model()
    model.setName("Day1_AnkleTest")
    ground = model.getGround()

    # Bodies
    tibia = osim.Body("tibia", 3.0, osim.Vec3(0), osim.Inertia(0.02,0.02,0.02))
    talus = osim.Body("talus", 0.2, osim.Vec3(0), osim.Inertia(0.001,0.001,0.001))
    calc = osim.Body("calcaneus", 0.3, osim.Vec3(0), osim.Inertia(0.002,0.002,0.002))
    model.addBody(tibia)
    model.addBody(talus)
    model.addBody(calc)

    # ✅ Weld tibia to ground (fixes tibia in place, no warning anymore)
    tibia_joint = osim.WeldJoint("tibia_ground",
                                 ground, osim.Vec3(0), osim.Vec3(0),
                                 tibia,  osim.Vec3(0), osim.Vec3(0))
    model.addJoint(tibia_joint)

    # Tibio-talar joint: tibia ↔ talus
    TT = osim.PinJoint("TT", tibia, osim.Vec3(0), osim.Vec3(0),
                       talus, osim.Vec3(0), osim.Vec3(0))
    TT.upd_coordinates(0).setName("tt_pfdf")   # plantarflexion / dorsiflexion
    model.addJoint(TT)

    # Subtalar joint: talus ↔ calcaneus
    ST = osim.PinJoint("ST", talus, osim.Vec3(0), osim.Vec3(0),
                       calc, osim.Vec3(0), osim.Vec3(0))
    ST.upd_coordinates(0).setName("st_inv_ev") # inversion / eversion
    model.addJoint(ST)

    # ROM / ligament limits (placeholder values)
    model.addForce(osim.CoordinateLimitForce("tt_pfdf",
                                             30*math.pi/180, 1e3,
                                             -20*math.pi/180, 1e3,
                                             5, 0.05))
    model.addForce(osim.CoordinateLimitForce("st_inv_ev",
                                             30*math.pi/180, 1e3,
                                             -20*math.pi/180, 1e3,
                                             5, 0.05))

    # Simple vertical GRF impulse at calcaneus
    BW = 700.0  # Newtons (example body weight)
    force = osim.PrescribedForce("forefoot_force", calc)  # attach to calcaneus
    force.set_pointIsGlobal(True)
    force.set_forceIsGlobal(True)

    # Force profile
    times = [0.0, 0.06, 0.20]         # 0 ms, 60 ms peak, 200 ms decay
    Fy = [0.0, 2.0*BW, 0.5*BW]        # vertical load profile

    fn_y = osim.PiecewiseLinearFunction()
    for t, v in zip(times, Fy):
        fn_y.addPoint(t, v)

    fn_x = osim.PiecewiseLinearFunction()
    fn_x.addPoint(0, 0); fn_x.addPoint(0.2, 0)

    fn_z = osim.PiecewiseLinearFunction()
    fn_z.addPoint(0, 0); fn_z.addPoint(0.2, 0)

    # Package functions into FunctionSet
    funcs = osim.FunctionSet()
    funcs.cloneAndAppend(fn_x)
    funcs.cloneAndAppend(fn_y)
    funcs.cloneAndAppend(fn_z)

    force.set_forceFunctions(funcs)
    model.addForce(force)

    model.finalizeConnections()
    return model

def run_once():
    model = build_model()
    state = model.initSystem()

    # Initial slight plantarflexion & eversion
    model.updCoordinateSet().get("tt_pfdf").setValue(state, -5*math.pi/180)
    model.updCoordinateSet().get("st_inv_ev").setValue(state, -2*math.pi/180)

    manager = osim.Manager(model)
    state.setTime(0.0)
    manager.initialize(state)
    final_state = manager.integrate(0.20)

    tt = model.getCoordinateSet().get("tt_pfdf").getValue(final_state)
    st = model.getCoordinateSet().get("st_inv_ev").getValue(final_state)

    print("Final joint angles (deg):")
    print("  Tibio-talar:", round(tt*180/math.pi, 2))
    print("  Subtalar:   ", round(st*180/math.pi, 2))

if __name__ == "__main__":
    run_once()
