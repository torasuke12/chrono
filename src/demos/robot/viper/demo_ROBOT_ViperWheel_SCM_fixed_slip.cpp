// =============================================================================
// Viper single-wheel fixed-slip test on SCM terrain.
// The prescribed slip definition matches the CRM sweep:
//     s = 1 - vx / (r * omega)
// =============================================================================

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "chrono/utils/ChConstants.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"

#include "demos/SetChronoSolver.h"
#include "viper_wheel.h"

using namespace chrono;
using namespace chrono::vehicle;

struct Parameters {
    double peripheral_speed = 1.0;
    double slip = 0.5;
    double slope = 0.0;
    double gravity = 1.635;
    double total_mass = 100.0;
    double sim_time = 10.0;
    double step_size = 1e-3;
    double output_fps = 100.0;
    int num_threads = 16;
    double terrain_length = 30.0;
    double terrain_width = 0.8;
    double grid_spacing = 0.005;
    double grouser_height = 0.01;
    double grouser_width = 0.005;
    double Bekker_Kphi = 0.82e6;
    double Bekker_Kc = 0.14e4;
    double Bekker_n = 1.0;
    double Mohr_cohesion = 0.017e4;
    double Mohr_friction = 35.0;
    double Janosi_shear = 1.78e-2;
    std::string output_dir = "SCM_VIPER_FIXED_SLIP";
};

bool ParseCommandLine(int argc, char** argv, Parameters& p) {
    ChCLI cli(argv[0], "Viper single-wheel fixed-slip SCM test");
    cli.AddOption<double>("Test", "peripheral-speed", "Prescribed wheel peripheral speed [m/s]",
                          std::to_string(p.peripheral_speed));
    cli.AddOption<double>("Test", "slip", "Slip s=1-vx/(r*omega)", std::to_string(p.slip));
    cli.AddOption<double>("Test", "slope", "Slope angle [deg]", std::to_string(p.slope));
    cli.AddOption<double>("Test", "gravity", "Gravity magnitude [m/s^2]", std::to_string(p.gravity));
    cli.AddOption<double>("Test", "total-mass", "Equivalent supported mass [kg]", std::to_string(p.total_mass));
    cli.AddOption<double>("Test", "sim-time", "Simulation duration [s]", std::to_string(p.sim_time));
    cli.AddOption<double>("Numerics", "step-size", "Dynamics time step [s]", std::to_string(p.step_size));
    cli.AddOption<double>("Numerics", "output-fps", "CSV output frequency [Hz]", std::to_string(p.output_fps));
    cli.AddOption<int>("Numerics", "threads", "OpenMP threads used by Chrono/SCM", std::to_string(p.num_threads));
    cli.AddOption<double>("Terrain", "terrain-length", "SCM patch length [m]", std::to_string(p.terrain_length));
    cli.AddOption<double>("Terrain", "terrain-width", "SCM patch width [m]", std::to_string(p.terrain_width));
    cli.AddOption<double>("Terrain", "grid-spacing", "SCM grid spacing [m]", std::to_string(p.grid_spacing));
    cli.AddOption<double>("Wheel", "grouser-height", "Viper grouser height [m]", std::to_string(p.grouser_height));
    cli.AddOption<double>("Wheel", "grouser-width", "Viper grouser width [m]", std::to_string(p.grouser_width));
    cli.AddOption<std::string>("Output", "output-dir", "Output directory", p.output_dir);
    if (!cli.Parse(argc, argv))
        return false;

    p.peripheral_speed = cli.GetAsType<double>("peripheral-speed");
    p.slip = cli.GetAsType<double>("slip");
    p.slope = cli.GetAsType<double>("slope");
    p.gravity = cli.GetAsType<double>("gravity");
    p.total_mass = cli.GetAsType<double>("total-mass");
    p.sim_time = cli.GetAsType<double>("sim-time");
    p.step_size = cli.GetAsType<double>("step-size");
    p.output_fps = cli.GetAsType<double>("output-fps");
    p.num_threads = cli.GetAsType<int>("threads");
    p.terrain_length = cli.GetAsType<double>("terrain-length");
    p.terrain_width = cli.GetAsType<double>("terrain-width");
    p.grid_spacing = cli.GetAsType<double>("grid-spacing");
    p.grouser_height = cli.GetAsType<double>("grouser-height");
    p.grouser_width = cli.GetAsType<double>("grouser-width");
    p.output_dir = cli.GetAsType<std::string>("output-dir");
    return true;
}

int main(int argc, char** argv) {
    Parameters p;
    if (!ParseCommandLine(argc, argv, p))
        return 1;
    if (!(p.peripheral_speed > 0 && p.slip >= 0 && p.slip < 1 && p.gravity > 0 && p.total_mass > 0 &&
          p.sim_time > 0 && p.step_size > 0 && p.output_fps > 0 && p.terrain_length > 0 &&
          p.terrain_width > 0 && p.grid_spacing > 0 && p.num_threads > 0)) {
        std::cerr << "Invalid test parameter. Require speed>0, 0<=slip<1, and positive dimensions/times.\n";
        return 2;
    }

    auto wheel = chrono_types::make_shared<DummyViperWheel>();
    auto tire = chrono_types::make_shared<ViperTire>();
    tire->SetGrouserHeight(p.grouser_height);
    tire->SetGrouserWidth(p.grouser_width);

    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    // SCMTerrain uses GetNumThreadsChrono() in its OpenMP ray-casting loops.
    // Bullet collision and Eigen remain single-threaded for this small rig.
    sys.SetNumThreads(p.num_threads, 1, 1);
    SetChronoSolver(sys, ChSolver::Type::BARZILAIBORWEIN,
                    ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    ChTireTestRig rig(wheel, tire, &sys);
    rig.SetGravitationalAcceleration(p.gravity);
    rig.SetNormalLoad(p.total_mass * p.gravity);
    rig.SetTireStepsize(p.step_size);
    rig.SetTireVisualizationType(VisualizationType::COLLISION);

    ChTireTestRig::TerrainPatchSize size;
    size.length = p.terrain_length;
    size.width = p.terrain_width;
    ChTireTestRig::TerrainParamsSCM soil;
    soil.Bekker_Kphi = p.Bekker_Kphi;
    soil.Bekker_Kc = p.Bekker_Kc;
    soil.Bekker_n = p.Bekker_n;
    soil.Mohr_cohesion = p.Mohr_cohesion;
    soil.Mohr_friction = p.Mohr_friction;
    soil.Janosi_shear = p.Janosi_shear;
    soil.grid_spacing = p.grid_spacing;
    rig.SetTerrainSCM(size, soil);

    const double radius = 0.225 + p.grouser_height;
    const double longitudinal_speed = p.peripheral_speed * (1.0 - p.slip);
    const double angular_speed = p.peripheral_speed / radius;
    rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(longitudinal_speed));
    rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(angular_speed));
    rig.SetTimeDelay(0.0);
    rig.Initialize(ChTireTestRig::Mode::TEST, 0.05);

    // Keep the SCM plane horizontal and represent the slope through gravity,
    // matching the CRM fixed-slip test convention. Positive slope opposes +x travel.
    const double theta = p.slope * CH_DEG_TO_RAD;
    sys.SetGravitationalAcceleration(
        ChVector3d(-p.gravity * std::sin(theta), 0, -p.gravity * std::cos(theta)));

    std::filesystem::create_directories(p.output_dir);
    std::ofstream csv(std::filesystem::path(p.output_dir) / "results.csv");
    if (!csv) {
        std::cerr << "Cannot create output CSV in " << p.output_dir << "\n";
        return 1;
    }
    csv << std::setprecision(12);
    csv << "time,x,y,z,vx,vy,vz,omega_y,peripheral_speed,slip_crm,slip_tire_rig,"
           "contact_fx,contact_fy,contact_fz,contact_tx,contact_ty,contact_tz,drawbar_pull,motor_torque\n";

    std::cout << "SCM Viper fixed-slip test\n"
              << "  r*omega: " << p.peripheral_speed << " m/s\n"
              << "  vx:      " << longitudinal_speed << " m/s\n"
              << "  slip:    " << p.slip << "\n"
              << "  slope:   " << p.slope << " deg\n"
              << "  load:    " << p.total_mass * p.gravity << " N at zero slope\n"
              << "  threads: " << p.num_threads << "\n"
              << "  output:  " << p.output_dir << "/results.csv\n";

    int output_frame = 0;
    while (sys.GetChTime() <= p.sim_time) {
        const double time = sys.GetChTime();
        if (time + 1e-12 >= output_frame / p.output_fps) {
            auto spindle = rig.GetSpindle();
            auto tf = rig.ReportTireForce();
            const auto pos = spindle->GetPos();
            const auto vel = spindle->GetPosDt();
            const double omega = spindle->GetAngVelLocal().y();
            const double vp_actual = radius * omega;
            const double s_actual = std::abs(vp_actual) > 1e-9 ? 1.0 - vel.x() / vp_actual : 0.0;
            csv << time << ',' << pos.x() << ',' << pos.y() << ',' << pos.z() << ',' << vel.x() << ',' << vel.y()
                << ',' << vel.z() << ',' << omega << ',' << vp_actual << ',' << s_actual << ','
                << rig.GetLongitudinalSlip() << ',' << tf.force.x() << ',' << tf.force.y() << ',' << tf.force.z()
                << ',' << tf.moment.x() << ',' << tf.moment.y() << ',' << tf.moment.z() << ',' << rig.GetDBP()
                << ',' << rig.GetMotorWheel()->GetMotorTorque() << '\n';
            output_frame++;
        }
        rig.Advance(p.step_size);
    }
    return 0;
}
