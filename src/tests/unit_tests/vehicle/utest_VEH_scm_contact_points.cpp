// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
// Copyright (c) 2026 projectchrono.org
// Use of this source code is governed by a BSD-style license in the LICENSE file.
// =============================================================================

#include <algorithm>

#include "gtest/gtest.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

using namespace chrono;
using namespace chrono::vehicle;

TEST(SCMContactPoints, WorldForceMomentTimingAndClear) {
    for (double angle : {0.0, 0.3}) {
        ChSystemSMC sys;
        sys.SetGravitationalAcceleration(VNULL);
        sys.SetNumThreads(1, 1, 1);
        sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
        auto rotation = QuatFromAngleY(-angle);
        ChVector3d origin(1.2, -0.8, 0.4);
        auto material = chrono_types::make_shared<ChContactMaterialSMC>();
        auto body = chrono_types::make_shared<ChBodyEasyBox>(0.4, 0.4, 0.2, 1000, false, true, material);
        body->SetPos(origin + rotation.Rotate(ChVector3d(0, 0, 0.09)));
        body->SetRot(rotation);
        body->SetPosDt(rotation.Rotate(ChVector3d(0, 0, -0.1)));
        // Spin about the terrain normal: opposite sides of the footprint slide in opposite directions.
        body->SetAngVelParent(rotation.Rotate(ChVector3d(0, 0, 2)));
        sys.AddBody(body);
        SCMTerrain terrain(&sys, false);
        terrain.SetReferenceFrame(ChCoordsys<>(origin, rotation));
        terrain.SetSoilParameters(820000, 1400, 1, 170, 35, 0.0178, 2e8, 30000);
        terrain.Initialize(2, 2, 0.02);
        EXPECT_EQ(terrain.GetContactForcesBody(body), nullptr);
        const auto initial_com = body->GetPos();
        sys.DoStepDynamics(0.001);
        auto snapshot = terrain.GetContactForcesBody(body);
        ASSERT_NE(snapshot, nullptr);
        ASSERT_FALSE(snapshot->contacts.empty());
        EXPECT_NEAR(snapshot->time, 0, 1e-14);
        EXPECT_LT((snapshot->reference_point - initial_com).Length(), 1e-12);
        // Query after integration: moment reconstruction must use the saved COM, not the updated body position.
        EXPECT_GT((body->GetPos() - initial_com).Length(), 1e-8);
        ChVector3d force = VNULL;
        ChVector3d moment = VNULL;
        ChVector3d direction = rotation.Rotate(VECT_X);
        double prop = 0;
        double oppose = 0;
        for (const auto& contact : snapshot->contacts) {
            force += contact.force;
            moment += Vcross(contact.point - snapshot->reference_point, contact.force);
            double projection = contact.force.Dot(direction);
            prop += std::max(projection, 0.0);
            oppose += std::max(-projection, 0.0);
        }
        ChVector3d existing_force, existing_moment;
        ASSERT_TRUE(terrain.GetContactForceBody(body, existing_force, existing_moment));
        EXPECT_LT((force - existing_force).Length(), 1e-9);
        EXPECT_LT((moment - existing_moment).Length(), 1e-9);
        EXPECT_NEAR(prop - oppose, existing_force.Dot(direction), 1e-9);
        EXPECT_GT(prop, 1e-4);
        EXPECT_GT(oppose, 1e-4);
        // Updating the system must not recompute the history-dependent SCM forces.
        auto count = snapshot->contacts.size();
        sys.Update();
        ASSERT_NE(terrain.GetContactForcesBody(body), nullptr);
        EXPECT_EQ(terrain.GetContactForcesBody(body)->contacts.size(), count);
        EXPECT_NEAR(terrain.GetContactForcesBody(body)->time, 0, 1e-14);
        // On the next force evaluation, absent contacts must clear the complete snapshot.
        body->SetPos(origin + rotation.Rotate(ChVector3d(0, 0, 2)));
        body->SetPosDt(VNULL);
        body->SetAngVelParent(VNULL);
        sys.DoStepDynamics(0.001);
        EXPECT_EQ(terrain.GetContactForcesBody(body), nullptr);
        EXPECT_FALSE(terrain.GetContactForceBody(body, existing_force, existing_moment));
        EXPECT_EQ(existing_force.Length(), 0);
        EXPECT_EQ(existing_moment.Length(), 0);
    }
}
