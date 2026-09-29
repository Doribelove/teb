#include <gtest/gtest.h>

#include <teb_local_planner/g2o_types/edge_kinematics.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/timed_elastic_band.h>
#include <teb_local_planner/recovery_behaviors.h>
#include <costmap_2d/cost_values.h>

namespace
{

class TestableTebOptimalPlanner : public teb_local_planner::TebOptimalPlanner
{
public:
  using teb_local_planner::TebOptimalPlanner::TebOptimalPlanner;
  using teb_local_planner::TebOptimalPlanner::AddEdgesKinematicsCarlike;
  using teb_local_planner::TebOptimalPlanner::AddTEBVertices;
};

} // namespace

TEST(TEBBasic, autoResizeLargeValueAtEnd)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // add a pose with a large timediff as the last one
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt + 2*dt_hysteresis);

  // auto resize + test of the result
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBBasic, autoResizeSmallValueAtEnd)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // add a pose with a small timediff as the last one
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt - 2*dt_hysteresis);

  // auto resize + test of the result
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBBasic, autoResize)
{
  double dt = 0.1;
  double dt_hysteresis = dt/3.;
  teb_local_planner::TimedElasticBand teb;
  
  teb.addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  for (int i = 1; i < 10; ++i) {
    teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(i * 1., 0., 0.), dt);
  }
  // modify the timediff in the middle and add a pose with a smaller timediff as the last one
  teb.TimeDiff(5) = dt + 2*dt_hysteresis;
  teb.addPoseAndTimeDiff(teb_local_planner::PoseSE2(10., 0., 0.), dt - 2*dt_hysteresis);

  // auto resize
  teb.autoResize(dt, dt_hysteresis, 3, 100, false);
  for (int i = 0; i < teb.sizeTimeDiffs(); ++i) {
    ASSERT_LE(teb.TimeDiff(i), dt + dt_hysteresis + 1e-3) << "dt is greater than allowed: " << i;
    ASSERT_LE(dt - dt_hysteresis - 1e-3, teb.TimeDiff(i)) << "dt is less than allowed: " << i;
  }
}

TEST(TEBKinematicsCarlike, PenalizesBackwardButNotForwardMotion)
{
  teb_local_planner::TebConfig cfg;
  cfg.robot.min_turning_radius = 1.2;

  teb_local_planner::VertexPose start(0., 0., 0.);
  teb_local_planner::VertexPose forward(1., 0., 0.);
  teb_local_planner::VertexPose backward(-1., 0., 0.);
  teb_local_planner::EdgeKinematicsCarlike edge;
  edge.setVertex(0, &start);
  edge.setTebConfig(cfg);

  edge.setVertex(1, &forward);
  edge.computeError();
  EXPECT_DOUBLE_EQ(0., edge.error()[1]);
  EXPECT_DOUBLE_EQ(0., edge.error()[2]);

  edge.setVertex(1, &backward);
  edge.computeError();
  EXPECT_DOUBLE_EQ(1., edge.error()[1]);
  EXPECT_DOUBLE_EQ(0., edge.error()[2]);
}

TEST(TEBKinematicsCarlike, KeepsTurningRadiusAsThirdConstraint)
{
  teb_local_planner::TebConfig cfg;
  cfg.robot.min_turning_radius = 1.2;

  teb_local_planner::VertexPose start(0., 0., 0.);
  teb_local_planner::VertexPose tight_turn(0.5, 0., 1.0);
  teb_local_planner::EdgeKinematicsCarlike edge;
  edge.setVertex(0, &start);
  edge.setVertex(1, &tight_turn);
  edge.setTebConfig(cfg);
  edge.computeError();

  EXPECT_DOUBLE_EQ(0., edge.error()[1]);
  EXPECT_NEAR(0.7, edge.error()[2], 1e-12);
}

TEST(TEBKinematicsCarlike, GraphUsesConfiguredForwardDriveWeight)
{
  teb_local_planner::TebConfig cfg;
  cfg.robot.min_turning_radius = 1.2;
  cfg.optim.weight_kinematics_nh = 11.;
  cfg.optim.weight_kinematics_forward_drive = 22.;
  cfg.optim.weight_kinematics_turning_radius = 33.;

  teb_local_planner::ObstContainer obstacles;
  TestableTebOptimalPlanner planner(cfg, &obstacles);
  planner.teb().addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  planner.teb().addPoseAndTimeDiff(
      teb_local_planner::PoseSE2(-1., 0., 0.), 0.3);
  planner.AddTEBVertices();
  planner.AddEdgesKinematicsCarlike();

  ASSERT_EQ(1u, planner.optimizer()->edges().size());
  auto* edge = dynamic_cast<teb_local_planner::EdgeKinematicsCarlike*>(
      *planner.optimizer()->edges().begin());
  ASSERT_NE(nullptr, edge);
  EXPECT_DOUBLE_EQ(11., edge->information()(0, 0));
  EXPECT_DOUBLE_EQ(22., edge->information()(1, 1));
  EXPECT_DOUBLE_EQ(33., edge->information()(2, 2));
  edge->computeError();
  EXPECT_DOUBLE_EQ(1., edge->error()[1]);
}

TEST(TEBRecovery, LocalWindowUsesGlobalEndpointIndex)
{
  std::vector<geometry_msgs::PoseStamped> plan(21);
  for (int i = 0; i < 21; ++i)
    plan[i].pose.position.x = 30 + i;
  int global_goal_idx = 50;
  ASSERT_TRUE(teb_local_planner::reduceHorizon(plan, global_goal_idx, 100, 1));
  EXPECT_EQ(11u, plan.size());
  EXPECT_EQ(40, global_goal_idx);
  EXPECT_DOUBLE_EQ(global_goal_idx, plan.back().pose.position.x);
  EXPECT_DOUBLE_EQ(30., plan.front().pose.position.x);
}

TEST(TEBRecovery, RepeatedFailuresFurtherShortenRatherThanLengthenHorizon)
{
  std::vector<geometry_msgs::PoseStamped> plan(21);
  for (int i = 0; i < 21; ++i)
    plan[i].pose.position.x = 30 + i;
  int global_goal_idx = 50;
  ASSERT_TRUE(teb_local_planner::reduceHorizon(plan, global_goal_idx, 100, 10));
  EXPECT_EQ(6u, plan.size());
  EXPECT_EQ(35, global_goal_idx);
  EXPECT_DOUBLE_EQ(global_goal_idx, plan.back().pose.position.x);
}

TEST(TEBRecovery, FinalGoalAndItsOrientationRemainAuthoritative)
{
  std::vector<geometry_msgs::PoseStamped> plan(21);
  plan.back().pose.orientation.z = 0.5;
  int global_goal_idx = 99;
  EXPECT_FALSE(teb_local_planner::reduceHorizon(plan, global_goal_idx, 100, 10));
  EXPECT_EQ(21u, plan.size());
  EXPECT_EQ(99, global_goal_idx);
  EXPECT_DOUBLE_EQ(0.5, plan.back().pose.orientation.z);
}

TEST(TEBRecovery, SmallAndPrunedWindowsKeepAtLeastStartAndGoal)
{
  for (int size = 0; size < 10; ++size)
  {
    std::vector<geometry_msgs::PoseStamped> plan(size);
    int goal_idx = 30 + size - 1;
    for (int i = 0; i < size; ++i)
      plan[i].pose.position.x = 30 + i;
    const bool changed = teb_local_planner::reduceHorizon(plan, goal_idx, 100, 10);
    EXPECT_EQ(size > 2, changed);
    if (changed)
    {
      EXPECT_GE(plan.size(), 2u);
      EXPECT_LT(plan.size(), static_cast<size_t>(size));
      EXPECT_DOUBLE_EQ(goal_idx, plan.back().pose.position.x);
    }
  }
}

TEST(TEBRecovery, FullVehicleFootprintStillRejectsObstacles)
{
  teb_local_planner::TebConfig cfg;
  teb_local_planner::ObstContainer obstacles;
  teb_local_planner::TebOptimalPlanner planner(cfg, &obstacles);
  planner.teb().addPose(teb_local_planner::PoseSE2(0., 0., 0.));
  planner.teb().addPoseAndTimeDiff(teb_local_planner::PoseSE2(0.2, 0., 0.05), 0.4);
  planner.teb().addPoseAndTimeDiff(teb_local_planner::PoseSE2(0.4, 0.02, 0.1), 0.4);
  costmap_2d::Costmap2D map(200, 200, 0.05, -5., -5., 0);
  base_local_planner::CostmapModel model(map);
  std::vector<geometry_msgs::Point> footprint(4);
  footprint[0].x = footprint[1].x = 0.9;
  footprint[2].x = footprint[3].x = -0.9;
  footprint[0].y = footprint[3].y = 0.5;
  footprint[1].y = footprint[2].y = -0.5;
  EXPECT_TRUE(planner.isTrajectoryFeasible(&model, footprint, 0.5, 1.03, 2));
  unsigned int mx, my;
  ASSERT_TRUE(map.worldToMap(0.9, 0., mx, my));
  map.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(planner.isTrajectoryFeasible(&model, footprint, 0.5, 1.03, 2));
}

TEST(TEBVelocityCommand, DoesNotAverageAcrossForwardReverseCusps)
{
  for (const double direction : {-1., 1.})
  {
    teb_local_planner::TebConfig cfg;
    cfg.robot.min_turning_radius = 1.35;
    teb_local_planner::TebOptimalPlanner planner(cfg);
    planner.teb().addPose(0., 0., 0.);
    planner.teb().addPoseAndTimeDiff(direction * 0.18, 0., 0., 0.3);
    planner.teb().addPoseAndTimeDiff(direction * -0.04, 0., 0., 0.3);
    double vx, vy, omega;
    ASSERT_TRUE(planner.getVelocityCommand(vx, vy, omega, 2));
    EXPECT_NEAR(direction * 0.6, vx, 1e-12);
    EXPECT_DOUBLE_EQ(0., vy);
    EXPECT_DOUBLE_EQ(0., omega);
  }
}

TEST(TEBVelocityCommand, RecordedM2TurnKeepsFirstGearAndTurningRadius)
{
  // Field capture: each segment has radius 1.35 m. Averaging across their
  // cusp used to reverse the first command and collapse its radius to 0.23 m.
  teb_local_planner::TebConfig cfg;
  cfg.robot.min_turning_radius = 1.35;
  teb_local_planner::TebOptimalPlanner planner(cfg);
  planner.teb().addPose(1.2381639998488156, -2.222195672230098,
                       -2.0755298385083885);
  planner.teb().addPoseAndTimeDiff(1.15853226852593, -2.3932582008963488,
                                  -1.935716533014051, 0.361269271);
  planner.teb().addPoseAndTimeDiff(1.2261756114370648, -2.149301105810842,
                                  -1.7481108008635926, 0.390362014);
  double vx, vy, omega;
  ASSERT_TRUE(planner.getVelocityCommand(vx, vy, omega, 2));
  EXPECT_NEAR(0.5222950887253943, vx, 1e-10);
  EXPECT_NEAR(0.387005806243447, omega, 1e-10);
  EXPECT_GT(vx / omega, 1.34);
  EXPECT_LT(vx / omega, 1.36);
}

TEST(TEBVelocityCommand, SameDirectionRetainsLookaheadSmoothing)
{
  for (const double direction : {-1., 1.})
  {
    teb_local_planner::TebConfig cfg;
    cfg.robot.min_turning_radius = 1.35;
    teb_local_planner::TebOptimalPlanner planner(cfg);
    planner.teb().addPose(0., 0., 0.);
    planner.teb().addPoseAndTimeDiff(direction * 0.1, 0., 0., 0.2);
    planner.teb().addPoseAndTimeDiff(direction * 0.4, 0., 0., 0.4);
    planner.teb().addPoseAndTimeDiff(direction * 0.2, 0., 0., 0.3);
    double vx, vy, omega;
    ASSERT_TRUE(planner.getVelocityCommand(vx, vy, omega, 2));
    EXPECT_NEAR(direction * (0.4 / 0.6), vx, 1e-12);
  }
}

TEST(TEBVelocityCommand, LongLookaheadIncludesOnlyTimeBeforeFirstCusp)
{
  teb_local_planner::TebConfig cfg;
  cfg.robot.min_turning_radius = 1.35;
  teb_local_planner::TebOptimalPlanner planner(cfg);
  planner.teb().addPose(0., 0., 0.);
  planner.teb().addPoseAndTimeDiff(0.1, 0., 0., 0.1);
  planner.teb().addPoseAndTimeDiff(0.3, 0., 0., 0.2);
  planner.teb().addPoseAndTimeDiff(0.1, 0., 0., 0.3);
  double vx, vy, omega;
  ASSERT_TRUE(planner.getVelocityCommand(vx, vy, omega, 5));
  EXPECT_NEAR(1., vx, 1e-12);
}

TEST(TEBVelocityCommand, LeadingStationaryIntervalDoesNotHideReverseCusp)
{
  teb_local_planner::TebConfig cfg;
  cfg.robot.min_turning_radius = 1.35;
  teb_local_planner::TebOptimalPlanner planner(cfg);
  planner.teb().addPose(0., 0., 0.);
  planner.teb().addPoseAndTimeDiff(0., 0., 0., 0.1);
  planner.teb().addPoseAndTimeDiff(-0.18, 0., 0., 0.3);
  planner.teb().addPoseAndTimeDiff(0.04, 0., 0., 0.3);
  double vx, vy, omega;
  ASSERT_TRUE(planner.getVelocityCommand(vx, vy, omega, 3));
  EXPECT_NEAR(-0.45, vx, 1e-12);
}

TEST(TEBVelocityCommand, DifferentialDriveAndHolonomicBehaviorIsUnchanged)
{
  for (const bool holonomic : {false, true})
  {
    teb_local_planner::TebConfig cfg;
    cfg.robot.min_turning_radius = holonomic ? 1.35 : 0.;
    cfg.robot.max_vel_y = holonomic ? 0.4 : 0.;
    teb_local_planner::TebOptimalPlanner planner(cfg);
    planner.teb().addPose(0., 0., 0.);
    planner.teb().addPoseAndTimeDiff(0.18, 0., 0., 0.3);
    planner.teb().addPoseAndTimeDiff(-0.04, 0., 0., 0.3);
    double vx, vy, omega;
    ASSERT_TRUE(planner.getVelocityCommand(vx, vy, omega, 2));
    EXPECT_NEAR(-0.04 / 0.6, vx, 1e-12);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
