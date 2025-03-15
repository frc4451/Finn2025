package frc.robot.bobot_state.varc;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.BobotState;
import frc.robot.field.FieldUtils;

public class HPSTagTracker extends TargetAngleTracker {
  private Rotation2d rotationTarget;

  public HPSTagTracker() {
    super();
  }

  public Rotation2d getRotationTarget() {
    return rotationTarget;
  }

  public void update() {
    Pose3d robotPose = new Pose3d(BobotState.getGlobalPose());

    // TODO Handle Vision targeting here

    this.rotationTarget = (FieldUtils.getClosestHPSTag().pose().getRotation().toRotation2d());
  }
}
