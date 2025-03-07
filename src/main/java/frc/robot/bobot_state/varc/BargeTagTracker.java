package frc.robot.bobot_state.varc;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.BobotState;
import frc.robot.field.FieldUtils;
import java.util.Optional;

public class BargeTagTracker extends TargetAngleTracker {
  private Rotation2d rotationTarget;

  public BargeTagTracker() {
    super();
  }

  public Rotation2d getRotationTarget() {
    return rotationTarget;
  }

  public void update() {
    Pose3d robotPose = new Pose3d(BobotState.getGlobalPose());

    // TODO Handle Vision targeting here

    this.rotationTarget = (FieldUtils.getBargeTag().pose().getRotation().toRotation2d().plus(Rotation2d.kPi));
  }
}
