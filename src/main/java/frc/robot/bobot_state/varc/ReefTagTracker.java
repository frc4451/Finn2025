package frc.robot.bobot_state.varc;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.BobotState;
import frc.robot.field.FieldConstants.ReefFace;
import frc.robot.field.FieldUtils;

public class ReefTagTracker extends TargetAngleTracker {
  private Rotation2d rotationTarget;

  private ReefFace closestReef;

  public ReefTagTracker() {
    super();
  }

  public Rotation2d getRotationTarget() {
    return rotationTarget;
  }

  public ReefFace getClosestReef() {
    return closestReef;
  }

  public void update() {
    Pose3d robotPose = new Pose3d(BobotState.getGlobalPose());

    // TODO Handle Vision targeting here

    this.rotationTarget = FieldUtils.getClosestReef()
        .tag()
        .pose()
        .getRotation()
        .toRotation2d()
        .plus(Rotation2d.kPi);

    this.closestReef = FieldUtils.getClosestReef();

    // this.rotationTarget =
    // Optional.of(
    // FieldUtils.getClosestReefAprilTag()
    // .pose()
    // .relativeTo(robotPose)
    // .getTranslation()
    // .toTranslation2d()
    // .getAngle()
    // .plus(BobotState.getGlobalPose().getRotation()));
  }
}
