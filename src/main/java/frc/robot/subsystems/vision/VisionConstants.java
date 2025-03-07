package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.List;
import java.util.Optional;
import org.photonvision.simulation.VisionSystemSim;

public class VisionConstants {
        public static final record AprilTagCameraConfig(VisionSource source, SimCameraConfig simConfig) {
        }

        public static enum PoseEstimationMethod {
                MULTI_TAG,
                SINGLE_TAG
        }

        public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
                        .loadField(AprilTagFields.k2025Reefscape);

        public static final Optional<VisionSystemSim> aprilTagSim = Constants.currentMode == Mode.SIM
                        ? Optional.of(new VisionSystemSim("AprilTagSim"))
                        : Optional.empty();

        public static final List<AprilTagCameraConfig> finnConfigs = List.of(
                        new AprilTagCameraConfig(
                                        new VisionSource(
                                                        "FrontRight",
                                                        new Transform3d(
                                                                        new Translation3d(
                                                                                        36.7 / 100.0, // forward+
                                                                                        -15.9 / 100.0, // left+
                                                                                        16.6 / 100.0), // up+
                                                                        new Rotation3d(0, Units.degreesToRadians(-10),
                                                                                        Units.degreesToRadians(30)))),
                                        SimCameraConfig.ARDUCAM_OV9281_70),

                        new AprilTagCameraConfig(
                                        new VisionSource(
                                                        "FrontLeft",
                                                        new Transform3d(
                                                                        new Translation3d(
                                                                                        36.7 / 100.0,
                                                                                        15.9 / 100.0,
                                                                                        16.6 / 100.0),
                                                                        new Rotation3d(0, Units.degreesToRadians(-10),
                                                                                        Units.degreesToRadians(-30)))),
                                        SimCameraConfig.ARDUCAM_OV9281_70));

        public static final double ambiguityCutoff = 0.05;
        public static final double singleTagPoseCutoffMeters = 4;

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.35, 0.35, 0.75);
}
