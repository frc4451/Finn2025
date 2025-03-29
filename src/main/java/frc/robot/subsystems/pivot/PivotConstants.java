package frc.robot.subsystems.pivot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class PivotConstants {

    public static final int kPivotMotorId = 7;

    private final double reduction = 0.0;
    private double positionGoalRotations = 0;

    public static final MotorType kPivotMotorType = MotorType.kBrushless;
    public static final int kUpdatePeriodMilliseconds = 20;
    public static final double kMotorReduction = 1.0;
    public static final double kMoi = 1.0;

    public static final double kP = 0.005;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kF = 0;
}
