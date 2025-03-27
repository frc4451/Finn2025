package frc.robot.subsystems.singleRoller;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SingleRollerConstants {

    public static final int kSingleRollerMotorId = 5;

    private final double reduction = 0.0;
    private double positionGoalRotations = 0;

    public static final MotorType kSingleRollerMotorType = MotorType.kBrushed;
    public static final int kUpdatePeriodMilliseconds = 20;
    public static final double kMotorReduction = 1.0;
    public static final double kMoi = 1.0;

    public static final double kP = 0.002;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kF = 0;
}
