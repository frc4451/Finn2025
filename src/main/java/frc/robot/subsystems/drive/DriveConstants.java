package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final int kFrontLeftId = 1;
    public static final int kFrontRightId = 2;
    public static final int kBackLeftId = 3;
    public static final int kBackRightId = 4;
    public static final int kPigeonId = 5;

    public static final double kRampRateSeconds = 0.3;

    public static final boolean kLeftInverted = true;
    public static final boolean kRightInverted = false;

    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final double kTrackWidthMeters = Units.inchesToMeters(26.0);
    public static final double kWheelRadiusMeters = Units.inchesToMeters(6.0);
    public static final double kMotorReduction = 1.0;

    /** Real values for PIDF */
    public static final double kMotorKp = 1.0;
    public static final double kMotorKi = 0.0;
    public static final double kMotorKd = 0.0;
    public static final double kMotorKf = 0.0;

    /** Sim values for PID */
    public static final double kSimKp = 0.44;
    public static final double kSimKi = 0.0;
    public static final double kSimKd = 0.0;

}
