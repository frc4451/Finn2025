package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final int kFrontLeftId = 1;
    public static final int kFrontRightId = 2;
    public static final int kBackLeftId = 3;
    public static final int kBackRightId = 4;
    public static final int kPidgeonId = 5;

    public static final double kRampRateSeconds = 0.3;

    public static final boolean kLeftInverted = true;
    public static final boolean kRightInverted = false;

    public static final MotorType kMotorType = MotorType.kBrushed;

    public static final double kTrackWidthMeters = Units.inchesToMeters(26.0);
    public static final double kWheelRadiusMeters = Units.inchesToMeters(6.0);
    public static final double kMotorReduction = 1.0;
}
