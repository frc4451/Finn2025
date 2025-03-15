package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class DriveCommandConstants {
    public static ProfiledPIDController makeAngleController() {
        ProfiledPIDController angleController = new ProfiledPIDController(1.8, 0.0, 0.0,
                new TrapezoidProfile.Constraints(8.0, 20.0));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(Units.degreesToRadians(2));

        return angleController;
    }

    public static PIDController makeTranslationController() {
        PIDController translationController = new PIDController(5, 0, 0.1);
        translationController.setTolerance(Units.inchesToMeters(0.5));

        return translationController;
    }
}
