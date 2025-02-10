package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoRoutines {

    private final AutoFactory factory;
    private final CoralSubsystem Coral;

    public AutoRoutines(AutoFactory factory, CoralSubsystem Coral) {
        this.factory = factory;
        this.Coral = Coral;

    }

    public Command OreoTest() {
        return Commands.sequence(
                factory.resetOdometry("Test"),
                factory.trajectoryCmd("Test"));
    }

    public Command Spaghetti() {
        return Commands.sequence(
                factory.resetOdometry("Spaghetti"),
                factory.trajectoryCmd("Spaghetti"));
    }

    public Command Bottom() {
        return Commands.sequence(
                factory.resetOdometry("Bottom path"),
                factory.trajectoryCmd("Bottom path"));
    }

    public Command Top() {
        return Commands.sequence(
                factory.resetOdometry("Top path"),
                factory.trajectoryCmd("Top Path"));
    }

    public Command Middle() {
        return Commands.sequence(
                factory.resetOdometry("Middle path"),
                factory.trajectoryCmd("Middle Path"));
    }

}
