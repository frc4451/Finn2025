package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoRoutines {

    private final AutoFactory factory;
    private final CoralSubsystem coral;
    private final DriveSubsystem drive;

    public AutoRoutines(AutoFactory factory, CoralSubsystem coral, DriveSubsystem drive) {
        this.factory = factory;
        this.coral = coral;
        this.drive = drive;

    }

    private Command score() {
        return Commands.deadline(
                Commands.waitSeconds(1),
                coral.runCoral(8.0),
                drive.driveCommand(() -> 0.0, () -> 0.0));
    }

    private Command wait(int sec) {
        return Commands.deadline(
                Commands.waitSeconds(sec),
                drive.driveCommand(() -> 0.0, () -> 0.0));
    }

    public Command oreoTest() {
        return Commands.sequence(
                factory.resetOdometry("Test"),
                factory.trajectoryCmd("Test"));
    }

    public Command spaghetti() {
        return Commands.sequence(
                factory.resetOdometry("Spaghetti"),
                factory.trajectoryCmd("Spaghetti"));
    }

    public Command bottom() {
        return Commands.sequence(
                factory.resetOdometry("Bottom path"),
                factory.trajectoryCmd("Bottom path"));
    }

    public Command top() {
        return Commands.sequence(
                factory.resetOdometry("Top path"),
                factory.trajectoryCmd("Top Path"));
    }

    public Command middle() {
        return Commands.sequence(
                factory.resetOdometry("Middle path"),
                factory.trajectoryCmd("Middle path"),
                score());
    }

    public Command duolingo() {
        return Commands.sequence(
                factory.resetOdometry("Duolingo"),
                factory.trajectoryCmd("Duolingo"),
                score());
    }

    public Command S2BP() {
        return Commands.sequence(
                factory.resetOdometry("bottomtoCF"),
                factory.trajectoryCmd("bottomtoCF"),
                score(),
                factory.trajectoryCmd("CFtoSta"),
                wait(2),
                factory.trajectoryCmd("StatoBF"),
                score()

        );
    }

    /*
     * public Command SimpleGoScore() {
     * return Command.sequence(
     * //Commands.deadline(Commands.waitSeconds(3),
     * //drive.setVoltageCommand(3.0, 3.0)),
     * Commands.deadline(
     * Commands.waitSeconds(1),
     * coral.runCoral(8.0),
     * drive.driveCommand(() -> 0.0, () -> 0.0)));
     * 
     * }
     */
}
