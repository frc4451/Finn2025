package frc.robot.autos;

import java.util.ArrayList;

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

    private Command score(double volts, double sec) {
        return Commands.deadline(
                Commands.waitSeconds(sec),
                coral.runCoral(volts),
                drive.driveCommand(() -> 0.0, () -> 0.0));
    }

    private Command score(double volts) {
        return Commands.deadline(
                Commands.waitSeconds(1),
                coral.runCoral(volts),
                drive.driveCommand(() -> 0.0, () -> 0.0));
    }

    private Command wait(Double sec) {
        return Commands.deadline(
                Commands.waitSeconds(sec),
                drive.driveCommand(() -> 0.0, () -> 0.0));
    }

    public Command ResetOdometry(ChoreoPaths path) {
        return Commands.sequence(
                factory.resetOdometry(path.name),
                wait(0.01),
                factory.resetOdometry(path.name),
                wait(0.01));
    }

    public Command move(ChoreoPaths path) {
        return Commands.sequence(
                factory.trajectoryCmd(path.name),
                wait(0.15));
    }

    public Command Shpeal() {
        return Commands.sequence(
                ResetOdometry(ChoreoPaths.SMtoCGH),
                move(ChoreoPaths.SMtoCGH),
                score(6));
    }

    public Command Wailmer() {
        return Commands.sequence(
                ResetOdometry(ChoreoPaths.SLtoCIJ),
                move(ChoreoPaths.SLtoCIJ),
                score(5),
                move(ChoreoPaths.CIJtoHL),
                move(ChoreoPaths.HLtoCKL),
                score(6),
                move(ChoreoPaths.CKLtoHL));

    }

    public Command Seel() {
        return Commands.sequence(
                ResetOdometry(ChoreoPaths.SRtoCEF),
                move(ChoreoPaths.SRtoCEF),
                score(5),
                wait(0.15),
                move(ChoreoPaths.CEFtoHR),
                move(ChoreoPaths.HRtoCCD),
                score(6));
        // time elapsed AS OF 2/27 - 14.8 sec
        // time elapsed AS OF 3/15 - 14.2 sec
    }

    public Command SeelTest() {

        return Commands.sequence(
                ResetOdometry(ChoreoPaths.SRtoCEFtest),
                move(ChoreoPaths.SRtoCEFtest),
                score(5, .5));
    }
}
