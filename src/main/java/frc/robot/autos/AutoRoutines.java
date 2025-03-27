package frc.robot.autos;

import java.util.ArrayList;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.singleRoller.SingleRollerSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoRoutines {

    private final AutoFactory factory;
    private final SingleRollerSubsystem coral;
    private final DriveSubsystem drive;

    public AutoRoutines(AutoFactory factory, SingleRollerSubsystem coral, DriveSubsystem drive) {
        this.factory = factory;
        this.coral = coral;
        this.drive = drive;

    }

    private Command score(double volts, double sec) {
        return Commands.deadline(
                Commands.waitSeconds(sec),
                coral.runSingleRoller(volts),
                drive.driveCommand(() -> 0.0, () -> 0.0));
    }

    private Command score(double volts) {
        return Commands.deadline(
                Commands.waitSeconds(1),
                coral.runSingleRoller(volts),
                drive.driveCommand(() -> 0.0, () -> 0.0));
    }

    private Command wait(Double sec) {
        return Commands.deadline(
                Commands.waitSeconds(sec),
                drive.driveCommand(() -> 0.0, () -> 0.0));
    }

    /**
     * Have to do this to fix a funky odometry issue with reseting pose
     * Don't know why it works but it does
     */
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
                ResetOdometry(ChoreoPaths.SMBuffer),
                move(ChoreoPaths.SMBuffer),
                move(ChoreoPaths.SMBuffertoCGH),
                score(6));
    }

    public Command Wailmer() {
        return Commands.sequence(
                ResetOdometry(ChoreoPaths.SLtoCIJ),
                move(ChoreoPaths.SLtoCIJ),
                score(6.5),
                move(ChoreoPaths.CIJtoHL),
                move(ChoreoPaths.HLtoCKL),
                score(6.5),
                move(ChoreoPaths.CKLtoHL));

    }

    public Command Seel() {
        return Commands.sequence(
                ResetOdometry(ChoreoPaths.SRtoCEF),
                move(ChoreoPaths.SRtoCEF),
                score(6.5),
                wait(0.15),
                move(ChoreoPaths.CEFtoHR),
                move(ChoreoPaths.HRtoCCD),
                score(6.5));
        // time elapsed AS OF 2/27 - 14.8 sec
        // time elapsed AS OF 3/15 - 14.2 sec
    }

    public Command SeelTest() {

        return Commands.sequence(
                ResetOdometry(ChoreoPaths.SRtoCEFtest),
                move(ChoreoPaths.SRtoCEFtest),
                score(5, .25),
                move(ChoreoPaths.CEFtoHRtest),
                move(ChoreoPaths.HRtoCCDtest),
                score(6, 0.25));
    }

}
