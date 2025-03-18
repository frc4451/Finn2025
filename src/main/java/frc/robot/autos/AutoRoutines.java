package frc.robot.autos;

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

    private Command score(double scoreVolts) {
        return Commands.deadline(
                Commands.waitSeconds(1),
                coral.runCoral(scoreVolts),
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

    public Command Shpeal() {

        return Commands.sequence(
                ResetOdometry(ChoreoPaths.SMtoCGH),
                factory.trajectoryCmd(ChoreoPaths.SMtoCGH.name),
                wait(0.15),
                score(6));
    }

    public Command Wailmer() {

        return Commands.sequence(
                ResetOdometry(ChoreoPaths.SLtoCIJ),
                factory.trajectoryCmd(ChoreoPaths.SLtoCIJ.name),
                wait(0.15),
                score(5),
                factory.trajectoryCmd(ChoreoPaths.CIJtoHL.name),
                wait(.15),
                factory.trajectoryCmd(ChoreoPaths.HLtoCKL.name),
                wait(0.15),
                score(6),
                factory.trajectoryCmd(ChoreoPaths.CKLtoHL.name));

    }

    public Command Seel() {

        return Commands.sequence(
                ResetOdometry(ChoreoPaths.SRtoCEF),
                factory.trajectoryCmd(ChoreoPaths.SRtoCEF.name),
                wait(0.15),
                score(5),
                wait(0.15),
                factory.trajectoryCmd(ChoreoPaths.CEFtoHR.name),
                wait(.5),
                factory.trajectoryCmd(ChoreoPaths.HRtoCCD.name),
                wait(0.15),
                score(6),
                factory.trajectoryCmd(ChoreoPaths.CCDtoHR.name));
        // time elapsed AS OF 2/27 - 14.8 sec
        // time elapsed AS OF 3/15 - 14.2 sec
    }

    public Command SeelTest() {

        return Commands.sequence(
                ResetOdometry(ChoreoPaths.SRtoCEFtest),
                factory.trajectoryCmd(ChoreoPaths.SRtoCEFtest.name),
                wait(0.15),
                score(5),
                wait(0.15));
    }
}
