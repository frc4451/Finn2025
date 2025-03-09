package frc.robot;

import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
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
                coral.runCoral(5.0),
                drive.driveCommand(() -> 0.0, () -> 0.0));
    }

    private Command wait(Double sec) {
        return Commands.deadline(
                Commands.waitSeconds(sec),
                drive.driveCommand(() -> 0.0, () -> 0.0));
    }

    public Command Shpeal() {
        String trajectory = "SMtoCGH";

        return Commands.sequence(
                factory.resetOdometry(trajectory),
                factory.trajectoryCmd(trajectory),
                wait(0.15),
                score());
    }

    public Command Wailmer() {
        String trajectory1 = "SLtoCIJ";
        String trajectory2 = "CIJtoHL";
        String trajectory3 = "HLtoCKL";

        return Commands.sequence(
                factory.resetOdometry(trajectory1),
                factory.trajectoryCmd(trajectory1),
                wait(0.15),
                score(),
                wait(0.15),
                factory.trajectoryCmd(trajectory2),
                wait(.15),
                factory.trajectoryCmd(trajectory3),
                wait(0.15),
                score());

    }

    public Command Seel() {
        String trajectory1 = "SRtoCEF";
        String trajectory2 = "CEFtoHR";
        String trajectory3 = "HRtoCCD";

        return Commands.sequence(
                factory.resetOdometry(trajectory1),
                factory.trajectoryCmd(trajectory1),
                wait(0.15),
                score(),
                wait(0.15),
                factory.trajectoryCmd(trajectory2),
                wait(.5),
                factory.trajectoryCmd(trajectory3),
                wait(0.15),
                score());
        // time elapsed AS OF 2/27 - 14.8 sec
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
