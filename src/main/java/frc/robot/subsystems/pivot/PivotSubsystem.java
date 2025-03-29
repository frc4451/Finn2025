package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOInputsAutoLogged;

public class PivotSubsystem extends SubsystemBase {
    protected final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    protected PivotIO io;

    public PivotSubsystem(PivotIO io) {
        this.io = io;

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Coral", inputs);
        if (DriverStation.isDisabled()) {
            io.stop();
        }
    }

    /** Using direct voltage control because why not */
    public Command runPivot(double inputVolts) {
        return startEnd(() -> io.runVolts(inputVolts), () -> io.stop());
    }

    public Command setReference(double setpoint) {
        return startEnd(() -> io.setReference(setpoint), () -> io.stop());
    }
}
