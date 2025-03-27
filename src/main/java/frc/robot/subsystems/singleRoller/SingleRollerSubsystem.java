package frc.robot.subsystems.singleRoller;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class SingleRollerSubsystem extends SubsystemBase {
    protected final SingleRollerIOInputsAutoLogged inputs = new SingleRollerIOInputsAutoLogged();
    protected SingleRollerIO io;

    public SingleRollerSubsystem(SingleRollerIO io) {
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
    public Command runSingleRoller(double inputVolts) {
        return startEnd(() -> io.runVolts(inputVolts), () -> io.stop());
    }

    public Command setReference(double setpoint) {
        return startEnd(() -> io.setReference(setpoint), () -> io.stop());
    }

}
