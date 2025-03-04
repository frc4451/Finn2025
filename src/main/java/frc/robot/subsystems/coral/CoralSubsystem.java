package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralSubsystem extends SubsystemBase {

    protected final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();
    protected CoralIO io;
    private Servo servo = new Servo(0);

    public CoralSubsystem(CoralIO io) {
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
    public Command runCoral(double inputVolts) {
        return startEnd(() -> io.runVolts(inputVolts), () -> io.stop());
    }

    public Command rampUp(double V) {
        return Commands.runOnce(() -> servo.set(V), this);
    }
}
