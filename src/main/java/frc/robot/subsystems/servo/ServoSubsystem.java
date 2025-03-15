package frc.robot.subsystems.servo;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {
    private final ServoIOInputsAutoLogged inputs = new ServoIOInputsAutoLogged();
    private ServoIO io;

    public ServoSubsystem(ServoIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Servo", inputs);
    }

    public Command setAngle(double angle) {
        return runOnce(() -> io.setAngle(angle));
    }
}