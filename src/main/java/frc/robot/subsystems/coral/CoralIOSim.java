package frc.robot.subsystems.coral;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class CoralIOSim implements CoralIO {
    private final DCMotorSim sim;

    private final PIDController controller;

    private double appliedVolts = 0.0;

    private boolean closedLoop = false;

    public CoralIOSim() {
        sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), CoralConstants.kMoi, CoralConstants.kMotorReduction), DCMotor.getNEO(1));
        controller = new PIDController(CoralConstants.kP, CoralConstants.kI, CoralConstants.kD);
    }

    @Override
    public void updateInputs(CoralIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        } else if (closedLoop) {
            runVolts(controller.calculate(sim.getAngularPositionRad()));
        }

        sim.update(CoralConstants.kUpdatePeriodMilliseconds);

        inputs.coralPositionRad = sim.getAngularPositionRad();
        inputs.coralVelocityRadPerSec = sim.getAngularVelocityRadPerSec();

        inputs.coralAppliedVolts = appliedVolts;
        inputs.coralCurrentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void runVolts(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12, 12);
        sim.setInputVoltage(appliedVolts);
    }

    @Override
    public void stop() {
        runVolts(0.0);
    }

}
