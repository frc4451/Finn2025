package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.pivot.PivotConstants;

public class PivotIOSim implements PivotIO {
    private final DCMotorSim sim;

    private final PIDController controller;

    private double appliedVolts = 0.0;

    private boolean closedLoop = false;

    public PivotIOSim() {
        sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), PivotConstants.kMoi,
                PivotConstants.kMotorReduction), DCMotor.getNEO(1));
        controller = new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        } else if (closedLoop) {
            runVolts(controller.calculate(sim.getAngularPositionRad()));
        }

        sim.update(PivotConstants.kUpdatePeriodMilliseconds);

        inputs.PivotPositionRad = sim.getAngularPositionRad();
        inputs.PivotVelocityRadPerSec = sim.getAngularVelocityRadPerSec();

        inputs.PivotAppliedVolts = appliedVolts;
        inputs.PivotCurrentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void runVolts(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12, 12);
        sim.setInputVoltage(appliedVolts);
    }

    // @Override
    // public void setReference(double setpoint) {
    // PivotPosition
    // }

    @Override
    public void stop() {
        runVolts(0.0);
    }
}
