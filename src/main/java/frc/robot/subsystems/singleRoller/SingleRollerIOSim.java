package frc.robot.subsystems.singleRoller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SingleRollerIOSim implements SingleRollerIO {
    private final DCMotorSim sim;

    private final PIDController controller;

    private double appliedVolts = 0.0;

    private boolean closedLoop = false;

    public SingleRollerIOSim() {
        sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), SingleRollerConstants.kMoi,
                SingleRollerConstants.kMotorReduction), DCMotor.getNEO(1));
        controller = new PIDController(SingleRollerConstants.kP, SingleRollerConstants.kI, SingleRollerConstants.kD);
    }

    @Override
    public void updateInputs(SingleRollerIOInputs inputs) {
        if (DriverStation.isDisabled()) {
            stop();
        } else if (closedLoop) {
            runVolts(controller.calculate(sim.getAngularPositionRad()));
        }

        sim.update(SingleRollerConstants.kUpdatePeriodMilliseconds);

        inputs.singleRollerPositionRad = sim.getAngularPositionRad();
        inputs.singleRollerVelocityRadPerSec = sim.getAngularVelocityRadPerSec();

        inputs.singleRollerAppliedVolts = appliedVolts;
        inputs.singleRollerCurrentAmps = sim.getCurrentDrawAmps();
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
