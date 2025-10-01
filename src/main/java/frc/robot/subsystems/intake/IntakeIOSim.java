package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {

  private FlywheelSim simIntake = new FlywheelSim(DCMotor.getFalcon500(1), 1.5, 0.004);

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.speed = simIntake.getAngularVelocityRadPerSec();
  }

  public void setIntakeSpeed(double velocity) {
    simIntake.setState(velocity);
  }

  public void stop() {
    setIntakeSpeed(0);
  }
}
