package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private State currentState;

  public enum State {
    // Intake
    DISABLED,
    SPITTING,
    INTAKING,
    IDLE
  }

  public double intakeSpeed = 0.0;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private static Intake instance;

  public static Intake getInstance() {
    if (instance == null) {
      System.out.println("Intake initialized");
      switch (Constants.currentMode) {
        case REAL:
          break;

        case SIM:
          // Sim robot, instantiate physics sim IO implementations
          instance = new Intake(new IntakeIOSim());
          break;

        default:
          // Replayed robot, disable IO implementations
          instance = new Intake(new IntakeIO() {});
          break;
      }
    }
    return instance;
  }

  private Intake(IntakeIO io) {
    super("Intake");
    this.io = io;

    currentState = State.IDLE;

    switch (currentState) {
      case DISABLED:
        stop();
      case IDLE:
        stop();
      case INTAKING:
        giveToIndexer();
      case SPITTING:
        spit();
    }
  }

  public void giveToIndexer() {
    intakeSpeed = 0.25;
  }

  public void spit() {
    intakeSpeed = -0.25;
  }

  public void stop() {
    intakeSpeed = 0;
    io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    io.setIntakeSpeed(intakeSpeed);
    Logger.recordOutput("Intake/TargetBottomSpeed", intakeSpeed);
  }
}
