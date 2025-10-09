// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drive;
import frc.robot.util.ChassisAcceleration;

public class RobotContainer {
  public static final double MAX_LINEAR_VEL_mps = 4.8;
  public static final double MAX_FORWARD_ACC_mps = 55.0;
  public static final double MAX_AUTO_FORWARD_ACC_mps = 55.0;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private ChassisSpeeds acc;
  inputSpeeds = new ChassisSpeeds();

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final Drive drivetrain = TunerConstants.DriveTrain; // My drivetrain

  ChassisSpeeds measuredSpeeds = getChassisSpeeds();measuredAcc=ChassisAcceleration.calculate(lastSpeeds,measuredSpeeds,Constants.globalDelta_s);lastSpeeds=measuredSpeeds;
  ChassisSpeeds inputAcc = ChassisAcceleration.fromChassisSpeeds(measuredSpeeds, inputSpeeds,
      Constants.globalDelta_s);

  acc=inputAcc;

  acc=

  accLimitForward(acc, measuredSpeeds);
        acc = accLimitAngular(acc, measuredSpeeds);
        acc = accLimitTilt(acc);
        acc = accLimitSkid(acc);

  private final LegacySwerveRequest.FieldCentric drive = new LegacySwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final LegacySwerveRequest.SwerveDriveBrake brake = new LegacySwerveRequest.SwerveDriveBrake();
  private final LegacySwerveRequest.PointWheelsAt point = new LegacySwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    logger.telemeterize(drivetrain.getState());
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public ChassisSpeeds accLimitForward(ChassisSpeeds acc, ChassisSpeeds vel) {
    ChassisSpeeds res = acc;
    double accMag_mps = ChassisAcceleration.magnitude(acc);
    double velMag_mps = ChassisAcceleration.magnitude(vel);
    double accAng_rad = ChassisAcceleration.angle(acc);
    double velAng_rad = ChassisAcceleration.angle(vel);

    if (accMag_mps == 0.0) {
      return res;
    }

    if (velMag_mps == 0.0) {
      res.vxMetersPerSecond *= MAX_FORWARD_ACC_mps / accMag_mps;
      res.vyMetersPerSecond *= MAX_FORWARD_ACC_mps / accMag_mps;
    } else {

      double alpha = Math.cos(accAng_rad - velAng_rad);

      if (alpha > 0) {
        double max = DriverStation.isAutonomous() ? MAX_AUTO_FORWARD_ACC_mps : MAX_FORWARD_ACC_mps;
        double maxFwdAcc = max * (1.0 - velMag_mps / MAX_LINEAR_VEL_mps );
        double outMag = Math.min(accMag_mps, maxFwdAcc);
        res.vxMetersPerSecond *= outMag / accMag_mps;
        res.vyMetersPerSecond *= outMag / accMag_mps;
      }
    }

    return res;

  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = drivetrain.getModule(i).getCurrentState();
    }
    return states;
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    double i2m = Units.inchesToMeters(1.0);
    return new Translation2d[] {
        new Translation2d(i2m * TunerConstants.FrontLeft.LocationX,
            i2m * TunerConstants.FrontLeft.LocationY),
        new Translation2d(i2m * TunerConstants.FrontRight.LocationX,
            i2m * TunerConstants.FrontRight.LocationY),
        new Translation2d(i2m * TunerConstants.BackLeft.LocationX,
            i2m * TunerConstants.BackLeft.LocationY),
        new Translation2d(i2m * TunerConstants.BackRight.LocationX,
            i2m * TunerConstants.BackRight.LocationY)
    };
  }
}
