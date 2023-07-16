package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  public Flywheel() {
    this(getIO());
  }

  public static FlywheelIO getIO() {
    switch (Constants.kCurrentMode) {
      case kReal:
        return new FlywheelIOSparkMax();
      case kSim:
        return new FlywheelIOSim();
      default:

        return new FlywheelIO() {

        };
    }
  }

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.kCurrentMode) {
      case kReal:
      case kReplay:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case kSim:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  @Override
  public void periodic() {
    io.setVelocity(900, 0);
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Flywheel", inputs);

    // Log flywheel speed in RPM
    Logger.getInstance().recordOutput("FlywheelSpeedRPM", getVelocityRPM());
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec, ffModel.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.getInstance().recordOutput("FlywheelSetpointRPM", velocityRPM);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }
}
