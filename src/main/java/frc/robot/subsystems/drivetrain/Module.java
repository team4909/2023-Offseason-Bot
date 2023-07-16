package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;

public class Module {

  private final int m_index;
  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final double k2PI = 2 * Math.PI;

  public Module(ModuleIO io, int index) {
    m_io = io;
    m_index = index;
  }

  public void periodic() {
    m_io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drive/Module" + m_index, inputs);
    if (Constants.kCurrentMode.equals(Mode.kSim)) {
      m_io.updateSim();
    }
  }

  public SwerveModuleState setSetpoint(SwerveModuleState state) {
    var optimizedState = SwerveModuleState.optimize(state, getCurrentAngleRad());
    var currentAngleRad = optimizedState.angle.getRadians();
    var currentVelocityRadPerSec = optimizedState.speedMetersPerSecond / DriveConstants.kWheelRadius;
    // optimizedState.speedMetersPerSecond *= Math.cos(inputs.steerClosedLoopError);
    m_io.setSteerRotations(currentAngleRad / k2PI);
    m_io.setDriveRPS(currentVelocityRadPerSec / k2PI);
    return state;
  }

  public void stop() {
    m_io.setSteerVoltage(0.0);
    m_io.setDriveVoltage(0.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        inputs.driveVelocityRPS * k2PI * DriveConstants.kWheelRadius,
        getCurrentAngleRad());

  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        inputs.drivePositionRotations * k2PI * DriveConstants.kWheelRadius,
        getCurrentAngleRad());
  }

  private Rotation2d getCurrentAngleRad() {
    return new Rotation2d(inputs.latencyCompensatedPositionRotations * k2PI);
  }

}
