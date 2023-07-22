package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;

public class Module {

  private final int m_index;
  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  public Module(ModuleIO io, int index) {
    m_io = io;
    m_index = index;
  }

  public void periodic() {
    m_io.updateInputs(inputs);
    Logger.getInstance().processInputs("Drivetrain/Module " + m_index, inputs);
    if (Constants.kCurrentMode.equals(Mode.kSim)) {
      m_io.updateSim();
    }
    Logger.getInstance().recordOutput("Test/Measured Speed Module " + m_index, getDriveSpeedMetersPerSec());
    Logger.getInstance().recordOutput("Test/Measured Angle Module Degrees " + m_index,
        getCurrentAngleRad().getDegrees());
  }

  public SwerveModuleState setSetpoint(SwerveModuleState state) {
    var optimizedState = SwerveModuleState.optimize(state, getCurrentAngleRad());
    var setpointAngleRot = optimizedState.angle.getRotations();
    var setpointVelocityRPS = Units.radiansToRotations(
        optimizedState.speedMetersPerSecond / DriveConstants.kWheelRadiusMeters) * DriveConstants.kDriveRatio;
    // optimizedState.speedMetersPerSecond *=
    // Math.cos(Units.rotationsToRadians(inputs.steerClosedLoopError));
    Logger.getInstance().recordOutput("Test/Desired Speed Module " + m_index, optimizedState.speedMetersPerSecond);
    m_io.setSteerRotations(setpointAngleRot);
    m_io.setDriveRPS(setpointVelocityRPS);
    return state;
  }

  public void stop() {
    m_io.setSteerVoltage(0.0);
    m_io.setDriveVoltage(0.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveSpeedMetersPerSec(), getCurrentAngleRad());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePositionMeters(), getCurrentAngleRad());
  }

  private Rotation2d getCurrentAngleRad() {
    return Rotation2d.fromRotations(inputs.latencyCompensatedSteerPositionRotations);
  }

  private double getDriveSpeedMetersPerSec() {
    return Units.rotationsToRadians(inputs.driveVelocityRPS / DriveConstants.kDriveRatio)
        * DriveConstants.kWheelRadiusMeters;
  }

  private double getDrivePositionMeters() {
    return Units.rotationsToRadians(inputs.latencyCompensatedDrivePositionRotations / DriveConstants.kDriveRatio)
        * DriveConstants.kWheelRadiusMeters;
  }
}
