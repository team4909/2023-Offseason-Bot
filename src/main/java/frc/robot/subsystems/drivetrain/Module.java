package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

public final class Module {

  private final int m_index;
  private final ModuleIO m_moduleIO;
  private final ModuleIOInputsAutoLogged m_moduleInputs = new ModuleIOInputsAutoLogged();

  public Module(ModuleIO io, int index) {
    m_moduleIO = io;
    m_index = index;
  }

  public void periodic() {
    m_moduleIO.updateInputs(m_moduleInputs);
    Logger.getInstance().processInputs("Drivetrain/Module " + m_index, m_moduleInputs);
    Logger.getInstance().recordOutput("Test/Measured Speed Module " + m_index, getDriveSpeedMetersPerSec());
    Logger.getInstance().recordOutput("Test/Measured Angle Module Degrees " + m_index,
        getCurrentAngleRad().getDegrees());
  }

  public SwerveModuleState setSetpoint(SwerveModuleState state) {
    final SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getCurrentAngleRad());
    // Compensate for wheel scrub
    optimizedState.speedMetersPerSecond *= Math.cos(Units.rotationsToRadians(m_moduleInputs.steerClosedLoopError));
    final double setpointAngleRot = optimizedState.angle.getRotations();
    final double setpointVelocityRPS = Units.radiansToRotations(
        optimizedState.speedMetersPerSecond / DriveConstants.kWheelRadiusMeters) * DriveConstants.kDriveRatio;

    Logger.getInstance().recordOutput("Test/Desired Speed Module " + m_index, optimizedState.speedMetersPerSecond);
    m_moduleIO.setSteerRotations(setpointAngleRot);
    m_moduleIO.setDriveRPS(setpointVelocityRPS);
    return state;
  }

  public void stop() {
    m_moduleIO.setSteerVoltage(0.0);
    m_moduleIO.setDriveVoltage(0.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveSpeedMetersPerSec(), getCurrentAngleRad());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePositionMeters(), getCurrentAngleRad());
  }

  private Rotation2d getCurrentAngleRad() {
    return Rotation2d.fromRotations(m_moduleInputs.latencyCompensatedSteerPositionRotations);
  }

  private double getDriveSpeedMetersPerSec() {
    return Units.rotationsToRadians(m_moduleInputs.driveVelocityRPS / DriveConstants.kDriveRatio)
        * DriveConstants.kWheelRadiusMeters;
  }

  private double getDrivePositionMeters() {
    return Units
        .rotationsToRadians(m_moduleInputs.latencyCompensatedDrivePositionRotations / DriveConstants.kDriveRatio)
        * DriveConstants.kWheelRadiusMeters;
  }
}
