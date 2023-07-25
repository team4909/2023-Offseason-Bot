package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionRotations;
    public double driveRotorPositionRotations;
    public double driveVelocityRPS;
    public double driveClosedLoopReference;
    public double driveClosedLoopError;
    public double driveClosedLoopOutput;
    public double driveTorqueCurrentAmps;
    public double driveStatorCurrentAmps;
    public double driveSupplyCurrentAmps;
    public double driveSupplyVoltage;
    public double driveTempCelsius;

    public double steerPositionRotations;
    public double steerRotorPositionRotations;
    public double steerVelocityRPS;
    public double steerClosedLoopReference;
    public double steerClosedLoopError;
    public double steerClosedLoopOutput;
    public double steerTorqueCurrentAmps;
    public double steerStatorCurrentAmps;
    public double steerSupplyCurrentAmps;
    public double steerSupplyVoltage;
    public double steerTempCelsius;

    public double azimuthPositionRotations;
    public double azimuthVelocityRPS;
    public double azimuthAbsolutePositionRotations;
    public double azimuthUnfilteredVelocityRPS;
    public double azimuthPositionSinceBoot;

    public double latencyCompensatedDrivePositionRotations;
    public double latencyCompensatedSteerPositionRotations;
  }

  default void updateInputs(ModuleIOInputs inputs) {};

  default void setDriveRPS(double speedRPS) {};

  default void setSteerRotations(double angleRotations) {};

  default void setDriveVoltage(double volts) {};

  default void setSteerVoltage(double volts) {};

}
