package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface ImuIO {
  @AutoLog
  public static class ImuIOInputs {
    public double yawPositionDeg;
    public double pitchPositionDeg;
    public double rollPositionDeg;
    public double yawVelocityDegPerSec;
    public double pitchVelocityDegPerSec;
    public double rollVelocityDegPerSec;
    public double supplyVoltage;
    public double upTimeSeconds;
    public double tempCelsius;

    public double latencyCompensatedYaw;
  }

  default void updateInputs(ImuIOInputs inputs) {};

  default void updateSim(double dthetaRad) {};
}
