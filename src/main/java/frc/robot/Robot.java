package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;

public final class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private Drivetrain m_drivetrain;
  private CommandXboxController m_driverController = new CommandXboxController(0);

  public Robot() {
    Logger logger = Logger.getInstance();
    recordBuildMetadata(logger);
    switch (Constants.kCurrentMode) {
      case kReal:
        logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        logger.addDataReceiver(new NT4Publisher());
        LoggedPowerDistribution.getInstance(99, ModuleType.kRev);
        break;
      case kSim:
        logger.addDataReceiver(new WPILOGWriter(""));
        logger.addDataReceiver(new NT4Publisher());
        break;
      case kReplay:
        setUseTiming(false);
        final String logPath = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(logPath));
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
      default:
        throw new RuntimeException("Invalid Robot Mode");
    }
    logger.start();
    m_drivetrain = new Drivetrain();
    m_drivetrain.setJoystickAxes(
        m_driverController::getLeftX,
        m_driverController::getLeftY,
        m_driverController::getRightX);
    configureButtonBindings();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  private void configureButtonBindings() {

  }

  private void recordBuildMetadata(Logger logger) {
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
  }

}
