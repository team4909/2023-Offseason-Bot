package frc.lib;

import java.util.function.Consumer;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class LoggedDouble implements Consumer<Double> {

  private DoubleTopic m_topic;
  private DoublePublisher m_publisher;
  private DoubleLogEntry m_logEntry;

  public LoggedDouble(String subTable, String name) {
    m_topic = Logger.getInstance().logTable.getSubTable(subTable).getDoubleTopic(name);
    m_publisher = m_topic.publish();
    m_logEntry = new DoubleLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
  }

  public void accept(Double value) {
    if (Logger.getInstance().useNT.getAsBoolean())
      m_publisher.accept(value);
    else
      m_logEntry.append(value);

  }
}
