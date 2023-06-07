package frc.lib;

import java.util.function.Consumer;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class LoggedBoolean implements Consumer<Boolean> {

  private BooleanTopic m_topic;
  private BooleanPublisher m_publisher;
  private BooleanLogEntry m_logEntry;

  public LoggedBoolean(String subTable, String name) {
    m_topic = Logger.getInstance().logTable.getSubTable(subTable).getBooleanTopic(name);
    m_publisher = m_topic.publish();
    m_logEntry = new BooleanLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
  }

  public void accept(Boolean value) {
    if (Logger.getInstance().useNT.getAsBoolean())
      m_publisher.accept(value);
    else
      m_logEntry.append(value);

  }
}
