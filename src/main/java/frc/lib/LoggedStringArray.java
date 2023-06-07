package frc.lib;

import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArrayTopic;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class LoggedStringArray {

  private StringArrayTopic m_topic;
  private StringArrayPublisher m_publisher;
  private StringArrayLogEntry m_logEntry;

  public LoggedStringArray(String subTable, String name) {
    m_topic = Logger.getInstance().logTable.getSubTable(subTable).getStringArrayTopic(name);
    m_publisher = m_topic.publish();
    m_logEntry = new StringArrayLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
  }

  public void accept(String[] value) {
    if (Logger.getInstance().useNT.getAsBoolean())
      m_publisher.accept(value);
    else
      m_logEntry.append(value);

  }
}
