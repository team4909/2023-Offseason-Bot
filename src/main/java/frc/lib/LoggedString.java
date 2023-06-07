package frc.lib;

import java.util.function.Consumer;

import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class LoggedString implements Consumer<String> {

  private StringTopic m_topic;
  private StringPublisher m_publisher;
  private StringLogEntry m_logEntry;
  private StringSubscriber m_currentValue; // Used online
  private String m_lastValueFromDataLog; // Used offline

  public LoggedString(String subTable, String name) {
    m_topic = Logger.getInstance().logTable.getSubTable(subTable).getStringTopic(name);
    m_publisher = m_topic.publish();
    m_currentValue = m_topic.subscribe("");
    m_logEntry = new StringLogEntry(DataLogManager.getLog(), "Robot/" + subTable + "/" + name);
  }

  public void accept(String value) {
    if (Logger.getInstance().useNT.getAsBoolean())
      m_publisher.accept(value);
    else {
      m_lastValueFromDataLog = value;
      m_logEntry.append(value);
    }
  }

  public void append(String value) {
    if (Logger.getInstance().useNT.getAsBoolean())
      m_publisher.accept(m_currentValue.get().concat(value));
    else
      m_logEntry.append(m_lastValueFromDataLog += value);
  }
}
