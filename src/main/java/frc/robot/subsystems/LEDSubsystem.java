package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    AddressableLED m_led8 = new AddressableLED(8);
    AddressableLED m_led9 = new AddressableLED(9);// ADD PORT
    AddressableLEDBuffer m_ledBuffer8 = new AddressableLEDBuffer(144);
    AddressableLEDBuffer m_ledBuffer9 = new AddressableLEDBuffer(144);

    public LEDSubsystem(){
        m_led8.setLength(m_ledBuffer8.getLength());
        m_led9.setLength(m_ledBuffer9.getLength());
        m_led8.setData(m_ledBuffer8);
        m_led9.setData(m_ledBuffer9);
        m_led8.start();
        m_led9.start();


    }

    public void cargoLed(int r, int g, int b)
    {
        for (var i = 65; i < m_ledBuffer9.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
            m_ledBuffer8.setRGB(i, r, g, b);
            m_ledBuffer9.setRGB(i, r, g, b);
        }
        m_led8.setData(m_ledBuffer8);
        m_led9.setData(m_ledBuffer9);

    }
    
    public void aprilTagLed(int r, int g, int b){
        for (var i = 0; i < 65; i++) {
      // Sets the specified LED to the RGB values for red
            m_ledBuffer8.setRGB(i, r, g, b);
            m_ledBuffer9.setRGB(i, r, g, b);
        }
        m_led8.setData(m_ledBuffer8);
        m_led9.setData(m_ledBuffer9);
    }

    
    
    // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    //   // Sets the specified LED to the RGB values for red
    //   m_ledBuffer.setRGB(i, 0, 255, 0);
    // }
    // m_led.close();
   
    // m_led.setData(m_ledBuffer);
    @Override
    public void periodic() {
        
    }
}
