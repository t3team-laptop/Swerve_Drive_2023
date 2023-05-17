package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.HSV;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.LEDs.LEDMode;
import frc.robot.subsystems.LEDs.LEDModes.Flash;


public class LEDs extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private LEDMode mode;

    private final Flash whiteDot;
    private final Flash purpleFlash;
    private final Flash yellowFlash;

    public LEDs() {
        this.mode = Constants.LEDs.defaultMode;
        this.m_led = new AddressableLED(Constants.LEDs.id);
        this.m_ledBuffer = new AddressableLEDBuffer(Constants.LEDs.length);

        this.whiteDot = new Flash(m_ledBuffer, HSV.googleColorPickerHSV(44, 0, 100));
        this.purpleFlash = new Flash(m_ledBuffer, HSV.googleColorPickerHSV(263, 73, 96));
        this.yellowFlash = new Flash(m_ledBuffer, HSV.googleColorPickerHSV(35, 100, 100));

        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @Override
    public void periodic() {
        switch (mode) {
            case WHITEDOT:
                whiteDot.execute();
                break;
            case PURPLEFLASH:
                purpleFlash.execute();
                break;
            case YELLOWFLASH:
                yellowFlash.execute();
                break;
        }
        m_led.setData(m_ledBuffer);
        SmartDashboard.putString("LED Mode", mode.toString());
    }

    public void setLEDMode(LEDMode mode) {
        this.mode = mode;
    }

    public void setHPSignal(GamePiece hpSignal){
      switch(hpSignal){
        case CONE:
          hpSignal = GamePiece.CONE;
          setLEDMode(LEDMode.YELLOWFLASH);
          break;
        case CUBE:
          hpSignal = GamePiece.CUBE;
          setLEDMode(LEDMode.PURPLEFLASH);
          break;
      }
    }
}