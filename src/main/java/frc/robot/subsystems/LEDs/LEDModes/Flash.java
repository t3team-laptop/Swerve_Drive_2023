package frc.robot.subsystems.LEDs.LEDModes;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.lib.util.HSV;
import frc.robot.Constants.LEDs;

public class Flash extends LEDModeBase {
    private double brightness = 0;
    private int multiplier = 1;

    private HSV hsv;

    public Flash(AddressableLEDBuffer m_ledBuffer, HSV hsv) {
        super(m_ledBuffer);

        this.hsv = hsv;
    }

    public void execute() {
        brightness = MathUtil.clamp(brightness + LEDs.Flash.speed * multiplier, 0, hsv.v);
        if (brightness == hsv.v || brightness == 0) {
            multiplier *= -1;
        }

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, hsv.h, hsv.s, (int) brightness);
        }
    }
}