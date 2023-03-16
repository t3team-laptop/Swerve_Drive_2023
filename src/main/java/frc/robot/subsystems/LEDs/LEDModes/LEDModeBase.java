package frc.robot.subsystems.LEDs.LEDModes;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public abstract class LEDModeBase {
    protected AddressableLEDBuffer m_ledBuffer;

    protected LEDModeBase(AddressableLEDBuffer m_ledBuffer) {
        this.m_ledBuffer = m_ledBuffer;
    }

    public abstract void execute();
}