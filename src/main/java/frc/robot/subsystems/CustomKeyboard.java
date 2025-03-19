package frc.robot.subsystems;

import java.nio.channels.NonWritableChannelException;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class CustomKeyboard {
    private final Joystick keyboard;

    public CustomKeyboard(int port) {
        keyboard = new Joystick(port);
    }

    public Trigger moveLevel1() {
        return new Trigger(() -> keyboard.getRawButton(1));
    }

    public Trigger moveLevel2() {
        return new Trigger(() -> keyboard.getRawButton(2));
    }

    public Trigger moveLevel3() {
        return new Trigger(() -> keyboard.getRawButton(3));
    }

    public Trigger moveLevel4() {
        return new Trigger(() -> keyboard.getRawButton(4));
    }
}
