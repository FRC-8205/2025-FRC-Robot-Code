package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;


public class CustomKeyboard extends CommandGenericHID{
    private final Joystick keyboard;

    public CustomKeyboard(int port) {
        super(port);
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
