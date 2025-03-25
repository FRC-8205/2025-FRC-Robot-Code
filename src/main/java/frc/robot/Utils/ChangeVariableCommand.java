package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ChangeVariableCommand<T> extends InstantCommand {
    private T variable;
    private T[] options;

    public ChangeVariableCommand(T variable, T[] options) {
        this.variable = variable;
        this.options = options;
    }

    @Override
    public void initialize() {
        int currentIdx = -1;
        for (int i = 0; i < options.length; i++) {
            if (options[i].equals(variable)) {
                currentIdx = i;
            }
        }

        if (currentIdx >= 0) {
            try {
                variable = options[currentIdx + 1];
            } catch (IndexOutOfBoundsException e) {
                variable = options[0];
            }
        }
    }
}
