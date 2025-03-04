package frc.robot.subsystem.led;

import static frc.robot.RobotMap.CANDLE_ID;

import java.util.function.Supplier;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.Util;

public class LedSubsystem extends SubsystemBase {
    
    private final CANdle m_candle = new CANdle(CANDLE_ID);
    
    public record PercentageSetpoint(double pct, LedState color) {
    }

    public LedSubsystem() {
        m_candle.configBrightnessScalar(1.0);
        
    }

    public void writePixels(LedState state) {
        if (state == null)
            state = LedState.kOff;
        m_candle.setLEDs(state.red, state.green, state.blue);
    }

    
    public void writePixels(LedState[] pixels) {
        // do not write empty data
        if (pixels == null || pixels.length == 0) {
            return;
        }

        LedState run = pixels[0];
        int runStart = 0;
        for (int i = 0; i < pixels.length; i++) {
            if (pixels[i] == null)
                pixels[i] = LedState.kOff;
            if (!run.equals(pixels[i])) {
                m_candle.setLEDs(run.red, run.green, run.blue, 255, runStart, i - runStart);
                runStart = i;
                run = pixels[i];
            }
        }

        m_candle.setLEDs(run.red, run.green, run.blue, 255, runStart, pixels.length - runStart);
    } 

    public Command commandSolidColor(LedState state) {
        return run(() -> setSolidColor(state)).ignoringDisable(true).withName("LED Solid Color");
    }

    public Command commandSolidColor(Supplier<LedState> state) {
        return run(() -> setSolidColor(state.get())).ignoringDisable(true)
                .withName("LED Solid Color");
    }

    public Command commandSolidPattern(LedState[] states) {
        return run(() -> setSolidPattern(states)).ignoringDisable(true).withName("LED Solid Pattern");
    }

    public Command commandPercentageFull(double percentageFull, LedState state) {
        return run(() -> setPercentageFull(percentageFull, state)).ignoringDisable(true);
    }

    public Command commandPercentageFull(Supplier<PercentageSetpoint> percentageSupplier) {
        return run(() -> setPercentageFull(percentageSupplier.get().pct, percentageSupplier.get().color))
                .ignoringDisable(true);
    }

    public Command commandBlinkingState(LedState stateOne, LedState stateTwo, double durationOne, double durationTwo) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> setSolidColor(stateOne)),
                new WaitCommand(durationOne),
                Commands.runOnce(() -> setSolidColor(stateTwo)),
                new WaitCommand(durationTwo)).repeatedly().ignoringDisable(true).withName("Blinking LED command");
    }

    public Command commandBlinkingStateWithoutScheduler(LedState stateOne, LedState stateTwo, double durationOne,
            double durationTwo) {
        var state = new Object() {
            public boolean color1 = true;
            public double timestamp = Timer.getFPGATimestamp();
        };
        return Commands.runOnce(() -> {
            state.color1 = true;
            state.timestamp = Timer.getFPGATimestamp();
        }).andThen(commandSolidColor(() -> {
            if (state.color1 && state.timestamp + durationOne <= Timer.getFPGATimestamp()) {
                state.color1 = false;
                state.timestamp = Timer.getFPGATimestamp();
            } else if (!state.color1 && state.timestamp + durationTwo <= Timer.getFPGATimestamp()) {
                state.color1 = true;
                state.timestamp = Timer.getFPGATimestamp();
            }

            if (state.color1) {
                return stateOne;
            } else {
                return stateTwo;
            }
        })).ignoringDisable(true).withName("Blinking LED command");
    }

    public Command commandBlinkingState(LedState stateOne, LedState stateTwo, double duration) {
        return commandBlinkingState(stateOne, stateTwo, duration, duration).ignoringDisable(true);
    }

    private void setSolidColor(LedState state) {
        writePixels(state);
    }

    private void setSolidPattern(LedState[] states) {
        writePixels(states);
    }

    private void setPercentageFull(double percentageFull, LedState state) {
        LedState[] pixels = new LedState[21/ 2];
        for (int i = 0; i < pixels.length; i++) {
            if (i < pixels.length * Util.limit(percentageFull, 0.0, 1.0)) {
                pixels[i] = state;
            }
        }

        //io.writePixels(mirror(pixels));
    }

    
}
