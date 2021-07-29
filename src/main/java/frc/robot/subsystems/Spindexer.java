/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.util.Utils;
import frc.util.Utils.MovingAverage;

/**
* An example subsystem. You can replace me with your own Subsystem.
*/
public class Spindexer extends TerribleSubsystem {

    public static final double SHOOT_TIME = 3.0;
    public static final double INDEX_TIME = 5.0;

    private static final double READY_SPEED = 0.6;
    private static final double SHOOT_SPEED = 0.5;

    private static final double INDEX_BIAS = 0.3;
    private static final double INDEX_AMP = 0.4;
    private static final double INDEX_PER = 1.5;

    private static final double STANDBY_BIAS = 0.3;
    private static final double STANDBY_AMP = 0.5;
    private static final double STANDBY_PER = 1.7;

    private static final double TICKS_PER_ENCODER_REV = 2048.0;
    private static final double DIST_P_PULSE = (24.0 / 400.0) / TICKS_PER_ENCODER_REV * 5.0; // Converts to Spindexer Sections

    private WPI_VictorSPX spindexerSPX;

    private enum MODE{
        STOPPED,
        INDEX,
        STANDBY,
        JAM_CLEAR,
        READYING,
        SHOOTING,
    }

    private MODE spinMode = MODE.STOPPED;
    private MODE lastMode = MODE.STOPPED;

    private Encoder spinCoder;

    private MovingAverage jamCounter;
    private static final double MIN_UNJAMMED_SPEED = 1.0;
    private static final double AVG_JAM_THRESHOLD = 0.5;

    public Spindexer() {
        setName("Spindexer");
        spindexerSPX = new WPI_VictorSPX(Constants.Shooter.SPINDEXER_MOTOR_SPX);
        spindexerSPX.setInverted(true);
        spinCoder = new Encoder(0, 1);
        spinCoder.setDistancePerPulse(DIST_P_PULSE);
        jamCounter = new MovingAverage(100);
    }

    private double getPosition(){
        return spinCoder.getDistance();
    }

    private boolean isJammed() {
        double speed = spinCoder.getRate();
        boolean jammed = Utils.outsideDeadband(speed, 0.0, MIN_UNJAMMED_SPEED);
        return jammed && spinMode != MODE.STOPPED;
    }

    private void jamChecker() {
        double avg;
        if (isJammed()) avg = jamCounter.update(1.0);
        else avg = jamCounter.update(0.0);

        if (avg > AVG_JAM_THRESHOLD && spinMode != MODE.JAM_CLEAR)
            CommandScheduler.getInstance().schedule(new InstantCommand(() -> this.toggleJamClear(), this)
                    .withTimeout(2.0));
    }


    public boolean isReady(){
        double moddedDelta = spinCoder.getDistance() % 1.0;
        // TODO might have to tune this
        boolean readyState = moddedDelta < 0.1 || moddedDelta > 0.9;
        readyState &= spinMode == MODE.READYING;
        return readyState;
    }

    private void setOutput() {
        switch (spinMode){
            case STOPPED:
                setSpinning(0.0);
                break;
            case INDEX:
                spinBackAndForth(INDEX_BIAS, INDEX_AMP, INDEX_PER);
                break;
            case READYING:
                setSpinning(READY_SPEED);
                break;
            case STANDBY:
                spinBackAndForth(STANDBY_BIAS, STANDBY_AMP, STANDBY_PER);
                break;
            case JAM_CLEAR:
                spinBackAndForth(0.1, 0.8, 1.5);
                break;
            case SHOOTING:
                setSpinning(SHOOT_SPEED);
                break;
        }
        if (spinMode != MODE.STOPPED) jamChecker();
    }

    public void startIndex(){
        if (spinMode == MODE.STANDBY || spinMode == MODE.STOPPED)
            spinMode = MODE.INDEX;
    }

    public void setStandby() {
        spinMode = MODE.STANDBY;
    }

    public void startReadying(){
        spinMode = MODE.READYING;
    }

    public void startShooting(){
        spinMode = MODE.SHOOTING;
    }

    public void toggleJamClear(){
        if (spinMode == MODE.JAM_CLEAR)
            spinMode = lastMode;
        else {
            lastMode = spinMode;
            spinMode = MODE.JAM_CLEAR;
        }
    }

    private void setSpinning(double speed) {
        spindexerSPX.set(ControlMode.PercentOutput, -speed);
    }

    private void spinBackAndForth(double bias, double amplitude, double period) {
        setSpinning(Math.sin(Timer.getFPGATimestamp()*6.28/period) * amplitude + bias);
    }

    public void stopSpinning(){
        spinMode = MODE.STOPPED;
    }

    public void monitor(){
        SmartDashboard.putString("Spindexer/Spindexer Mode", spinMode.toString());
        display("Position", getPosition());
        display("Slot", Math.round((getPosition() % 5)+1));
        display("Output", spindexerSPX.getMotorOutputPercent());
        display("Int Cast", (int)getPosition());
    }

    public void periodic(){
        setOutput();
    }
}
