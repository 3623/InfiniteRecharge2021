/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
* An example subsystem. You can replace me with your own Subsystem.
*/
public class Spindexer extends TerribleSubsystem {
    public static final double SHOOT_TIME = 3.5;
    public static final double INDEX_TIME = 5.0;
    private static final double READY_SPEED = 0.2;
    private static final double INDEX_SPEED = 0.4;
    private static final double SHOOT_SPEED = 0.45;
    private WPI_VictorSPX spindexerSPX;
    private static final double TICKS_PER_ENCODER_REV = 2048.0;
    private static final double DISTANCE_PER_PULSE = 1.0 / TICKS_PER_ENCODER_REV // Converts to Output Revolution Count
                                                        * (1.0 / (400/24)) // Converts to Whole Spindexer Revolution Count
                                                        * 5; // Converts to Spindexer Sections

    private enum MODE{
        STOPPED,
        INDEX,
        JAM_CLEAR,
        READYING,
        SHOOTING
    }

    private MODE spinMode = MODE.STOPPED;
    private MODE lastMode = MODE.STOPPED;

    //private DigitalInput spindexerInPosition; //, ballSensor2, ballSensor3, ballSensor4,
    // ballSensor5;
    private Encoder spinCoder;

    private double jamPosition = 0.0;
    private boolean jamFlipper = false;
    private int flipDebounceCounter = 0;

    public Spindexer() {
        spindexerSPX = new WPI_VictorSPX(Constants.Shooter.SPINDEXER_MOTOR_SPX);
        spindexerSPX.setInverted(true);
        spinCoder = new Encoder(0, 1);
        spinCoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    }

    public double getPosition(){
        return spinCoder.getDistance();
    }

    public int getMode(){
        switch (spinMode){
            case STOPPED:
            return 0;
            case INDEX:
            return 1;
            case READYING:
            return 2;
            case JAM_CLEAR:
            return 4;
            case SHOOTING:
            return 3;
        }
        return -1;
    }

    public boolean clearingJam(){
        if (getMode() == 4) return true;
        else return false;
    }


    public boolean isReady(){
        boolean readyState = false;
        int wholeDistance = (int)spinCoder.getDistance();
        double tenthDistance = spinCoder.getDistance()-wholeDistance;
        if (tenthDistance < 0.1 || tenthDistance > 0.9) readyState = true;
        return readyState;
    }

    private void setOutput() {
        switch (spinMode){
            case STOPPED:
                setSpinning(0.0);
                break;
            case INDEX:
                setSpinning(INDEX_SPEED);
                break;
            case READYING:
                if (!isReady()) setSpinning(READY_SPEED);
                else {
                    spinMode = MODE.STOPPED;
                    setSpinning(0.0);
                }
                break;
            case JAM_CLEAR:
                clearJam();
                break;
            case SHOOTING:
                setSpinning(SHOOT_SPEED);
                break;
        }
    }

    public void startIndex(){
        spinMode = MODE.INDEX;
    }

    public void startReadying(){
        spinMode = MODE.READYING;
    }

    public void startShooting(){
        spinMode = MODE.SHOOTING;
    }

    public void startJamClear(){
        lastMode = spinMode;
        spinMode = MODE.JAM_CLEAR;
        jamPosition = getPosition();
    }

    public void clearedJam(){
        spinMode = lastMode;
    }

    private void clearJam(){
        double output = 0.35;
        boolean currentFlip = jamFlipper;
        if (Math.abs(getPosition()-jamPosition) > 0.25 && flipDebounceCounter == 0) jamFlipper = jamFlipper^true;
        if (flipDebounceCounter > 0) flipDebounceCounter += 1;
        if (flipDebounceCounter > 25) flipDebounceCounter = 0;
        if (jamFlipper != currentFlip) flipDebounceCounter = 1;
        if (jamFlipper) output *= -1;
        setSpinning(output);
    }

    private void setSpinning(double speed) {
        spindexerSPX.set(ControlMode.PercentOutput, speed);
    }

    public void stopSpinning(){
        spinMode = MODE.STOPPED;
    }

    public void monitor(){
        SmartDashboard.putString("Spindexer/Spindexer Mode", spinMode.toString());
        SmartDashboard.putNumber("Spindexer/Spindexer Position", getPosition());
        SmartDashboard.putNumber("Spindexer/Spindexer Slot", Math.round((getPosition() % 5)+1));
        SmartDashboard.putNumber("Spindexer/Output", spindexerSPX.getMotorOutputPercent());
        SmartDashboard.putNumber("Spindexer/Int Cast", (int)getPosition());
    }
    

    public void periodic(){
        setOutput();
    }
}
