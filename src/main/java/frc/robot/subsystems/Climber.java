/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private Solenoid engagePTO;
    private Solenoid extend;

    public Climber() {
        engagePTO = new Solenoid(Constants.Climber.ENGAGE_PTO);
        extend = new Solenoid(Constants.Climber.EXTEND_CLIMBER);
    }

    private boolean checkGameTime(){
        if (DriverStation.getInstance().getMatchTime() < 40
            && DriverStation.getInstance().isOperatorControlEnabled()) return true;
        else if (!DriverStation.getInstance().isFMSAttached()) return true;
        else return false;
    }

    public void EngagePTO(){
        engagePTO.set(true);
    }

    public void DisengagePTO(){
        engagePTO.set(false);
    }

    public void ExtendClimber(){
        extend.set(true);
    }

    public void RetractClimber(){
        extend.set(false);
    }

    public boolean isPTOEngaged(){
        return engagePTO.get();
    }

    public boolean isClimberExtended(){
        return extend.get();
    }
}
