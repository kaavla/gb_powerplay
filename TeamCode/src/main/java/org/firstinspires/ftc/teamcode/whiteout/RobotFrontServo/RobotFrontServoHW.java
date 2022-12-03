package org.firstinspires.ftc.teamcode.whiteout.RobotFrontServo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class RobotFrontServoHW {

    public Servo FRS1   = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        FRS1      = ahwMap.get(Servo.class, "FRS1");

    }

    public void setPos(double pos){
        FRS1.setPosition(pos);
    }

    public double getPos(){
        return FRS1.getPosition();
    }

    public void resetFrontServo() {
        setPos(0.0);
    }
    public void turnFrontServo(double pos) {
        setPos(pos);
    }


}

