package org.firstinspires.ftc.teamcode.whiteout.RobotCarousel.RC;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.whiteout.Common.tataAutonomousBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;


public class RobotCarouselHW {
    public DcMotorEx S0 = null;

    public CRServo CRS0    = null;
    public CRServo CRS1    = null;
    public tataAutonomousBase.SideColor currSide;

    private static final double CAROUSEL_DIAM_INCH = 15;
    private static final double WHEEL_DIAM_INCH = 2;

    private static final double TICK_COUNTS_PER_INCH = (13.7 * 28) / (4 * 3.1415);


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, tataAutonomousBase.SideColor sc) {
        S0  = ahwMap.get(DcMotorEx.class, "S0");
        currSide = sc;

        if (currSide == tataAutonomousBase.SideColor.Blue) {
            S0.setDirection(DcMotorSimple.Direction.REVERSE);
        }

    }

    public void rotateCarousel(double numTurns, double speed) {
        int newPos = 0;

        S0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Check of the motor is not already running
        if (S0.isBusy()) {
            RobotLog.ii("L124", "Motor is Busy");
            S0.setPower(0);
        }

        double wheelTurns = (CAROUSEL_DIAM_INCH*numTurns)/WHEEL_DIAM_INCH;
        double totalInch =  (wheelTurns*3.1415*WHEEL_DIAM_INCH);

        newPos = S0.getCurrentPosition() + (int) (totalInch *TICK_COUNTS_PER_INCH);

        //Ensure newPos is always >= 0
        if (newPos < 0) {
            RobotLog.ii("L124", "Motor New Position negative. Setting to zero");
            newPos = 0;
        }

        S0.setTargetPosition(newPos);
        S0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //S0.setPower(Math.abs(speed));
        S0.setVelocity(1400);
        RobotLog.ii("L124", "Motors run to positoin %2d", newPos);
    }


    public void reverseDir() {
        if (currSide == tataAutonomousBase.SideColor.Blue) {
            currSide = tataAutonomousBase.SideColor.Red;
        } else {
            currSide = tataAutonomousBase.SideColor.Blue;
        }

    }

    public void setPower(double power){
        S0.setPower(power);
    }

    public double getPower(){
        return S0.getPower();
    }

    public void stopCarousel() {
        setPower(0.0);
    }
    public void startCarousel(double power) {
        setPower(power);
    }


}

