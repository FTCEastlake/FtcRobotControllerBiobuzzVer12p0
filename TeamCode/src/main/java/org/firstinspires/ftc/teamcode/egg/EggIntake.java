package org.firstinspires.ftc.teamcode.egg;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EggIntake {

    private DcMotorEx _motorExIntake;

    public EggIntake (HardwareMap hardwareMap) {
        _motorExIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");
        _motorExIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        // This uses RUN_USING_ENCODER to be more accurate.
        _motorExIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorExIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setVelocity(double ticksPerSecond) {
        _motorExIntake.setVelocity(ticksPerSecond);
    }

    public void setOn() {
        // Looks like 3000 ticks/sec is the fastest it will spin.
        // We can lower the velocity to save power.
        _motorExIntake.setVelocity(3000);
    }

    public void setOff() {
        _motorExIntake.setVelocity(0);
    }

    public void reverse() {_motorExIntake.setVelocity(-1000);}

}
