package org.firstinspires.ftc.teamcode.egg;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EggFlyWheel {

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private DcMotorEx _motorExFlywheel;

    public EggFlyWheel (LinearOpMode opMode) throws InterruptedException {
        _opMode = opMode;
        _hardwareMap = _opMode.hardwareMap;

        _motorExFlywheel = _hardwareMap.get(DcMotorEx.class, "motorFlywheel");
        _motorExFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        // This uses RUN_USING_ENCODER to be more accurate.
        _motorExFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorExFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setVelocity(double ticksPerSecond) {
        _motorExFlywheel.setVelocity(ticksPerSecond);
    }

    public void setOn() {
        // TODO: test out velocity.
        // We can lower the velocity to save power.
        _motorExFlywheel.setVelocity(3000);
    }

    public void setOff() {
        _motorExFlywheel.setVelocity(0);
    }
    public double getSpeed(){
        return _motorExFlywheel.getVelocity();
    }
}
