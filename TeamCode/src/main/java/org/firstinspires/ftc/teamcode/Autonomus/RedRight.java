package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red2", group="Auto", preselectTeleOp = "TeleOP")
public class RedRight extends LinearOpMode {
    public AutoMethodsTime bot = new AutoMethodsTime();

    //ОСНОВНАЯ ПРОГРАММА
    @Override

    public void runOpMode() throws InterruptedException {
        bot.initC(this);
        waitForStart();
    while (opModeIsActive() && !isStopRequested()){
    bot.right(3,1);
    }

    }
}
