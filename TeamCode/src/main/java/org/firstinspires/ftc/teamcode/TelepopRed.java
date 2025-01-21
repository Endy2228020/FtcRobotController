package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;
import java.util.Timer;

/*

 */

@TeleOp(name="TelepopRed", group="INFINITY")
//@Disabled
public class TelepopRed extends LinearOpMode implements Inter{

    //Железо
    private DcMotor m1, m2, m3, m4, m5;
    private Servo s5;
    //private DigitalChannel touch;

    //Переменные моторов
    private double test=0;
    private double zm1, zm2, zm3, zm4, zm5;
    private double encoder1,encoder2,encoder3,encoder4,averageEncoder;
    private double last_moment_serv = 0.0, last_moment_switch = 0.0, last_moment_free = 0.0;
    private double moment_diff_serv, moment_diff_switch, moment_diff_free;
    private boolean auto_mode = true, free_mode = false;
    private double a, turn;
    int telescopePos = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private double zs5=OPEN;
    private double lamp=0;
    private int height;
    File telescopeFile = AppUtil.getInstance().getSettingsFile("telescopeFile.txt"); //Файл с позицией телескопа
    private int svob=0;

    //Инициализируем железо
    public void initC() {
        //Инициализация
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");
        m4 = hardwareMap.get(DcMotor.class, "m4");
        m5 = hardwareMap.get(DcMotor.class, "teleskop");

        s5 = hardwareMap.get(Servo.class, "20kg");


        s5.setPosition(OPEN);
        //ВЕРНУТЬСЯ / НЕ ЗАБЫТЬ!!!!
        // En1 = hardwareMap.get(DcMotor.class, "En1");
        // En2 = hardwareMap.get(DcMotor.class, "En2");
        // En3 = hardwareMap.get(DcMotor.class, "En3");

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //ВЕРНУТЬСЯ / НЕ ЗАБЫТЬ!!!!
        //En1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // En2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //En3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //ВЕРНУТЬСЯ / НЕ ЗАБЫТЬ!!!!
        // En1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //En2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // En3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //ВЕРНУТЬСЯ / НЕ ЗАБЫТЬ!!!!
        //  En1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // En2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //En3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void runOpMode() {

        class CalcThread implements Runnable {
            private Thread c;
            private boolean running;

            public void run() {
                telemetry.addLine("Calc thread running");
                telemetry.update();

                try {
                    m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m5.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    //ВЕРНУТЬСЯ / НЕ ЗАБЫТЬ!!!!
                    // En1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    // En2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    // En3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    while (!isStopRequested() & opModeIsActive()) {


                        //ТЕЛЕЖКА


                        //Коэффицент скорости робота
                        if(gamepad1.left_trigger<0.5){
                            a=0.8/1;
                        }
                        else if(gamepad1.left_trigger>0.5){
                            a=10/1;
                        }
//                        if(gamepad2.right_stick_y <0.5){
//                            m5.setPower(gamepad2.right_stick_y);
//                        }

                        //Поворот
                        turn = gamepad1.right_stick_x;

                        //Мощность моторов
                        zm1 = -1*(Range.clip((gamepad1.left_stick_x + gamepad1.left_stick_y - -1*turn ) * a, -1, 1));
                        if (zm1 > -0.05 && zm1 < 0.05) {
                            zm1 = 0;
                        }

                        zm2 = Range.clip((gamepad1.left_stick_x - gamepad1.left_stick_y - turn ) * a, -1, 1);
                        if (zm2 > -0.05 && zm2 < 0.05) {
                            zm2 = 0;
                        }

                        zm3 = -1*(Range.clip((-gamepad1.left_stick_x - gamepad1.left_stick_y - -1*turn ) * a, -1, 1));
                        if (zm3 > -0.05 && zm3 < 0.05) {
                            zm3 = 0;
                        }

                        zm4 = Range.clip((-gamepad1.left_stick_x + gamepad1.left_stick_y - turn ) * a, -1, 1);
                        if (zm4 > -0.05 && zm4 < 0.05) {
                            zm4 = 0;
                        }



                        //ТЕЛЕСКОП
                        if(gamepad1.left_bumper==true){

                            if(gamepad1.left_trigger > 0.08){
                                m5.setPower(gamepad1.left_trigger*100);
                            }

                            if(gamepad1.right_trigger>0.08){
                                m5.setPower(gamepad1.right_trigger*-100);
                            }

                        }


                        else {
                            if(gamepad1.left_trigger > 0.08){
                                m5.setPower(gamepad1.left_trigger*1);
                            }

                            if(gamepad1.right_trigger>0.08){
                                m5.setPower(gamepad1.right_trigger*-1);
                            }


                        }

                        //Захват "прямоугольничков"





                        moment_diff_serv = runtime.milliseconds() - last_moment_serv;
                        moment_diff_switch = runtime.milliseconds() - last_moment_switch;

                     if(gamepad1.a==true && moment_diff_serv > 200) {
                         if (zs5 == CLOSE) {
                             zs5 = OPEN;
                             lamp = 0;
                             last_moment_serv = runtime.milliseconds();
                         } else {
                             zs5 = CLOSE;
                             lamp = -0.1;
                             last_moment_serv = runtime.milliseconds();
                         }
                     }
                    }

                } catch (Exception e) {
                    telemetry.addLine("Calc thread interrupted");
                    telemetry.update();
                }
            }
            public void start_c() {
                if (c == null) {
                    c = new Thread(this, "Calc thread");
                    c.start();
                }
            }
        }

        //Инициализация
        initC();

        waitForStart();

        //Запуск подпроцессов
        CalcThread C1 = new CalcThread();
        C1.start_c();

        //ОСНОВНАЯ ПРОГРАММА

        while(opModeIsActive() & !isStopRequested()) {

            m1.setPower(zm1);//слева спереди
            m2.setPower(zm2);//справа спереди
            m3.setPower(zm3);//слева сзади
            m4.setPower(zm4);//справа сздади
            m5.setPower(zm5);//барабан

            // Считываем значения энкодеров
            encoder1 = m1.getCurrentPosition();
            encoder2 = m2.getCurrentPosition();
            encoder3 = m3.getCurrentPosition();
            encoder4 = m4.getCurrentPosition();

            double averageEncoder = (encoder1 + encoder2 + encoder3 + encoder4) / 4.0;
           //En3.setPower(lamp);

            s5.setPosition(zs5);

            telemetry.addData("Состояние тригера", gamepad1.left_trigger);
            telemetry.addData("коэфицент скорости", a);
            telemetry.addData("Положение серво", height);
            telemetry.addData("Конус", zs5);
            telemetry.addData("Svod", svob);
            telemetry.addData("Мотор1", zm1);
            telemetry.addData("Мотор2", zm2);
            telemetry.addData("Мотор3", zm3);
            telemetry.addData("Мотор4", zm4);
            telemetry.addData("Мотор5", zm5);
            telemetry.addData("Стик1 X", gamepad1.left_stick_x);
            telemetry.addData("Стик1 Y", gamepad1.left_stick_y);
            telemetry.addData("Стик2 X", gamepad2.right_stick_x);
            telemetry.addData("Стик2 Y", gamepad2.right_stick_y);
            telemetry.addData("Уровень по энкодеру", m5.getCurrentPosition());
            telemetry.addData("Уровень", height);
            // Выводим значения в телеметрию
            telemetry.addData("Энкодер 1", encoder1);
            telemetry.addData("Энкодер 2", encoder2);
            telemetry.addData("Энкодер 3", encoder3);
            telemetry.addData("Энкодер 4", encoder4);
            telemetry.addData("Среднее значение энкодеров", averageEncoder);
            telemetry.addData("Ускорение", a);

            telemetry.update();

        };
        ReadWriteFile.writeFile(telescopeFile, Integer.toString(telescopePos));
    }
}