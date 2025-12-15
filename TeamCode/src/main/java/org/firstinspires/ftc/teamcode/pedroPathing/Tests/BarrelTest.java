package org.firstinspires.ftc.teamcode.pedroPathing.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class BarrelTest extends OpMode {
    TestColorSensorMecanism mecanism = new TestColorSensorMecanism();
    TestColorSensorMecanism.DetectedColor detectedColor;
//Tl: POSICIONES☺
    private static final double  Ain = 0.38;
    private static final double  Bin = 0.46;
    private static final double  Cin = 0.535;
    private static final double  Aout = 0.5;
    private static final double  Bout = 0.57;
    private static final double  Cout = 0.65;
    char actualPos = 'a';
//NOTE: 0 = empty || 1 = PURPLE || 2 = GREEN
    int A = 0;
    int B = 0;
    int C = 0;
//TL: MODES
    private boolean PPG = false;
    private boolean PGP = false;
    private boolean GPP = false;

    private boolean isShooting = false;
    private int shootStep = 0;
    private long shootStartTime = 0;

    private final long OUTTAKE_HOLD_TIME_MS = 3000;
    private long lastIntakeTime = 0;
    private static final long INTAKE_COOLDOWN_MS = 800;

    @Override
    public void init() {
        mecanism.init(hardwareMap);
        mecanism.barril.setPosition(Ain);
    }

    @Override
    public void loop() {
//TL ---------- MODE SELECT ----------
        if (gamepad2.dpad_right) {
            PPG = true;
            PGP = false;
            GPP = false;
        }
        if (gamepad2.dpad_up) {
            PGP = true;
            PPG = false;
            GPP = false;
        }
        if (gamepad2.dpad_left) {
            GPP = true;
            PPG = false;
            PGP = false;
        }
        if (gamepad2.dpad_down) {
            PPG = PGP = GPP = false;
        }

//TL  ---------- EMPTY -> G28 ----------
        if (A == 0 && B == 0 && C == 0) {
            mecanism.barril.setPosition(Ain);
            actualPos = 'a';
            isShooting = false;
        }
        if (A != 0 && B != 0 && C != 0){
            mecanism.barril.setPosition(Aout);
            actualPos = 'a';
        }

//TL ---------- CANNON / AUTOMATIC ----------

        if (gamepad2.right_trigger > 0.1f && !isShooting && (PPG || PGP || GPP)) {
            isShooting = true;
            shootStep = 0;
            shootStartTime = System.currentTimeMillis();

        }

        if (isShooting) {
            String sequence = PPG ? "PPG" : PGP ? "PGP" : "GPP";

            int neededValue = (sequence.charAt(shootStep) == 'P') ? 1 : 2;
            telemetry.addData("VALUE: ", neededValue);

            char chamber = '\0';
            //note: Searches on the current pos if there si the shit we need
            int currentVal = 0;
            if (actualPos == 'a') currentVal = A;
            else if (actualPos == 'b') currentVal = B;
            else if (actualPos == 'c') currentVal = C;

            if (currentVal == neededValue) {
                chamber = actualPos;

            } else {
                if (A == neededValue) chamber = 'a';
                else if (B == neededValue) chamber = 'b';
                else if (C == neededValue) chamber = 'c';
            }

            if (chamber == '\0') { //note:  skipeao
                shootStep++;
                shootStartTime = System.currentTimeMillis();
            } else {
                //note: changes the barrel pos
                double targetPos = (chamber == 'a') ? Aout : (chamber == 'b') ? Bout : Cout;
                mecanism.barril.setPosition(targetPos);
                actualPos = chamber;

                if (System.currentTimeMillis() - shootStartTime >= OUTTAKE_HOLD_TIME_MS) {
                    if (chamber == 'a') A = 0;
                    else if (chamber == 'b') B = 0;
                    else C = 0;

                    shootStep++;

                    if (shootStep >= 3) {
                        isShooting = false;
                        advanceToPreferredEmpty();
                    }
                    shootStartTime = System.currentTimeMillis();
                }
            }

            if (shootStep >= 3) {
                isShooting = false;
                advanceToPreferredEmpty();
            }

        } else {
//TL             --------- INTAKE MODE ------
            // empty chambers return to home
            if (A == 0 && B == 0 && C == 0) {
                mecanism.barril.setPosition(Ain);
                actualPos = 'a';
            }

            TestColorSensorMecanism.DetectedColor detected = mecanism.getDetectedColor(telemetry);

            boolean canIntakeNow = System.currentTimeMillis() - lastIntakeTime >= INTAKE_COOLDOWN_MS;

            if (canIntakeNow &&
                            (detected == TestColorSensorMecanism.DetectedColor.PURPLE ||
                            detected == TestColorSensorMecanism.DetectedColor.GREEN)) {

                int value = detected == TestColorSensorMecanism.DetectedColor.PURPLE ? 1 : 2;

                boolean actuallyLoaded = false;

                if (actualPos == 'a' && A == 0) {
                    A = value;
                    actuallyLoaded = true;
                } else if (actualPos == 'b' && B == 0) {
                    B = value;
                    actuallyLoaded = true;
                } else if (actualPos == 'c' && C == 0) {
                    C = value;
                    actuallyLoaded = true;
                }

                if (actuallyLoaded) {
                    advanceToPreferredEmpty();                  // rotate immediately
                    lastIntakeTime = System.currentTimeMillis(); // start cooldown so we don't instantly load the same pixel again
                }
            }

// Auto-advance even if we didn't intake (covers manual rotate, after shooting, pixel fell out, etc.)
            int currentValue = switch (actualPos) {
                case 'a' -> A;
                case 'b' -> B;
                case 'c' -> C;
                default -> 0;
            };

            if (currentValue != 0 && (A == 0 || B == 0 || C == 0)) {
                advanceToPreferredEmpty();
            }
        }
        telemetry.addData("A: ",A);
        telemetry.addData("B: ",B);
        telemetry.addData("C: ",C);
    }
//tl ----------------MOVE BARREL---------------------
    private void advanceToPreferredEmpty() {
        if (actualPos == 'a') {
            if (B == 0) {
                mecanism.barril.setPosition(Bin);
                actualPos = 'b';
            } else if (C == 0) {
                mecanism.barril.setPosition(Cin);
                actualPos = 'c';
            }
        } else if (actualPos == 'b') {
            if (A == 0) {
                mecanism.barril.setPosition(Ain);
                actualPos = 'a';
            } else if (C == 0) {
                mecanism.barril.setPosition(Cin);
                actualPos = 'c';
            }
        } else if (actualPos == 'c') {
            if (A == 0) {
                mecanism.barril.setPosition(Ain);
                actualPos = 'a';
            } else if (B == 0) {
                mecanism.barril.setPosition(Bin);
                actualPos = 'b';
            }
        }
    }
    public void autoShoot(){
        if (!isShooting && (PPG || PGP || GPP)) {
            isShooting = true;
            shootStep = 0;
            shootStartTime = System.currentTimeMillis();
        }
    }


}
