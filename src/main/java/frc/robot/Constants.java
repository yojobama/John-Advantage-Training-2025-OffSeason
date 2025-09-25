// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static class s_TRANSFER_CONSTANTS {
        public static final double kTransferSpeed = 0.2;
        public static final int kTransferSensorID = 0;
        public static final int kTransferMotorID = 15;
    }
    public static class s_LIFT_CONSTANTS {
        public static final int ELEVATOR_ID = 16;
        public static final int FOLD_SWITCH = 1;
        public static final int BRAKE_SWITCH = 2;

        public static final double KP = 1.2;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double KS = 0.19;
        public static final double KG = 0.31;
        public static final double UPPER_KG = 0.15;
        public static final double KV = 0;
        public static final double RESIST_GRAVITY = 0;
        public static final double TOLERANCE = 0.15;

        public static final double MAX_ACCELERATION = 85;
        public static final double MAX_VELOCITY = 65;

        public static final double FORWARD_SOFT_LIMIT = 0;
        public static final double L2_POSITION = 34;
        public static final double L1_POSITION = 13.2;
        public static final double L3_POSITION = 55;

        public static final double CLOSE_ELEVATOR_SPEED = -0.5;

        public static final double POSITION_CONVERSION_FACTOR = 1;

        public static double KG_OF_CORAL = 0.1;

        public static final boolean INVERTED = true;

        public static final int CURRENT_LIMIT = 40;

        public static final double VOLTAGE_COMPENSATION = 12.0;

        public static final double UPPER_POSITION = 20;

        public static final double MANUAL_SLOW_OPEN = 2;
        public static final double MANUAL_SLOW_CLOSE = -1;
        public static final double MANUAL_FAST_OPEN = 3.5;
        public static final double MANUAL_FAST_CLOSE = -3;    }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
