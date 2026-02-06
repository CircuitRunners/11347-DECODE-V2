package org.firstinspires.ftc.teamcode.support;

import com.pedropathing.geometry.Pose;

public class AlliancePresets {
    public enum Alliance {
        BLUE(20),
        RED(24);

        public final int tagId;

        Alliance(int tagId) {
            this.tagId = tagId;
        }

        public int getTagId() {
            return tagId;
        }
    }

    public static int allianceShooterTag, currentCypher;
    public static Pose globalPose;

    public static int getAllianceShooterTag() {
        return allianceShooterTag;
    }

    public static void setAllianceShooterTag(int allianceTag) {
        allianceShooterTag = allianceTag;
    }

    public static int getCurrentCypher() {
        return currentCypher;
    }

    public static void setCurrentCypherId(int cypher) {
        currentCypher = cypher;
    }

    public static void setCurrentPose(Pose currentPose) {
        globalPose = currentPose;
    }

    public static Pose getGlobalPose() {
        return globalPose;
    }
}
