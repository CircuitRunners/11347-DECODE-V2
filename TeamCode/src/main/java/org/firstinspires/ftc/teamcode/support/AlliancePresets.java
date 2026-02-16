package org.firstinspires.ftc.teamcode.support;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

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

    public enum Cyphers {
        GPP(21),
        PGP(22),
        PPG(23);

        public final int cypherTag;

        Cyphers(int cypherTag) {
            this.cypherTag = cypherTag;
        }

        public int getCypherTag() {
            return cypherTag;
        }
    }

    public static int allianceShooterTag, currentCypher;
    public static Pose2D globalPose2D;
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

    public static void setCurrentPose2D(Pose2D currentPose) {
        globalPose2D = currentPose;
    }

    public static Pose2D getGlobalPose2D() {
        return globalPose2D;
    }

    public static void setCurrentPose(Pose currentPose) {
        globalPose = currentPose;
    }

    public static Pose getGlobalPose() {
        return globalPose;
    }
}
