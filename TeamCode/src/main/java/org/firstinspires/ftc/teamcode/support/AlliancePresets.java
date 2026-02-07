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
    public static Pose2D globalPose;

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

    public static void setCurrentPose(Pose2D currentPose) {
        globalPose = currentPose;
    }

    public static Pose2D getGlobalPose() {
        return globalPose;
    }
}
