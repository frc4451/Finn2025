package frc.robot.autos;

import java.util.Arrays;
import java.util.Iterator;

public enum ChoreoPaths {
    SLtoCIJ("SLtoCIJ"),
    SRtoCEF("SRtoCEF"),
    SMBuffertoCGH("SMBuffertoCGH"),
    CEFtoHR("CEFtoHR"),
    CIJtoHL("CIJtoHL"),
    CKLtoHL("CKLtoHL"),
    HLtoCKL("HLtoCKL"),
    HRtoCCD("HRtoCCD"),
    CCDtoHR("CCDtoHR"),
    SRtoCEFtest("SRtoCEF(1)"),
    CEFtoHRtest("CEFtoHR(1)"),
    HRtoCCDtest("HRtoCCD(1)"),
    SMBuffer("SMBuffer");

    public final String name;

    private ChoreoPaths(String name) {
        this.name = name;
    }

    public static Iterator<String> pathSequence(ChoreoPaths... paths) {
        return Arrays.stream(paths).map(path -> path.name).iterator();
    }
}
// my cat "4y ````````"