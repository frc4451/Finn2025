package frc.robot.autos;

import java.util.Arrays;
import java.util.Iterator;

public enum ChoreoPaths {
    SLtoCIJ("SLtoCIJ", false),
    SRtoCEF("SRtoCEF", false),
    SMtoCGH("SMtoCGH", false),
    CEFtoHR("CEFtoHR", false),
    CIJtoHL("CIJtoHL", false),
    CKLtoHL("CKLtoHL", false),
    HLtoCKL("HLtoCKL", false),
    HRtoCCD("HRtoCCD", false);

    public final String name;
    public final boolean scoring;

    private ChoreoPaths(String name, boolean scoring) {
        this.name = name;
        this.scoring = scoring;
    }

    public static Iterator<String> pathSequence(ChoreoPaths... paths) {
        return Arrays.stream(paths).map(path -> path.name).iterator();
    }
}