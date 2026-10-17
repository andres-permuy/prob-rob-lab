#!/usr/bin/env python3
"""Create a curated Lab 3 ZIP without raw/full-length recordings."""

from __future__ import annotations

import hashlib
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZIP_STORED, ZipFile


ROOT = Path(__file__).resolve().parents[1]
LAB = ROOT / "submission" / "lab3_Bayes_Filter"
OUTPUT = ROOT / "submission" / "lab3_Bayes_Filter_submission.zip"
ARCHIVE_ROOT = Path("lab3_Bayes_Filter_submission")


FILES = [
    (LAB / "START_HERE.txt", Path("START_HERE.txt")),
    (LAB / "Lab3_Bayes_Filter_Report.docx", Path("Lab3_Bayes_Filter_Report.docx")),
    (LAB / "evidence" / "assignment5_threshold_plot_annotated.png", Path("evidence/assignment5_threshold_plot_annotated.png")),
    (LAB / "evidence" / "prob_rob_lab3_#3_closed.png", Path("evidence/prob_rob_lab3_#3_closed.png")),
    (LAB / "evidence" / "prob_rob_lab3_#3_open.png", Path("evidence/prob_rob_lab3_#3_open.png")),
    (LAB / "evidence" / "prob_rob_lab3_#5_closed.png", Path("evidence/prob_rob_lab3_#5_closed.png")),
    (LAB / "evidence" / "prob_rob_lab3_#5_open.png", Path("evidence/prob_rob_lab3_#5_open.png")),
    (LAB / "evidence" / "logs" / "assignment5_open_stats.log", Path("evidence/logs/assignment5_open_stats.log")),
    (LAB / "evidence" / "logs" / "assignment5_closed_stats.log", Path("evidence/logs/assignment5_closed_stats.log")),
    (LAB / "evidence" / "logs" / "assignment6_bayes.log", Path("evidence/logs/assignment6_bayes.log")),
    (LAB / "evidence" / "logs" / "assignment7_flaky_opener.log", Path("evidence/logs/assignment7_flaky_opener.log")),
    (LAB / "evidence" / "logs" / "assignment7_trials.log", Path("evidence/logs/assignment7_trials.log")),
    (LAB / "evidence" / "logs" / "assignment8_full_bayes.log", Path("evidence/logs/assignment8_full_bayes.log")),
    (LAB / "video" / "prob_rob_lab3_#3.mp4", Path("video/prob_rob_lab3_#3.mp4")),
    (LAB / "video" / "prob_rob_lab3_#5.mp4", Path("video/prob_rob_lab3_#5.mp4")),
    (LAB / "video" / "prob_rob_lab3_#6.mp4", Path("video/prob_rob_lab3_#6.mp4")),
    (LAB / "video" / "prob_rob_lab3_#7.mp4", Path("video/prob_rob_lab3_#7.mp4")),
    (LAB / "video" / "prob_rob_lab3_#8.mp4", Path("video/prob_rob_lab3_#8.mp4")),
    (ROOT / "prob_rob_labs" / "package.xml", Path("source/prob_rob_labs/package.xml")),
    (ROOT / "prob_rob_labs" / "setup.cfg", Path("source/prob_rob_labs/setup.cfg")),
    (ROOT / "prob_rob_labs" / "setup.py", Path("source/prob_rob_labs/setup.py")),
    (ROOT / "prob_rob_labs" / "launch" / "door_controller_launch.py", Path("source/prob_rob_labs/launch/door_controller_launch.py")),
    (ROOT / "prob_rob_labs" / "launch" / "feature_stats_launch.py", Path("source/prob_rob_labs/launch/feature_stats_launch.py")),
    (ROOT / "prob_rob_labs" / "launch" / "lab3_controller_launch.py", Path("source/prob_rob_labs/launch/lab3_controller_launch.py")),
    (ROOT / "prob_rob_labs" / "src" / "door_controller" / "__init__.py", Path("source/prob_rob_labs/src/door_controller/__init__.py")),
    (ROOT / "prob_rob_labs" / "src" / "door_controller" / "door_controller.py", Path("source/prob_rob_labs/src/door_controller/door_controller.py")),
    (ROOT / "prob_rob_labs" / "src" / "door_controller" / "sequence.py", Path("source/prob_rob_labs/src/door_controller/sequence.py")),
    (ROOT / "prob_rob_labs" / "src" / "lab3_bayes" / "__init__.py", Path("source/prob_rob_labs/src/lab3_bayes/__init__.py")),
    (ROOT / "prob_rob_labs" / "src" / "lab3_bayes" / "controller.py", Path("source/prob_rob_labs/src/lab3_bayes/controller.py")),
    (ROOT / "prob_rob_labs" / "src" / "lab3_bayes" / "feature_stats.py", Path("source/prob_rob_labs/src/lab3_bayes/feature_stats.py")),
    (ROOT / "prob_rob_labs" / "src" / "lab3_bayes" / "filter.py", Path("source/prob_rob_labs/src/lab3_bayes/filter.py")),
    (ROOT / "prob_rob_labs" / "src" / "flaky_door_opener" / "flaky_door_opener.py", Path("source/prob_rob_labs/src/flaky_door_opener/flaky_door_opener.py")),
    (ROOT / "prob_rob_labs" / "test" / "test_door_sequence.py", Path("source/prob_rob_labs/test/test_door_sequence.py")),
    (ROOT / "prob_rob_labs" / "test" / "test_lab3_bayes_filter.py", Path("source/prob_rob_labs/test/test_lab3_bayes_filter.py")),
    (ROOT / "tools" / "run_assignment7_trials.py", Path("source/tools/run_assignment7_trials.py")),
]


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def main() -> None:
    missing = [str(source) for source, _ in FILES if not source.is_file()]
    if missing:
        raise SystemExit("Missing required files:\n" + "\n".join(missing))

    forbidden = [
        source for source, _ in FILES
        if source.suffix.lower() == ".mp4"
        and (source.name.startswith("Assignment") or source.name.startswith("Screen Recording"))
    ]
    if forbidden:
        raise SystemExit("Full-length recording selected: " + ", ".join(map(str, forbidden)))

    manifest_lines = []
    temp = OUTPUT.with_suffix(".zip.tmp")
    with ZipFile(temp, "w") as archive:
        for source, relative in FILES:
            archive_name = ARCHIVE_ROOT / relative
            compression = ZIP_STORED if source.suffix.lower() == ".mp4" else ZIP_DEFLATED
            archive.write(source, archive_name.as_posix(), compress_type=compression)
            manifest_lines.append(f"{digest(source)}  {relative.as_posix()}")
        archive.writestr(
            (ARCHIVE_ROOT / "MANIFEST.sha256").as_posix(),
            "\n".join(manifest_lines) + "\n",
            compress_type=ZIP_DEFLATED,
        )
    temp.replace(OUTPUT)
    print(OUTPUT)


if __name__ == "__main__":
    main()
