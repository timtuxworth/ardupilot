"""Summarize paired SITL boot-clock samples from the Plane Follow benchmark.

AP_FLAKE8_CLEAN
"""

import csv
import os
import sys

from pathlib import Path


def summarize(samples: list[tuple[float, float, float]]) -> str:
    """Return endpoint rates and peak drift relative to the first sample."""
    if len(samples) < 2:
        return "insufficient paired clock samples"
    start_wall, start_follower, start_target = samples[0]
    end_wall, end_follower, end_target = samples[-1]
    wall = end_wall - start_wall
    if wall <= 0:
        return "zero wall time"
    follower = end_follower - start_follower
    target = end_target - start_target
    drift = target - follower
    peak = max(abs((t - start_target) - (f - start_follower)) for _, f, t in samples)
    return (
        f"N={len(samples)}, wall={wall:.2f}s, follower={follower/wall:.2f}x, "
        f"target={target/wall:.2f}x, target-minus-follower final={drift:+.3f}s, "
        f"peak absolute drift={peak:.3f}s"
    )


def main(console_path: Path, csv_path: Path) -> None:
    """Write the paired samples and emit a public check annotation."""
    phases = {}
    samples = []
    with csv_path.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(("phase", "wall_monotonic_s", "follower_boot_ms", "target_boot_ms"))
        for line in console_path.read_text(errors="replace").splitlines():
            if not line.startswith("PFCLOCK,"):
                continue
            _, phase, wall, follower, target = line.split(",")
            writer.writerow((phase, wall, follower, target))
            sample = (float(wall), int(follower) / 1000.0, int(target) / 1000.0)
            samples.append(sample)
            phases.setdefault(phase, []).append(sample)

    results = [f"Requested 100x interval – {summarize(samples)}",
               f"Tracking interval – {summarize(phases.get('tracking', []))}"]
    print("\n".join(results))
    if "GITHUB_STEP_SUMMARY" in os.environ:
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a") as summary:
            summary.write("\n".join(f"- {result}" for result in results) + "\n")
        for result in results:
            print(f"::notice title=Plane Follow clock benchmark::{result}")


if __name__ == "__main__":
    main(Path(sys.argv[1]), Path(sys.argv[2]))
