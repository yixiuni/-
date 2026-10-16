#ifndef ODOMETRY_CONFIG_H
#define ODOMETRY_CONFIG_H
/* Course calibration refined 2026-10-14 after one lap overshot by ~70 mm.
 * Same body reference at start/finish. Firmware run=37356 ms;
 * stop=L45258/R40570, settled=L45431/R40716, mean settled=43073.5.
 * Actual path estimate = 2*1500+2*pi*530+70 = 6400.088212805 mm.
 * Common scale = 43073.5/6400.088212805 = 6.730141612 counts/mm.
 * Inner/outer wheel travel differs in turns: use mean counts, NOT a full
 * lap distance for each wheel. This empirical course estimate needs
 * checking after speed/tire/load changes; mode 2 still benefits from
 * straight ground calibration. No timed lap cutoff.
 * Previous scale 6.809345408 displayed 6325 mm for this run.
 */
#define ODOMETRY_LEFT_COUNTS_PER_MM  6.730141612f
#define ODOMETRY_RIGHT_COUNTS_PER_MM 6.730141612f
#define TRACK_LAP_LENGTH_MM 6330U
/* Mean coast = 43073.5-(45258+40570)/2 = 159.5 counts = 23.70 mm.
 * Default 1200 cps setting. Verify the stopping position on the track. */
#define TRACK_STOP_COAST_MM 24U
/* Ground checks at 2400 cps / two laps, smooth approach:
 * With +30 mm trim, settled counts were 85410 / 85411 (12690 mm),
 * but ground endpoints were approximately 50 / 20 mm short. Center those
 * two repeat runs by another 35 mm, for total +65 mm. Observed ground
 * spread remains ~30 mm: this trim cannot remove run-to-run variation.
 * Apply once at the final stop, independent of lap geometry/encoder scale.
 * Other speed/lap combinations still require ground validation. */
#define TRACK_FINISH_TRIM_MM 65
#endif
