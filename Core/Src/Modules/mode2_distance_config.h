#ifndef MODE2_DISTANCE_CONFIG_H
#define MODE2_DISTANCE_CONFIG_H
/* PROVISIONAL straight-ground scale, NOT a precision calibration.
 * Run 20261016-180458-312: old estimate 997 mm, user reports "forty-something
 * cm" after turning. Use 450 mm temporarily pending a measured repeat.
 * Applied only to mode-2 straight distance and displayed physical speed.
 * Leave mode-1 course scale and existing turn/calibration travel guards alone.
 */
#define MODE2_STRAIGHT_DISTANCE_SCALE (450.0f / 997.0f)
#define MODE2_DISTANCE_PROVISIONAL 1U
#endif
