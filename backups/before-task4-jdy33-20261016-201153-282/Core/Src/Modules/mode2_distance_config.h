#ifndef MODE2_DISTANCE_CONFIG_H
#define MODE2_DISTANCE_CONFIG_H
/* Mode-2 straight-ground estimate, awaiting measured repeat validation.
 * OneStop run 20261016-184223-538: stopped estimate 1002 mm; user reports
 * about 50 mm short of 1000 mm, hence actual travel approximately 950 mm.
 * Scale only the post-turn displacement and displayed physical speed.
 * This is NOT the rejected 450/997 trial; mode-1 and turn guards are unchanged.
 */
#define MODE2_STRAIGHT_DISTANCE_SCALE (950.0f / 1002.0f)
#define MODE2_DISTANCE_PROVISIONAL 1U
#endif
