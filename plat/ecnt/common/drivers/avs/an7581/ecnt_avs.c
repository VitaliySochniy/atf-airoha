/*
 * EN7581 CPU core voltage through the on-die AVS DAC, BL31 only.
 *
 * The vendor ships no AVS for this SoC: in its SDK and in the stock kernel every
 * DAC function for EN7581 is an empty stub, and there is no efuse calibration
 * for it either. What the silicon has was worked out on two Nokia XG-040G-MD
 * boards:
 *
 *   write protect	0x268, key 0x12
 *   DAC code		0x2e8 [9:0], bit 16 latches it
 *   DAC output enable	0x2a8 bit 0
 *   main ADC		mux 0x2ec [3:1], output 0x2f8
 *			1 = AVS pin (AVSMON), 2 = Vcore, 3 = DAC output
 *
 * A higher code gives a lower rail. Whether the DAC reaches the rail at all is
 * a property of the board, not of the chip: on one of the two boards the AVS
 * pin is not tied to the buck's feedback and no code moves Vcore. So nothing
 * is assumed. At startup, before Linux owns the ADC:
 *
 *  1. With the output off, the AVS pin sits on the buck's feedback reference
 *     and Vcore is at its stock value.
 *  2. The code whose DAC output matches the pin is searched for, so switching
 *     the output on does not step the rail.
 *  3. The output goes on and a probe moves the code. If Vcore does not follow,
 *     the board has no AVS path: the output goes off again and nothing above
 *     the stock rail is ever offered.
 *  4. From the measured slope, the code for the stock rail becomes the
 *     reference, and a raised level is checked against the ADC and undone.
 *
 * There is no absolute scale, so levels are offsets above this board's stock
 * rail. To turn millivolts into ADC units the feedback reference is taken as
 * 800 mV, which is what the pin reads with the output off; an error there
 * scales the offsets, it does not move the stock point.
 *
 * At runtime only the DAC code is written, from the reference and the slope,
 * with no ADC access: the kernel's thermal driver owns the mux then. The write
 * protect is shared with that driver, and a write made after it re-locked the
 * block is dropped silently, so every write is read back and repeated.
 */
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>

#define SCU_BASE		0x1fa20000
#define PLLRG_PROTECT		(SCU_BASE + 0x268)
#define PROTECT_KEY		0x12
#define AVS_OUT_EN		(SCU_BASE + 0x2a8)
#define AVS_DAC			(SCU_BASE + 0x2e8)
#define   DAC_CODE_MASK		0x3ff
#define   DAC_CHG		(1U << 16)
#define PWD_TADC		(SCU_BASE + 0x2ec)
#define   TADC_MUX_SHIFT	1
#define   TADC_MUX_MASK		(0x7U << TADC_MUX_SHIFT)
#define DOUT_TADC		(SCU_BASE + 0x2f8)

#define MUX_AVSMON		1
#define MUX_VCORE		2
#define MUX_DAC_OUT		3

#define WRITE_TRIES		5
#define DAC_PRE_MS		5	/* code stable before CHG rises */
#define DAC_CHG_HOLD_MS		10	/* CHG high */
#define ADC_SETTLE_MS		20
#define ADC_SAMPLES		8
#define ADC_DISCARD		2
#define ADC_GAP_MS		5
#define RAIL_SETTLE_MS		100	/* calibration, after the rail moved */
#define DAC_SETTLE_MS		10	/* runtime, as the vendor's AVS */
#define RAMP_CODES		25	/* calibration moves the rail gradually */

/* Sanity limits, ADC units (output >> 5). Both boards: pin 840, Vcore ~1062. */
#define AVSMON_MIN		700
#define AVSMON_MAX		1000
#define VCORE_MIN		900
#define VCORE_MAX		1250
#define FB_REF_MV		800

#define CODE_MIN		128
#define CODE_MAX		1000
#define MATCH_TOL		8	/* DAC output vs pin before switching on */
#define MATCH_DONE		2
#define MATCH_TURNS		12
#define MATCH_STEP_MAX		200
#define CODE_START		512
#define DACOUT_SLOPE		700	/* thousandths of a unit per code, both boards */
#define JUMP_MAX		30	/* rail step allowed when switching on */
#define PROBE_CODES		60
#define MOVE_MIN		6	/* less than this: no AVS path */
#define SLOPE_MIN		100	/* thousandths of a unit per code */
#define SLOPE_MAX		2000	/* second board: 350 */
#define REF_TOL			4
#define VERIFY_MV		50
#define OFFSET_MAX_MV		80

static unsigned int g_ref_code, g_slope, g_avsmon, g_now_mv;
static int g_ready;

static void protect_open(void)
{
	mmio_write_32(PLLRG_PROTECT,
		      (mmio_read_32(PLLRG_PROTECT) & ~0xffU) | PROTECT_KEY);
}

static void protect_close(void)
{
	mmio_write_32(PLLRG_PROTECT, mmio_read_32(PLLRG_PROTECT) & ~0xffU);
}

/* One protected write, read back in full; the key may be re-locked under us. */
static int scu_write_checked(uintptr_t reg, unsigned int val)
{
	int tries = WRITE_TRIES;

	do {
		protect_open();
		mmio_write_32(reg, val);
		protect_close();
		if (mmio_read_32(reg) == val)
			return 0;
	} while (--tries);

	return -1;
}

/*
 * The DAC takes a code on CHG, but not from a pulse of a few microseconds: at
 * BL31 speed most codes were never latched (the DAC output stayed on the
 * previous one), while from userspace, with milliseconds between the writes,
 * they mostly were. So the code is set first, CHG is held for a while, and
 * each of the three writes is confirmed on its own. That also means a CHG
 * whose clearing write was dropped is noticed and cleared again.
 */
static int dac_write(unsigned int code)
{
	if (scu_write_checked(AVS_DAC, code))
		return -1;
	mdelay(DAC_PRE_MS);
	if (scu_write_checked(AVS_DAC, code | DAC_CHG))
		return -1;
	mdelay(DAC_CHG_HOLD_MS);
	if (scu_write_checked(AVS_DAC, code))
		return -1;

	return 0;
}

static unsigned int dac_code(void)
{
	return mmio_read_32(AVS_DAC) & DAC_CODE_MASK;
}

static int output_enable(int on)
{
	int tries = WRITE_TRIES;
	unsigned int v;

	do {
		protect_open();
		v = mmio_read_32(AVS_OUT_EN);
		mmio_write_32(AVS_OUT_EN, on ? (v | 1U) : (v & ~1U));
		protect_close();
		if ((mmio_read_32(AVS_OUT_EN) & 1U) == (on ? 1U : 0U))
			return 0;
	} while (--tries);

	return -1;
}

static int adc_mux(unsigned int reg)
{
	int tries = WRITE_TRIES;

	do {
		protect_open();
		mmio_write_32(PWD_TADC, reg);
		protect_close();
		if (mmio_read_32(PWD_TADC) == reg)
			return 0;
	} while (--tries);

	return -1;
}

/* Average of a few conversions at mux position pos, 0 on failure. */
static unsigned int adc_read(unsigned int mux0, unsigned int pos)
{
	unsigned int i, sum = 0;
	unsigned int want = (mux0 & ~TADC_MUX_MASK) | (pos << TADC_MUX_SHIFT);

	/* Only a real change of position needs the ADC to settle again. */
	if (mmio_read_32(PWD_TADC) != want) {
		if (adc_mux(want))
			return 0;
		mdelay(ADC_SETTLE_MS);
	}

	/* The first conversions after a change can still be the old value. */
	for (i = 0; i < ADC_DISCARD; i++) {
		(void)mmio_read_32(DOUT_TADC);
		mdelay(ADC_GAP_MS);
	}
	for (i = 0; i < ADC_SAMPLES; i++) {
		sum += (mmio_read_32(DOUT_TADC) & 0xffffU) >> 5;
		mdelay(ADC_GAP_MS);
	}

	return (sum + ADC_SAMPLES / 2) / ADC_SAMPLES;
}

/* Move the code in small steps while the output drives the rail. */
static int dac_ramp(unsigned int code)
{
	unsigned int c = dac_code();

	while (c != code) {
		if (c > code)
			c = (c - code > RAMP_CODES) ? c - RAMP_CODES : code;
		else
			c = (code - c > RAMP_CODES) ? c + RAMP_CODES : code;
		if (dac_write(c))
			return -1;
		mdelay(DAC_SETTLE_MS);
	}
	mdelay(RAIL_SETTLE_MS);

	return 0;
}

static int abs_diff(unsigned int a, unsigned int b)
{
	return a > b ? (int)(a - b) : (int)(b - a);
}

/* ADC units for mV above the stock rail, then codes for those units. */
static unsigned int offset_codes(unsigned int mv)
{
	unsigned int units_x1000 = mv * g_avsmon * 1000U / FB_REF_MV;

	return (units_x1000 + g_slope / 2) / g_slope;
}

/* The code that raises the rail by units (lowers it if negative) from code. */
static unsigned int code_moved(unsigned int code, int units)
{
	int c = (int)code - units * 1000 / (int)g_slope;

	if (c < CODE_MIN)
		c = CODE_MIN;
	if (c > CODE_MAX)
		c = CODE_MAX;

	return (unsigned int)c;
}

/*
 * Leave the rail where the board's divider puts it. match is the code the
 * output was switched on at, 0 if it never was.
 */
static void give_up(unsigned int match)
{
	if (mmio_read_32(AVS_OUT_EN) & 1U) {
		if (match)
			dac_ramp(match);
		output_enable(0);
	}
	dac_write(0);
}

/*
 * Returns 0 with per-step voltage available, 1 when the board has no AVS path,
 * -1 when something did not behave and the rail was left at stock.
 */
int AVS_EN7581_Calibrate(void)
{
	unsigned int mux0, a0, v0, v1, v2, v3, v4, o, i, m, code;
	unsigned int match = 0, moved, want;
	int ret = -1, rise;

	g_ready = 0;
	mux0 = mmio_read_32(PWD_TADC);

	/* Nothing before BL31 turns it on; after a warm reset assume nothing. */
	if (mmio_read_32(AVS_OUT_EN) & 1U) {
		if (output_enable(0))
			return -1;
		mdelay(RAIL_SETTLE_MS);
	}

	a0 = adc_read(mux0, MUX_AVSMON);
	v0 = adc_read(mux0, MUX_VCORE);
	NOTICE("AVS: output off, pin %u, Vcore %u (~%u mV), mux 0x%x\n",
	       a0, v0, a0 ? v0 * FB_REF_MV / a0 : 0, mux0);
	if (v0 < VCORE_MIN || v0 > VCORE_MAX) {
		ERROR("AVS: Vcore reading out of range\n");
		goto out;
	}
	/*
	 * With the output off a pin tied to the buck sits on its feedback
	 * reference. On the first MD it reads 1113 instead (the second: 840):
	 * it is not on the feedback node at all.
	 */
	if (a0 < AVSMON_MIN || a0 > AVSMON_MAX) {
		NOTICE("AVS: pin at %u with the output off is no feedback reference, no AVS path on this board\n",
		       a0);
		ret = 1;
		goto out;
	}

	/*
	 * The DAC output rises with the code, output enabled or not, by about
	 * DACOUT_SLOPE thousandths of a unit per code. Walk towards the pin
	 * with that slope rather than bisecting: on the first boot a bisection
	 * was thrown off for good by a single stale read, while this simply
	 * corrects itself on the next turn.
	 */
	m = CODE_START;
	o = 0;
	for (i = 0; i < MATCH_TURNS; i++) {
		int step;

		if (dac_write(m))
			goto fail;
		mdelay(DAC_SETTLE_MS);
		o = adc_read(mux0, MUX_DAC_OUT);
		NOTICE("AVS:   code %u -> DAC output %u\n", m, o);
		if (abs_diff(o, a0) <= MATCH_DONE)
			break;
		step = ((int)a0 - (int)o) * 1000 / DACOUT_SLOPE;
		if (step > MATCH_STEP_MAX)
			step = MATCH_STEP_MAX;
		if (step < -MATCH_STEP_MAX)
			step = -MATCH_STEP_MAX;
		if (step == 0)
			step = a0 > o ? 1 : -1;
		m = (int)m + step < CODE_MIN ? CODE_MIN :
		    (int)m + step > CODE_MAX ? CODE_MAX : (unsigned int)((int)m + step);
	}
	if (abs_diff(o, a0) > MATCH_TOL || m < CODE_MIN + PROBE_CODES) {
		ERROR("AVS: no usable DAC code matches the pin (%u at %u, pin %u)\n",
		      o, m, a0);
		goto fail;
	}

	match = m;
	if (output_enable(1))
		goto fail;
	mdelay(RAIL_SETTLE_MS);
	v1 = adc_read(mux0, MUX_VCORE);
	NOTICE("AVS: output on at code %u, Vcore %u\n", match, v1);
	if (abs_diff(v1, v0) > JUMP_MAX) {
		ERROR("AVS: rail stepped on switch-on\n");
		goto fail;
	}

	if (dac_ramp(match - PROBE_CODES))
		goto fail;
	v2 = adc_read(mux0, MUX_VCORE);
	moved = v2 > v1 ? v2 - v1 : 0;
	if (moved < MOVE_MIN) {
		NOTICE("AVS: Vcore does not follow the DAC (%u -> %u), no AVS path on this board\n",
		       v1, v2);
		give_up(match);
		ret = 1;
		goto out;
	}
	g_slope = moved * 1000U / PROBE_CODES;
	if (g_slope < SLOPE_MIN || g_slope > SLOPE_MAX) {
		ERROR("AVS: slope %u/1000 units per code out of range\n", g_slope);
		goto fail;
	}

	/* The code that gives the stock rail back. */
	code = code_moved(match, (int)v0 - (int)v1);
	if (dac_ramp(code))
		goto fail;
	v3 = adc_read(mux0, MUX_VCORE);
	if (abs_diff(v3, v0) > REF_TOL) {
		code = code_moved(code, (int)v0 - (int)v3);
		if (dac_ramp(code))
			goto fail;
		v3 = adc_read(mux0, MUX_VCORE);
		if (abs_diff(v3, v0) > 2 * REF_TOL) {
			ERROR("AVS: cannot hold the stock rail (%u, want %u)\n",
			      v3, v0);
			goto fail;
		}
	}
	g_ref_code = code;
	g_avsmon = a0;

	/* Check a raised level against the ADC and come back. */
	want = VERIFY_MV * a0 / FB_REF_MV;
	if (dac_ramp(code_moved(g_ref_code, (int)want)))
		goto fail;
	v4 = adc_read(mux0, MUX_VCORE);
	if (dac_ramp(g_ref_code))
		goto fail;
	rise = (int)v4 - (int)v3;
	NOTICE("AVS: slope %u/1000 units per code, stock rail at code %u, +%u mV -> %d units (want %u)\n",
	       g_slope, g_ref_code, VERIFY_MV, rise, want);
	if (rise < (int)(want - want / 3) - 2 || rise > (int)(want + want / 3) + 2) {
		ERROR("AVS: raised level missed\n");
		goto fail;
	}

	g_now_mv = 0;
	g_ready = 1;
	ret = 0;
	goto out;

fail:
	give_up(match);
out:
	adc_mux(mux0);
	return ret;
}

int AVS_Fast_Ready(void)
{
	return g_ready;
}

unsigned int AVS_Fast_Now(void)
{
	return g_now_mv;
}

/* mv is an offset above the stock rail. CPU parked on PLL2 by the caller. */
int AVS_Fast_Set_mV(unsigned int mv)
{
	unsigned int code, delta;

	if (!g_ready)
		return -1;
	if (mv > OFFSET_MAX_MV)
		mv = OFFSET_MAX_MV;
	if (mv == g_now_mv)
		return 0;

	delta = offset_codes(mv);
	code = g_ref_code > CODE_MIN + delta ? g_ref_code - delta : CODE_MIN;
	if (dac_write(code))
		return -1;

	mdelay(DAC_SETTLE_MS);
	g_now_mv = mv;

	return 0;
}
