
#ifndef _ECNT_CPUFREQ_H
#define _ECNT_CPUFREQ_H

enum e_cpu_freq {
    cpu_freq_500M=0,
    cpu_freq_550M,
    cpu_freq_600M,
    cpu_freq_650M,
    cpu_freq_700M,
    cpu_freq_750M,
    cpu_freq_800M,
    cpu_freq_850M,
    cpu_freq_900M,
    cpu_freq_950M,
	cpu_freq_1000M,
#if defined(TCSUPPORT_CPU_EN7581) || defined(TCSUPPORT_CPU_AN7583)
    cpu_freq_1050M,
    cpu_freq_1100M,
    cpu_freq_1150M,
    cpu_freq_1200M,
    /*
     * Steps above the vendor's 1200 MHz top. Only posdiv=0 works up here, so
     * the grid is a hard 50 MHz: freq = pcw * 50 with pcw an integer. 1450 is
     * the measured ceiling at stock Vcore on both Nokia XG-040G-MD (AN7581)
     * and XG-040G-MF (AN7583); 1500 hangs either board within seconds.
     *
     * Keep these in step with cpu_freq_config_pcw[], cpu_freq_config_posdiv[]
     * and armpll_clk_MHz[]: everything here is indexed by this enum and walked
     * up to cpu_freq_last, so a short array is an EL3 out-of-bounds read.
     */
    cpu_freq_1250M,
    cpu_freq_1300M,
    cpu_freq_1350M,
    cpu_freq_1400M,
    cpu_freq_1450M,
    /*
     * Headroom for raised Vcore. Nothing has been shown stable up here at
     * stock voltage - 1500 kills both boards - but the SoC can drive an
     * external buck through its AVS DAC, and these steps become reachable
     * once that is in use. They live here rather than being added later
     * because this table sits in the fip volume: extending it means
     * replacing the bootloader, while the DTS that decides which steps the
     * governor may actually ask for rides in the ordinary firmware image.
     *
     * Listing a step costs nothing on its own. It is only reachable if the
     * DTS offers it, and an index past the end is refused outright.
     */
    cpu_freq_1500M,
    cpu_freq_1550M,
    cpu_freq_1600M,
#endif
    cpu_freq_last
};

enum e_clk_src {
    clk_src_xtal=0,
    clk_src_armpll,
    clk_src_pll1,
    clk_src_pll2,
    clk_src_last
};

enum cpu_domain_clk_gating {
    cpu_clk_armpll,
    cpu_clk_pll1,
    cpu_clk_pll2,
    cpu_clk_armpll_div2
};

enum e_div_sel {
    div_sel_1=0,
    div_sel_2,
    div_sel_4,
    div_sel_6,
    div_sel_last
};

extern int en7523_armpll_set(enum e_cpu_freq cpuFreq);
extern void set_cpu_domain_clk_gating(enum cpu_domain_clk_gating pll, int isEnable);
unsigned int curr_armpll_clk_get (void);
int an7552_bootup_clk_src_switch(enum e_cpu_freq cpuFreq);

enum e_cpu_freq cpu_freq_enum_get(unsigned int armpll_clk);

#endif /* _ECNT_CPUFREQ_H */
