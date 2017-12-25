/*
 *  linux/arch/i386/kernel/time.c
 *
 *  Copyright (C) 1991, 1992, 1995  Linus Torvalds
 *
 * This file contains the PC-specific time handling details:
 * reading the RTC at bootup, etc..
 * 1994-07-02    Alan Modra
 *	fixed set_rtc_mmss, fixed time.year for >= 2000, new mktime
 * 1995-03-26    Markus Kuhn
 *      fixed 500 ms bug at call to set_rtc_mmss, fixed DS12887
 *      precision CMOS clock update
 * 1996-05-03    Ingo Molnar
 *      fixed time warps in do_[slow|fast]_gettimeoffset()
 * 1997-09-10	Updated NTP code according to technical memorandum Jan '96
 *		"A Kernel Model for Precision Timekeeping" by Dave Mills
 * 1998-09-05    (Various)
 *	More robust do_fast_gettimeoffset() algorithm implemented
 *	(works with APM, Cyrix 6x86MX and Centaur C6),
 *	monotonic gettimeofday() with fast_get_timeoffset(),
 *	drift-proof precision TSC calibration on boot
 *	(C. Scott Ananian <cananian@alumni.princeton.edu>, Andrew D.
 *	Balsa <andrebalsa@altern.org>, Philip Gladstone <philip@raptor.com>;
 *	ported from 2.0.35 Jumbo-9 by Michael Krause <m.krause@tu-harburg.de>).
 * 1998-12-16    Andrea Arcangeli
 *	Fixed Jumbo-9 code in 2.1.131: do_gettimeofday was missing 1 jiffy
 *	because was not accounting lost_ticks.
 * 1998-12-24 Copyright (C) 1998  Andrea Arcangeli
 *	Fixed a xtime SMP race (we need the xtime_lock rw spinlock to
 *	serialize accesses to xtime/lost_ticks).
 */

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/smp.h>

#include <asm/io.h>
#include <asm/smp.h>
#include <asm/irq.h>
#include <asm/msr.h>
#include <asm/delay.h>
#include <asm/mpspec.h>
#include <asm/uaccess.h>
#include <asm/processor.h>

#include <linux/mc146818rtc.h>
#include <linux/timex.h>
#include <linux/config.h>

#include <asm/fixmap.h>
#include <asm/cobalt.h>

/*
 * for x86_do_profile()
 */
#include <linux/irq.h>


unsigned long cpu_khz;	/* Detected as we calibrate the TSC */ // 无符号长整数32bit CPU频率 系统时钟频率

/* Number of usecs that the last interrupt was delayed */

/*
* 中断服务执行延迟
* 产生中断到内核中断服务程序是有时间延迟的
*/
static int delay_at_last_interrupt; //delay_at_last_interrupt来表示这一段时间延迟

/*
* 中断服务time_interupt真正在CPU上执行时刻的TSC寄存器值的低32位
*/
static unsigned long last_tsc_low; /* lsb 32 bits of Time Stamp Counter */

/* Cached *multiplier* to convert TSC counts to microseconds.
 * (see the equation below).
 * Equal to 2^32 * (1 / (clocks per usec) ).
 * Initialized in time_init.
 */

/*
* 至此，通过delay_at_last_interrupt, last_tsc_low和时刻X处的TSC寄存器的值
* 可以完全确定时刻X距上一次时钟中断产生时刻的时间间隔偏移量offset_usec
*/


/*
* 表示1个TSC计数值代表多长的时间间隔
*/
unsigned long fast_gettimeoffset_quotient;


/*
* 相对时间锁，保证进程对xtime修改的互斥性
*/
extern rwlock_t xtime_lock;

/*
* jiffies：
* 内核全局变量jiffies，改变量用来记录系统起来以后产生的节拍总数
* 系统启动时，该变量被设置为0，此后每产生一次时钟中断就增加该变量的值。每一秒增加的值就是HZ
* 32位的unsigned long，最大可以存放4294967295，当达到最大值时，再回到零
*/

/*
* 保存内核上一次更新Xtime时的jiffies值
*/
extern unsigned long wall_jiffies;  //记录系统起来以后产生的节拍总数


/*
* rtc锁，保证进程对rtc操作的互斥，初始状态为未锁状态
*/
spinlock_t rtc_lock = SPIN_LOCK_UNLOCKED;


/*
* 具有TSC寄存器的情况
* 通过TSC寄存器来计算do_fast_gettimeoffset(void)函数被执行的时刻距上次上次时钟中断的时间间隔
* 计算函数执行时到上次时钟中断的时间
*/
static inline unsigned long do_fast_gettimeoffset(void)
{
	register unsigned long eax, edx;

	/* Read the Time Stamp Counter */

	/*
    * 读取当前时刻的tsc寄存器的值
    * 将高32位存入edx
    * 将低32位存入eax
    */
    rdtsc(eax,edx);

	/* .. relative to previous jiffy (32 bits is enough) */
    /*
    * 读到的低32位数减去上次执行timer_interrupt()时TSC寄存器的值
    */
	eax -= last_tsc_low;	/* tsc_low delta */

	/*
         * Time offset = (tsc_low delta) * fast_gettimeoffset_quotient
         *             = (tsc_low delta) * (usecs_per_clock)
         *             = (tsc_low delta) * (usecs_per_jiffy / clocks_per_jiffy)
	 *
	 * Using a mull instead of a divl saves up to 31 clock cycles
	 * in the critical path.
         */

    /*
    * C语言内嵌汇编  __asm__(汇编语句模板: 输出部分: 输入部分: 破坏描述部分)
    * Time offset = (tsc_low delta) * fast_gettimeoffset_quotient)
    * 进行乘法运算从上一次执行timer_interrupt到当前时刻的时间间隔
    */ 
	__asm__("mull %2"
		:"=a" (eax), "=d" (edx)
		:"rm" (fast_gettimeoffset_quotient),
		 "0" (eax));

	/* our adjusted time offset in microseconds */
    /*
    * return结果表示从上一次时钟中断发生时到当前时刻之间的时间偏移间隔值
    */
	return delay_at_last_interrupt + edx;
}


/*
* 用全局变量来表示时间滴答的时间间隔长度
* tick变量的单位是微秒
*/
#define TICK_SIZE tick

/*
* 计数器8253锁，初始状态为未锁
*/
spinlock_t i8253_lock = SPIN_LOCK_UNLOCKED;

extern spinlock_t i8259A_lock;

#ifndef CONFIG_X86_TSC

/* This function must be called with interrupts disabled 
 * It was inspired by Steve McCanne's microtime-i386 for BSD.  -- jrs
 * 
 * However, the pc-audio speaker driver changes the divisor so that
 * it gets interrupted rather more often - it loads 64 into the
 * counter rather than 11932! This has an adverse impact on
 * do_gettimeoffset() -- it stops working! What is also not
 * good is that the interval that our timer function gets called
 * is no longer 10.0002 ms, but 9.9767 ms. To get around this
 * would require using a different timing source. Maybe someone
 * could use the RTC - I know that this can interrupt at frequencies
 * ranging from 8192Hz to 2Hz. If I had the energy, I'd somehow fix
 * it so that at startup, the timer code in sched.c would select
 * using either the RTC or the 8253 timer. The decision would be
 * based on whether there was any other device around that needed
 * to trample on the 8253. I'd set up the RTC to interrupt at 1024 Hz,
 * and then do some jiggery to have a version of do_timer that 
 * advanced the clock by 1/1024 s. Every time that reached over 1/100
 * of a second, then do all the old code. If the time was kept correct
 * then do_gettimeoffset could just return 0 - there is no low order
 * divider that can be accessed.
 *
 * Ideally, you would be able to use the RTC for the speaker driver,
 * but it appears that the speaker driver really needs interrupt more
 * often than every 120 us or so.
 *
 * Anyway, this needs more thought....		pjsg (1993-08-28)
 * 
 * If you are really that interested, you should be reading
 * comp.protocols.time.ntp!
 */


/*
* 不具有TSC寄存器的情况，运用可编程间隔定时器 PIT
* Intel 8254 PIT有3个计时通道，每个通道都有其不同的用途： 
*（1） 通道0用来负责更新系统时钟。每当一个时钟滴答过去时，它就会通过IRQ0向系统产生一次时钟中断。 
*（2） 通道1通常用于控制DMAC对RAM的刷新。 
*（3） 通道2被连接到PC机的扬声器，以产生方波信号。 
* 在无TSC寄存器时来计算do_fast_gettimeoffset(void)函数被执行的时刻距上次上次时钟中断的时间间隔
* 计算函数执行时到上次时钟中断的时间
*/

/*
每个通道都有一个向下减小的计数器，8254 PIT的输入时钟信号的频率是1.193181MHZ，也即一秒钟输入1193181个clock-cycle。每输入一个clock-cycle其时间通道的计数器就向下减1，一直减到0值。因此对于通道0而言，当他的计数器减到0时，PIT就向系统产生一次时钟中断，表示一个时钟滴答已经过去了。计数器为16bit,因此所能表示的最大值是65536，一秒内发生的滴答数是：1193181/65536=18.206482.
*/

/*
PIT的I/O端口：
     0x40   通道0 计数器 Read/Write
     0X41   通道1计数器 Read/Write
     0X42  通道2计数器  Read/Write
     0X43   控制字        Write Only
 Note: 因PIT I/O端口是8位，而PIT相应计数器是16位，因此必须对PIT计数器进行两次读写。
*/

static unsigned long do_slow_gettimeoffset(void)
{
	int count;

	/*
    * LATCH是计数初值，定义要写到PIT通道0的计数器中的值，它表示PIT将隔多少个时钟周期产生一次时钟中断
    */
    static int count_p = LATCH;    /* for the first call after boot */
	static unsigned long jiffies_p = 0;

	/*
	 * cache volatile jiffies temporarily; we have IRQs turned off. 
	 */
	unsigned long jiffies_t;

	/* gets recalled with irq locally disabled */
	spin_lock(&i8253_lock);
	/* timer count may underflow right here */

    /*
    * 把控制字写进8253控制端口
    */
	outb_p(0x00, 0x43);	/* latch the count ASAP */

    /*
    * 把计数器0的值写进count
    */    
	count = inb_p(0x40);	/* read the latched count */

	/*
	 * We do this guaranteed double memory access instead of a _p 
	 * postfix in the previous port access. Wheee, hackady hack
	 */
 	jiffies_t = jiffies;
    
    /*
     * PIT相应计数器是16位，因此必须对PIT计数器进行两次读写
     * 左移八位再或，相当于是在进行第二次读写，填充16位计数器
     */
	count |= inb_p(0x40) << 8;
	
        /* VIA686a test code... reset the latch if count > max + 1 */

        /*
         * 当count太高时，复位的过程
        */
        if (count > LATCH) {
                outb_p(0x34, 0x43);
                outb_p(LATCH & 0xff, 0x40);
                outb(LATCH >> 8, 0x40);
                count = LATCH - 1;
        }
	
	spin_unlock(&i8253_lock);

	/*
	 * avoiding timer inconsistencies (they are rare, but they happen)...
	 * there are two kinds of problems that must be avoided here:
	 *  1. the timer counter underflows
	 *  2. hardware problem with the timer, not giving us continuous time,
	 *     the counter does small "jumps" upwards on some Pentium systems,
	 *     (see c't 95/10 page 335 for Neptun bug.)
	 */

/* you can safely undefine this if you don't have the Neptune chipset */

#define BUGGY_NEPTUN_TIMER

	if( jiffies_t == jiffies_p ) {
		if( count > count_p ) {
			/* the nutcase */

			int i;

			spin_lock(&i8259A_lock);
			/*
			 * This is tricky when I/O APICs are used;
			 * see do_timer_interrupt().
			 */
			i = inb(0x20);
			spin_unlock(&i8259A_lock);

			/* assumption about timer being IRQ0 */
			if (i & 0x01) {
				/*
				 * We cannot detect lost timer interrupts ... 
				 * well, that's why we call them lost, don't we? :)
				 * [hmm, on the Pentium and Alpha we can ... sort of]
				 */
				count -= LATCH;
			} else {
#ifdef BUGGY_NEPTUN_TIMER
				/*
				 * for the Neptun bug we know that the 'latch'
				 * command doesnt latch the high and low value
				 * of the counter atomically. Thus we have to 
				 * substract 256 from the counter 
				 * ... funny, isnt it? :)
				 */

				count -= 256;
#else
				printk("do_slow_gettimeoffset(): hardware timer problem?\n");
#endif
			}
		}
	} else
		jiffies_p = jiffies_t;

	count_p = count;

    /*
    * TICK_SIZE是时间滴答的时间间隔长度
    * latch：多少个周期产生一次中断
    * count：剩余可减周期数
    * ((LATCH-1) - count)一共经历了几个周期
    * ((LATCH-1) - count) * TICK_SIZE：这些周期所使用的时间间隔长度
    */
	count = ((LATCH-1) - count) * TICK_SIZE;
    /*
    * (count + LATCH/2) / LATCH：计算每个中断所使用的时间间隔长度
    */
	count = (count + LATCH/2) / LATCH;

	return count;
}

static unsigned long (*do_gettimeoffset)(void) = do_slow_gettimeoffset;

#else

#define do_gettimeoffset()	do_fast_gettimeoffset()

#endif

/*
 * This version of gettimeofday has microsecond resolution
 * and better than microsecond precision on fast x86 machines with TSC.
 */

/*
 * 完成实际的当前时间检索工作
 * 精确地修正xtime的值
 */
void do_gettimeofday(struct timeval *tv)
{
	unsigned long flags;
	unsigned long usec, sec;

	read_lock_irqsave(&xtime_lock, flags);
	
    /*
    * 调用函数do_gettimeoffset()计算从上一次时钟中断到现在的时间间隔
    */
    usec = do_gettimeoffset();
	{

		unsigned long lost = jiffies - wall_jiffies;  //通过wall_jiffies和jiffies计算丢失的时钟间隔
		if (lost)
			usec += lost * (1000000 / HZ);   //1000000 / HZ：tick时钟滴答时间间隔长度
	}
	sec = xtime.tv_sec;    //表示系统当前时间在秒数量级上的值
	usec += xtime.tv_usec; //表示系统当前时间在微秒数量级上的值
	read_unlock_irqrestore(&xtime_lock, flags);

    /*
    * 判断usec是否溢出
    * 如果溢出，usec减去1000000us；sec加上1s
    */  
	while (usec >= 1000000) {
		usec -= 1000000;
		sec++;
	}

    /*
    * 把修正过的值存入tv时间
    */
	tv->tv_sec = sec;
	tv->tv_usec = usec;
}


/*
 * 把tv时间设置为系统时间
 * 把修正过的值存入xtime
 */
void do_settimeofday(struct timeval *tv)
{
	write_lock_irq(&xtime_lock);
	/*
	 * This is revolting. We need to set "xtime" correctly. However, the
	 * value in this location is the value at the most recent update of
	 * wall time.  Discover what correction gettimeofday() would have
	 * made, and then undo it!
	 */

    /*
    * 调用do_gettimeoffset()计算上一次时钟中断发生时刻到当前时刻之间的时间间隔值
    */
	tv->tv_usec -= do_gettimeoffset();
    /*
    * 计算jiffies 和 wall_jiffies 二者之间的时间间隔值 lost
    */
	tv->tv_usec -= (jiffies - wall_jiffies) * (1000000 / HZ);

    /*
    * 调整usec和sec之间的进位关系
    */
	while (tv->tv_usec < 0) {
		tv->tv_usec += 1000000;
		tv->tv_sec--;
	}

    /*
    * 修正过的tv时间更新内核全局时间变量xtime
    */
	xtime = *tv;
	time_adjust = 0;		/* stop active adjtime() */
	time_status |= STA_UNSYNC;
	time_maxerror = NTP_PHASE_LIMIT;
	time_esterror = NTP_PHASE_LIMIT;
	write_unlock_irq(&xtime_lock);
}

/*
 * In order to set the CMOS clock precisely, set_rtc_mmss has to be
 * called 500 ms after the second nowtime has started, because when
 * nowtime is written into the registers of the CMOS clock, it will
 * jump to the next second precisely 500 ms later. Check the Motorola
 * MC146818A or Dallas DS12887 data sheet for details.
 *
 * BUG: This routine does not handle hour overflow properly; it just
 *      sets the minutes. Usually you'll only notice that after reboot!
 */

/*
 * 向RTC中写回当前的时间与日期
 * 该函数用来更新RTC中的时间
 * 参数nowtime，是以秒数表示的当前时间
 */
static int set_rtc_mmss(unsigned long nowtime)
{
	int retval = 0;
	int real_seconds, real_minutes, cmos_minutes;
	unsigned char save_control, save_freq_select;

	/* gets recalled with irq locally disabled */
	spin_lock(&rtc_lock);

    /*
     * 把RTC寄存器中的当前的值读到变量save_control中
     */
	save_control = CMOS_READ(RTC_CONTROL); /* tell the clock it's being set */
    /*
     * 把(save_control|RTC_SET)写回到RTC_CONTROL中
     * 通知RTC软件要更新它的时间与日期
     */
	CMOS_WRITE((save_control|RTC_SET), RTC_CONTROL);

    /*
     * 把RTC_FREQ_SELECT寄存器中的当前的值读到变量save_freq_select中
     */
	save_freq_select = CMOS_READ(RTC_FREQ_SELECT); /* stop and reset prescaler */

    /*
     * 通过RTC_FREQ_SELECT寄存器bit[6:4]重启RTC芯片内部的除法器
     */
	CMOS_WRITE((save_freq_select|RTC_DIV_RESET2), RTC_FREQ_SELECT);
	
    /*
     * 将RTC_MINUTES寄存器当前值读到变量cmos_minutes中
     * 并根据需要将它从BCD格式转化成二进制格式
     */
    cmos_minutes = CMOS_READ(RTC_MINUTES);
	if (!(save_control & RTC_DM_BINARY) || RTC_ALWAYS_BCD)
		BCD_TO_BIN(cmos_minutes);

	/*
	 * since we're only adjusting minutes and seconds,
	 * don't interfere with hour overflow. This avoids
	 * messing with unknown time zones but requires your
	 * RTC not to be off by more than 15 minutes
	 */

    /*
     * 从nowtime参数中得到当前时间的秒数和分钟数
     * 并将real_seconds和real_minutes进行整理
     */
	real_seconds = nowtime % 60;
	real_minutes = nowtime / 60;
	if (((abs(real_minutes - cmos_minutes) + 15)/30) & 1)
		real_minutes += 30;		/* correct for half hour time zone */
	real_minutes %= 60;


    /*
     * real_minutes和RTC_MINUTES寄存器里的原值进行比较
     * 二者不超过30分钟的情况下
     * 将real_seconds和real_minutes的值转回BCD格式并写进RTC秒寄存器和分钟寄存器
     */
	if (abs(real_minutes - cmos_minutes) < 30) {
		if (!(save_control & RTC_DM_BINARY) || RTC_ALWAYS_BCD) {
			BIN_TO_BCD(real_seconds);
			BIN_TO_BCD(real_minutes);
		}
		CMOS_WRITE(real_seconds,RTC_SECONDS);
		CMOS_WRITE(real_minutes,RTC_MINUTES);
	} else {
		printk(KERN_WARNING
		       "set_rtc_mmss: can't update from %d to %d\n",
		       cmos_minutes, real_minutes);
		retval = -1;
	}

	/* The following flags have to be released exactly in this order,
	 * otherwise the DS12887 (popular MC146818A clone with integrated
	 * battery and quartz) will not reset the oscillator and will not
	 * update precisely 500 ms later. You won't find this mentioned in
	 * the Dallas Semiconductor data sheets, but who believes data
	 * sheets anyway ...                           -- Markus Kuhn
	 */

    /*
     * 恢复RTC_CONTROL和RTC_FREQ_SELECT寄存器原来的值
     * 最后再解除自旋锁
     */
	CMOS_WRITE(save_control, RTC_CONTROL);
	CMOS_WRITE(save_freq_select, RTC_FREQ_SELECT);
	spin_unlock(&rtc_lock);

	return retval;
}

/* last time the cmos clock got updated */
/*
 * 每一次成功地调用set_rtc_mmss()函数后，内核都会马上将last_rtc_update更新为当前时间（单位是秒数）
 */
static long last_rtc_update;

int timer_ack;  //时间中断申请

/*
 * timer_interrupt() needs to keep up the real-time clock,
 * as well as call the "do_timer()" routine every clocktick
 */

/*
 * 中断服务程序通用例程
 * 执行真正的中断时钟服务
 */
static inline void do_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
#ifdef CONFIG_X86_IO_APIC
	if (timer_ack) {
		/*
		 * Subtle, when I/O APICs are used we have to ack timer IRQ
		 * manually to reset the IRR bit for do_slow_gettimeoffset().
		 * This will also deassert NMI lines for the watchdog if run
		 * on an 82489DX-based system.
		 */

        /*
         * 若有时钟中断请求，响应后
         * 锁上8259A，向控制端口写控制字
         * 解锁
         */
		spin_lock(&i8259A_lock);
		outb(0x0c, 0x20);
		/* Ack the IRQ; AEOI will end it automatically. */
		inb(0x20);
		spin_unlock(&i8259A_lock);
	}
#endif

#ifdef CONFIG_VISWS
	/* Clear the interrupt */
    /* 修改时间中断标志位 */
	co_cpu_write(CO_CPU_STAT,co_cpu_read(CO_CPU_STAT) & ~CO_STAT_TIMEINTR);
#endif
	do_timer(regs);
    /*
     * do_timer()完成三个任务
     * 1、将标志自系统启动以来的时钟滴答计数变量jiffies加一
     * 2、调用update_process_times()函数更新当前进程的时间统计信息
     * 3、调用mark_bh()函数激活时钟中断的BottomHalf向量TIMER_BH 和 TQUEUE_BH
     */
/*
 * In the SMP case we use the local APIC timer interrupt to do the
 * profiling, except when we simulate SMP mode on a uniprocessor
 * system, in that case we have to call the local interrupt handler.
 */
#ifndef CONFIG_X86_LOCAL_APIC
	if (!user_mode(regs))
		x86_do_profile(regs->eip);
#else
	if (!using_apic_timer)
		smp_local_timer_interrupt(regs);
#endif

	/*
	 * If we have an externally synchronized Linux clock, then update
	 * CMOS clock accordingly every ~11 minutes. Set_rtc_mmss() has to be
	 * called as close as possible to 500 ms before the new second starts.
	 */

    /*
     * 判断是否需要更新CMOS时钟（即RTC）中的时间
     * Linux仅在下列三个条件同时成立时才更新CMOS时钟
     * 1、系统全局时间状态变量time_status中没有设置STA_UNSYNC标志
     * 2、
     * 3、在一秒时间间隔的中间位置500ms左右调用set_rtc_mmss()
     * 上述条件成立则调用set_rtc_mmss()将当前时间xtime.tv_sec更新写回到RTC
     */
	if ((time_status & STA_UNSYNC) == 0 &&
	    xtime.tv_sec > last_rtc_update + 660 &&
	    xtime.tv_usec >= 500000 - ((unsigned) tick) / 2 &&
	    xtime.tv_usec <= 500000 + ((unsigned) tick) / 2) {
		if (set_rtc_mmss(xtime.tv_sec) == 0)
			/* 更新成功，把xtime.tv_sec作为最近一次RTC更新时间，写入last_rtc_update */
            last_rtc_update = xtime.tv_sec;
		else
            /* 更新失败，将xtime.tv_sec - 600，以便在60秒之后再次对RTC进行更新 */
			last_rtc_update = xtime.tv_sec - 600; /* do it again in 60 s */
	}
	    
#ifdef CONFIG_MCA
	if( MCA_bus ) {
		/* The PS/2 uses level-triggered interrupts.  You can't
		turn them off, nor would you want to (any attempt to
		enable edge-triggered interrupts usually gets intercepted by a
		special hardware circuit).  Hence we have to acknowledge

		the timer interrupt.  Through some incredibly stupid
		design idea, the reset for IRQ 0 is done by setting the
		high bit of the PPI port B (0x61).  Note that some PS/2s,
		notably the 55SX, work fine if this is removed.  */

		irq = inb_p( 0x61 );	/* read the current state */
		outb_p( irq|0x80, 0x61 );	/* reset the IRQ */
	}
#endif
}

/*全局变量表示内核是否使用CPU的TSC寄存器，1为使用，0为不使用*/
static int use_tsc;

/*
 * This is the same as the above, except we _also_ save the current
 * Time Stamp Counter value at the time of the timer interrupt, so that
 * we later on can estimate the time of day more exactly.
 */

/*
 * 时钟中断程序
 */
static void timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	int count;

	/*
	 * Here we are in the timer irq handler. We just have irqs locally
	 * disabled but we don't know if the timer_bh is running on the other
	 * CPU. We need to avoid to SMP race with it. NOTE: we don' t need
	 * the irq version of write_lock because as just said we have irq
	 * locally disabled. -arca
	 */

    /*给xtime加锁*/
	write_lock(&xtime_lock);

    /*如果内核使用CPU的TSC寄存器*/
	if (use_tsc)
	{
		/*
		 * It is important that these two operations happen almost at
		 * the same time. We do the RDTSC stuff first, since it's
		 * faster. To avoid any inconsistencies, we need interrupts
		 * disabled locally.
		 */

		/*
		 * Interrupts are just disabled locally since the timer irq
		 * has the SA_INTERRUPT flag set. -arca
		 */
	
		/* read Pentium cycle counter */

        /*
         * 调用rdtscl将64位的TSC寄存器值中的低32位读到变量last_tsc_low中
         * 以便 do_fast_gettimeoffset()函数计算时间偏差
         * 实际作用就是将CPU中TSC寄存器的值更新到内核对TSC的缓存变量last_tsc_low
         */
		rdtscl(last_tsc_low);

		spin_lock(&i8253_lock);
		outb_p(0x00, 0x43);     /* latch the count ASAP */

		count = inb_p(0x40);    /* read the latched count */
		count |= inb(0x40) << 8;
		spin_unlock(&i8253_lock);

        /*
        * TICK_SIZE是时间滴答的时间间隔长度
        * latch：多少个周期产生一次中断
        * count：剩余可减周期数
        * ((LATCH-1) - count)一共经历了几个周期
        * ((LATCH-1) - count) * TICK_SIZE：这些周期所使用的时间间隔长度
        */
		count = ((LATCH-1) - count) * TICK_SIZE;
        /*
        * (count + LATCH/2) / LATCH：计算每个中断所使用的时间间隔长度
        * 中断服务执行延迟
        * 产生中断到内核中断服务程序是有时间延迟的
        * delay_at_last_interrupt表示该延迟
        */
		delay_at_last_interrupt = (count + LATCH/2) / LATCH;
	}
 
	do_timer_interrupt(irq, NULL, regs);

	write_unlock(&xtime_lock);

}

/* not static: needed by APM */

/*
 * 内核在启动时从RTC中读取启动时的时间与日期
 * 仅仅在内核启动时被调用一次
 */
unsigned long get_cmos_time(void)
{
	unsigned int year, mon, day, hour, min, sec;
	int i;

	spin_lock(&rtc_lock);
	/* The Linux interpretation of the CMOS clock register contents:
	 * When the Update-In-Progress (UIP) flag goes from 1 to 0, the
	 * RTC registers show the second which has precisely just started.
	 * Let's hope other operating systems interpret the RTC the same way.
	 */
	/* read RTC exactly on falling edge of update flag */
	
    /* 
     * 以下两个循环目的是在软件逻辑上同步RTC的Update Cycle
     * 第二个循环退出后RTC的Update Cycle已经结束，此时我们已经把当前时间逻辑定准在RTC的前一秒
     * 意味着此时可以从RTC中读取时间值
     */
    for (i = 0 ; i < 1000000 ; i++)	/* may take up to 1 second... */
		if (CMOS_READ(RTC_FREQ_SELECT) & RTC_UIP)
			break;
	for (i = 0 ; i < 1000000 ; i++)	/* must try at least 2.228 ms */
		if (!(CMOS_READ(RTC_FREQ_SELECT) & RTC_UIP))
			break;
	
    /*依次读取时间值，并转化成二进制*/
    do { /* Isn't this overkill ? UIP above should guarantee consistency */
		sec = CMOS_READ(RTC_SECONDS);
		min = CMOS_READ(RTC_MINUTES);
		hour = CMOS_READ(RTC_HOURS);
		day = CMOS_READ(RTC_DAY_OF_MONTH);
		mon = CMOS_READ(RTC_MONTH);
		year = CMOS_READ(RTC_YEAR);
	} while (sec != CMOS_READ(RTC_SECONDS));
	if (!(CMOS_READ(RTC_CONTROL) & RTC_DM_BINARY) || RTC_ALWAYS_BCD)
	  {
	    BCD_TO_BIN(sec);
	    BCD_TO_BIN(min);
	    BCD_TO_BIN(hour);
	    BCD_TO_BIN(day);
	    BCD_TO_BIN(mon);
	    BCD_TO_BIN(year);
	  }
	spin_unlock(&rtc_lock);
	if ((year += 1900) < 1970)
		year += 100;
	return mktime(year, mon, day, hour, min, sec);
}

static struct irqaction irq0  = { timer_interrupt, SA_INTERRUPT, 0, "timer", NULL, NULL};

/* ------ Calibrate the TSC ------- 
 * Return 2^32 * (1 / (TSC clocks per usec)) for do_fast_gettimeoffset().
 * Too much 64-bit arithmetic here to do this cleanly in C, and for
 * accuracy's sake we want to keep the overhead on the CTC speaker (channel 2)
 * output busy loop as low as possible. We avoid reading the CTC registers
 * directly because of the awkward 8-bit access mechanism of the 82C54
 * device.
 */

#define CALIBRATE_LATCH	(5 * LATCH)
#define CALIBRATE_TIME	(5 * 1000020/HZ)


/*
 * 校准TSC
 * 获得更新的fast_gettimeoffset_quotient的值
 */
static unsigned long __init calibrate_tsc(void)
{
       /* Set the Gate high, disable speaker */
	/*激活通道2，关闭扬声器*/
    outb((inb(0x61) & ~0x02) | 0x01, 0x61);

	/*
	 * Now let's take care of CTC channel 2
	 *
	 * Set the Gate high, program CTC channel 2 for mode 0,
	 * (interrupt on terminal count mode), binary count,
	 * load 5 * LATCH count, (LSB and MSB) to begin countdown.
	 */
    /*设置模式字*/
	outb(0xb0, 0x43);			/* binary, mode 0, LSB/MSB, Ch 2 */
	outb(CALIBRATE_LATCH & 0xff, 0x42);	/* LSB of count */
	outb(CALIBRATE_LATCH >> 8, 0x42);	/* MSB of count */

    /*开始校准*/
	{
		unsigned long startlow, starthigh;
		unsigned long endlow, endhigh;
		unsigned long count;

		rdtsc(startlow,starthigh);
		count = 0;
		do {
			count++;
		} while ((inb(0x61) & 0x20) == 0);
		rdtsc(endlow,endhigh);

		last_tsc_low = endlow;

		/* Error: ECTCNEVERSET */
		if (count <= 1)
			goto bad_ctc;

		/* 64-bit subtract - gcc just messes up with long longs */
		__asm__("subl %2,%0\n\t"
			"sbbl %3,%1"
			:"=a" (endlow), "=d" (endhigh)
			:"g" (startlow), "g" (starthigh),
			 "0" (endlow), "1" (endhigh));

		/* Error: ECPUTOOFAST */
		if (endhigh)
			goto bad_ctc;

		/* Error: ECPUTOOSLOW */
		if (endlow <= CALIBRATE_TIME)
			goto bad_ctc;

		__asm__("divl %2"
			:"=a" (endlow), "=d" (endhigh)
			:"r" (endlow), "0" (0), "1" (CALIBRATE_TIME));

		return endlow;
	}

	/*
	 * The CTC wasn't reliable: we got a hit on the very first read,
	 * or the CPU was so fast/slow that the quotient wouldn't fit in
	 * 32 bits..
	 */
bad_ctc:
	return 0;
}


/*
* 系统时钟初始化
*/
void __init time_init(void)
{
	extern int x86_udelay_tsc;
	
    /*把RTC中的系统启动时间日期，写到xtime.tv_sec变量中*/
	xtime.tv_sec = get_cmos_time();
	xtime.tv_usec = 0;

/*
 * If we have APM enabled or the CPU clock speed is variable
 * (CPU stops clock on HLT or slows clock to save power)
 * then the TSC timestamps may diverge by up to 1 jiffy from
 * 'real time' but nothing will break.
 * The most frequent case is that the CPU is "woken" from a halt
 * state by the timer interrupt itself, so we get 0 error. In the
 * rare cases where a driver would "wake" the CPU and request a
 * timestamp, the maximum error is < 1 jiffy. But timestamps are
 * still perfectly ordered.
 * Note that the TSC counter will be reset if APM suspends
 * to disk; this won't break the kernel, though, 'cuz we're
 * smart.  See arch/i386/kernel/apm.c.
 */
 	/*
 	 *	Firstly we have to do a CPU check for chips with
 	 * 	a potentially buggy TSC. At this point we haven't run
 	 *	the ident/bugs checks so we must run this hook as it
 	 *	may turn off the TSC flag.
 	 *
 	 *	NOTE: this doesnt yet handle SMP 486 machines where only
 	 *	some CPU's have a TSC. Thats never worked and nobody has
 	 *	moaned if you have the only one in the world - you fix it!
 	 */
    
    /*检测CPU是否存在时间戳计数器BUG*/
 	dodgy_tsc();
 	
    /*检测CPU是否存在TSC寄存器*/
	if (cpu_has_tsc) {
        /*确定每一次计数真正代表多长时间间隔，一个时钟周期的真正时间间隔长度*/
		unsigned long tsc_quotient = calibrate_tsc();
		if (tsc_quotient) {
			fast_gettimeoffset_quotient = tsc_quotient;
			use_tsc = 1;
			/*
			 *	We could be more selective here I suspect
			 *	and just enable this for the next intel chips ?
			 */
            /*表示可以通过TSC来实现微秒级的精确延时*/
			x86_udelay_tsc = 1;
#ifndef do_gettimeoffset
			do_gettimeoffset = do_fast_gettimeoffset;
#endif

			/* report CPU clock rate in Hz.
			 * The formula is (10^6 * 2^32) / (2^32 * 1 / (clocks/us)) =
			 * clock/second. Our precision is about 100 ppm.
			 */
			{	unsigned long eax=0, edx=1000;
				__asm__("divl %2"
		       		:"=a" (cpu_khz), "=d" (edx)
        	       		:"r" (tsc_quotient),
	                	"0" (eax), "1" (edx));
				printk("Detected %lu.%03lu MHz processor.\n", cpu_khz / 1000, cpu_khz % 1000);
			}
		}
	}

#ifdef CONFIG_VISWS
	printk("Starting Cobalt Timer system clock\n");

	/* Set the countdown value */
	co_cpu_write(CO_CPU_TIMEVAL, CO_TIME_HZ/HZ);

	/* Start the timer */
	co_cpu_write(CO_CPU_CTRL, co_cpu_read(CO_CPU_CTRL) | CO_CTRL_TIMERUN);

	/* Enable (unmask) the timer interrupt */
	co_cpu_write(CO_CPU_CTRL, co_cpu_read(CO_CPU_CTRL) & ~CO_CTRL_TIMEMASK);

	/* Wire cpu IDT entry to s/w handler (and Cobalt APIC to IDT) */
	/*时钟中断安装程序，将中断程序连入相应的中断请求队列
    * 等待时钟到来时，相应的中断程序被执行
    */
    setup_irq(CO_IRQ_TIMER, &irq0);
#else
	setup_irq(0, &irq0);
#endif
}
