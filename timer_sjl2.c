/*
 *  linux/kernel/timer.c
 *
 *  Kernel internal timers, kernel timekeeping, basic process system calls
 *
 *  Copyright (C) 1991, 1992  Linus Torvalds
 *
 *  1997-01-28  Modified by Finn Arne Gangstad to make timers scale better.
 *
 *  1997-09-10  Updated NTP code according to technical memorandum Jan '96
 *              "A Kernel Model for Precision Timekeeping" by Dave Mills
 *  1998-12-24  Fixed a xtime SMP race (we need the xtime_lock rw spinlock to
 *              serialize accesses to xtime/lost_ticks).
 *                              Copyright (C) 1998  Andrea Arcangeli
 *  1999-03-10  Improved NTP compatibility by Ulrich Windl
 */


//内核动态定时器
#include <linux/config.h>
#include <linux/mm.h>//内存管理头文件
#include <linux/timex.h>//计算命令的时间,报告进程数据和系统活动的头文件 
#include <linux/delay.h>//延时头文件
#include <linux/smp_lock.h>//SMP自旋锁头文件 
#include <linux/interrupt.h>//包含了与中断相关的大部分宏及struct结构的定义
#include <linux/kernel_stat.h>//包含了一些rstatd/perfmeter */使用的内核统计的所需的定义

#include <asm/uaccess.h>//用户空间内存访问头文件

/*
 * Timekeeping variables
 */

long tick = (1000000 + HZ/2) / HZ;	/* timer interrupt period 中断*/
//定时器中断周期

/* The current time */
//Linux内核通过timeval结构类型的全局变量xtime来维持当前时间
struct timeval xtime __attribute__ ((aligned (16)));
//volatile被设计用来修饰被不同线程访问和修改的变量 

/* Don't completely fail for HZ > 500.  */
int tickadj = 500/HZ ? : 1;		/* microsecs */
//时钟滴答校准（微秒） 

DECLARE_TASK_QUEUE(tq_timer);
DECLARE_TASK_QUEUE(tq_immediate);

/*
 * phase-lock loop variables
 */
/* TIME_ERROR prevents overwriting the CMOS clock */
int time_state = TIME_OK;		/* clock synchronization status	*/
int time_status = STA_UNSYNC;		/* clock status bits		*/
long time_offset;			/* time adjustment (us)		*/
long time_constant = 2;			/* pll time constant		*/
long time_tolerance = MAXFREQ;		/* frequency tolerance (ppm)	*/
long time_precision = 1;		/* clock precision (us)		*/
long time_maxerror = NTP_PHASE_LIMIT;	/* maximum error (us)		*/
long time_esterror = NTP_PHASE_LIMIT;	/* estimated error (us)		*/
long time_phase;			/* phase offset (scaled us)	*/
long time_freq = ((1000000 + HZ/2) % HZ - HZ/2) << SHIFT_USEC;
					/* frequency offset (scaled ppm)*/
long time_adj;				/* tick adjust (scaled 1 / HZ)	*/
long time_reftime;			/* time at last adjustment (s)	*/

long time_adjust;//停止正在运行的adjtime()
long time_adjust_step;

unsigned long event;

extern int do_setitimer(int, struct itimerval *, struct itimerval *);

unsigned long volatile jiffies;//从开机开始算起的滴答数时间值（10ms一次滴答）

unsigned int * prof_buffer;
unsigned long prof_len;
unsigned long prof_shift;

/*
 * Event timer code
 */
#define TVN_BITS 6//64个数组
#define TVR_BITS 8//256个数组
#define TVN_SIZE (1 << TVN_BITS)
#define TVR_SIZE (1 << TVR_BITS)
#define TVN_MASK (TVN_SIZE - 1)
#define TVR_MASK (TVR_SIZE - 1)

//定时器项链：双向循环队列
struct timer_vec {//tv2-tv5
	int index;//索引
	struct list_head vec[TVN_SIZE];//定时器向量组64
};

struct timer_vec_root {//tv1
	int index;//索引：指向正在扫描的寄存器
	struct list_head vec[TVR_SIZE];//定时器向量组256
};

static struct timer_vec tv5;//0x4000000≤interval≤0xffffffff，松散定时器向量
static struct timer_vec tv4;//0x100000≤interval≤0x3ffffff，松散定时器向量
static struct timer_vec tv3;//0x4000≤interval≤0xfffff，松散定时器向量
static struct timer_vec tv2;//0x100≤interval≤0x3fff，松散定时器向量
static struct timer_vec_root tv1;//0xff
//全局变量tv1，表示内核所关心的前256个定时器向量
//内核在处理是否有到期定时器时，
//它就只从定时器向量数组tv1.vec［256］中的某个定时器向量内进行扫描。

static struct timer_vec * const tvecs[] = {
	(struct timer_vec *)&tv1, &tv2, &tv3, &tv4, &tv5
};//指针数组，指向tv1。。。的结构变量

#define NOOF_TVECS (sizeof(tvecs) / sizeof(tvecs[0]))

void init_timervecs (void)//实现对动态定时器的初始化
{
	int i;

	for (i = 0; i < TVN_SIZE; i++) {//宏TVN_SIZE：
		//指timer_vec结构类型中的定时器向量指针数组vec［］的大小，值为64。
		INIT_LIST_HEAD(tv5.vec + i);//初始化定时器向量组
		INIT_LIST_HEAD(tv4.vec + i);//初始化定时器向量组
		INIT_LIST_HEAD(tv3.vec + i);//初始化定时器向量组
		INIT_LIST_HEAD(tv2.vec + i);//初始化定时器向量组
	}
	for (i = 0; i < TVR_SIZE; i++)//宏TVR_SIZE：
		//指timer_vec_root结构类型中的定时器向量数组vec［］的大小，值为256。
		INIT_LIST_HEAD(tv1.vec + i);//初始化定时器向量组
}

static unsigned long timer_jiffies;//表示上一次运行定时器机制时的jiffies值

static inline void internal_add_timer(struct timer_list *timer)
//将一个不处于任何定时器向量中的定时器插入到它应该所处的定时器向量中去
//根据定时器的expires值来决定
{
	/*
	 * must be cli-ed when calling this
	 */
	unsigned long expires = timer->expires;//指定定时器到期时间
	unsigned long idx = expires - timer_jiffies;//局部变量
	struct list_head * vec;
	//定时器向量的头部指针vec表示这个定时器应该所处的定时器向量链表头部。

	if (idx < TVR_SIZE) {//idx插入到那个计时器
		int i = expires & TVR_MASK;
		vec = tv1.vec + i;
	} else if (idx < 1 << (TVR_BITS + TVN_BITS)) {
		int i = (expires >> TVR_BITS) & TVN_MASK;
		vec = tv2.vec + i;
	} else if (idx < 1 << (TVR_BITS + 2 * TVN_BITS)) {
		int i = (expires >> (TVR_BITS + TVN_BITS)) & TVN_MASK;
		vec =  tv3.vec + i;
	} else if (idx < 1 << (TVR_BITS + 3 * TVN_BITS)) {
		int i = (expires >> (TVR_BITS + 2 * TVN_BITS)) & TVN_MASK;
		vec = tv4.vec + i;
	} else if ((signed long) idx < 0) {
		/* can happen if you add a timer with expires == jiffies,
		 * or you set a timer to go off in the past
		 */
		vec = tv1.vec + tv1.index;
	} else if (idx <= 0xffffffffUL) {
		int i = (expires >> (TVR_BITS + 3 * TVN_BITS)) & TVN_MASK;
		vec = tv5.vec + i;
	} else {
		/* Can only get here on architectures with 64-bit jiffies */
		INIT_LIST_HEAD(&timer->list);
		return;
	}
	/*
	 * Timers are FIFO!
	 */
	list_add(&timer->list, vec->prev);
	//将定时器插入到vec指针所指向的定时器队列的尾部。
}

/* Initialize both explicitly - let's try to have them in the same cache line */
spinlock_t timerlist_lock = SPIN_LOCK_UNLOCKED;
//内核动态定时器链表是一种系统全局共享资源，
//为了实现对它的互斥访问，Linux定义了专门的自旋锁timerlist_lock来保护
//任何想要访问动态定时器链表的代码段都首先必须先持有该自旋锁，
//并且在访问结束后释放该自旋锁。

#ifdef CONFIG_SMP
volatile struct timer_list * volatile running_timer;
#define timer_enter(t) do { running_timer = t; mb(); } while (0)
#define timer_exit() do { running_timer = NULL; } while (0)
#define timer_is_running(t) (running_timer == t)
#define timer_synchronize(t) while (timer_is_running(t)) barrier()
#else
#define timer_enter(t)		do { } while (0)
#define timer_exit()		do { } while (0)
#endif

//加timer
void add_timer(struct timer_list *timer)
//将参数timer指针所指向的定时器插入到一个合适的定时器链表中
{
	unsigned long flags;

	spin_lock_irqsave(&timerlist_lock, flags);//加锁
	if (timer_pending(timer))
		//判断所指 定的定时器是否已经位于在某个定时器向量中等待执行
		goto bug;//若已经在，bug（内核警告）
	internal_add_timer(timer);//若不在，完成实际插入操作
	spin_unlock_irqrestore(&timerlist_lock, flags);//自旋锁保护，解锁
	return;
bug:
	spin_unlock_irqrestore(&timerlist_lock, flags);//解锁
	printk("bug: kernel timer added twice at %p.\n",//内核警告
			__builtin_return_address(0));
}

//将定时器从某个链表中删除
static inline int detach_timer (struct timer_list *timer)
{
	if (!timer_pending(timer))
		//判断所指 定的定时器是否已经位于在某个定时器向量中等待执行
		return 0;//原本就不处于任何链表
	list_del(&timer->list);//将定时器从它原来所处的链表中摘除
	return 1;
}

//改timer
//修改一个定时器的expires值
//当一个定时器已经被插入到内核动态定时器链表中后，我们还可以修改该定时器的expires值
int mod_timer(struct timer_list *timer, unsigned long expires)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&timerlist_lock, flags);//lock
	timer->expires = expires;//更新定时器的expires
	ret = detach_timer(timer);//将该定时器从它原来所属的链表中删除
	internal_add_timer(timer);//将该定时器根据它新的expires值重新插入到相应的链表中
	spin_unlock_irqrestore(&timerlist_lock, flags);//解锁
	return ret;
}

//删除timer
//将一个定时器从相应的内核定时器队列中删除
//实际上是对detach_timer()函数的高层封装
int del_timer(struct timer_list * timer)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&timerlist_lock, flags);//上锁
	ret = detach_timer(timer);//从链表中删除
	timer->list.next = timer->list.prev = NULL;//修改指针
	spin_unlock_irqrestore(&timerlist_lock, flags);//解锁
	return ret;
}

#ifdef CONFIG_SMP
void sync_timers(void)
{
	spin_unlock_wait(&global_bh_lock);
}

/*
 * SMP specific function to delete periodic timer.
 * Caller must disable by some means restarting the timer
 * for new. Upon exit the timer is not queued and handler is not running
 * on any CPU. It returns number of times, which timer was deleted
 * (for reference counting).
 */

int del_timer_sync(struct timer_list * timer)
{
	int ret = 0;

	for (;;) {
		unsigned long flags;
		int running;

		spin_lock_irqsave(&timerlist_lock, flags);
		ret += detach_timer(timer);
		timer->list.next = timer->list.prev = 0;
		running = timer_is_running(timer);
		spin_unlock_irqrestore(&timerlist_lock, flags);

		if (!running)
			break;

		timer_synchronize(timer);
	}

	return ret;
}
#endif

//定时器迁移
//jiffies值增大，定时器的interval值减小
//迁移到上一个tv
static inline void cascade_timers(struct timer_vec *tv)
{
	/* cascade all the timers from tv up one level */
	struct list_head *head, *curr, *next;

	head = tv->vec + tv->index;//头指针指向第几个数组
	curr = head->next;//指针curr指向定时器向量中的第一个定时器
	/*
	 * We are removing _all_ timers from the list, so we don't  have to
	 * detach them individually, just clear the list afterwards.
	 */
	//全删除
	while (curr != head) {//循环中之队列，因为双向循环队列，终止条件：curr=head
		struct timer_list *tmp;//指针

		tmp = list_entry(curr, struct timer_list, list);//第几个定时器
		next = curr->next;//指针后移
		list_del(curr); // not needed？
		//循环体都先调用list_del()函数把当前定时器从链表中摘除
		internal_add_timer(tmp); //重新确定该定时器应该被放到哪个定时器向量中去
		curr = next;//指针后移
	}
	//while循环结束后，定时器向量tv->vec［tv->index］所有定时器已经被迁移，为空
	INIT_LIST_HEAD(head);//调用INIT_LIST_HEAD()宏来把定时器向量的表头结构初始化为空
	tv->index = (tv->index + 1) & TVN_MASK;//偏移量+1
}

//扫描并执行当前已经到期的定时器
static inline void run_timer_list(void)
{
	spin_lock_irq(&timerlist_lock);//上锁
	while ((long)(jiffies - timer_jiffies) >= 0) {
		//timer_jiffies：表示了内核上一次执行run_timer_list()函数的时间
		//差值就表示了自从上一次处理定时器以来，期间一共发生了多少次时钟中断
		struct list_head *head, *curr;
		if (!tv1.index) {//判断tv1.index是否为0
			//为零，从后面补充寄存器
			//寄存器迁移
			int n = 1;
			do {
				cascade_timers(tvecs[n]);
			} while (tvecs[n]->index == 1 && ++n < NOOF_TVECS);//？
		}
repeat://执行定时器向量tv1.vec［tv1.index］中的所有到期定时器
		head = tv1.vec + tv1.index;
		curr = head->next;
		if (curr != head) {
			struct timer_list *timer;
			void (*fn)(unsigned long);
			unsigned long data;

			timer = list_entry(curr, struct timer_list, list);
 			fn = timer->function;
 			data= timer->data;

			detach_timer(timer);//把当前定时器从对列中摘除后，可执行开关锁
			timer->list.next = timer->list.prev = NULL;
			timer_enter(timer);
			spin_unlock_irq(&timerlist_lock);//解锁
			fn(data);
			spin_lock_irq(&timerlist_lock);//上锁
			timer_exit();
			goto repeat;
		}
		++timer_jiffies; 
		tv1.index = (tv1.index + 1) & TVR_MASK;
	}
	spin_unlock_irq(&timerlist_lock);//解锁
}

spinlock_t tqueue_lock = SPIN_LOCK_UNLOCKED;

void tqueue_bh(void)
{
	run_task_queue(&tq_timer);
}

void immediate_bh(void)
{
	run_task_queue(&tq_immediate);
}

/*
 * this routine handles the overflow of the microsecond field
 *
 * The tricky bits of code to handle the accurate clock support
 * were provided by Dave Mills (Mills@UDEL.EDU) of NTP fame.
 * They were originally developed for SUN and DEC kernels.
 * All the kudos should go to Dave for this stuff.
 *
 */
static void second_overflow(void)//处理微秒数成员溢出的情况 
{
    long ltemp;

    /* Bump the maxerror field */
    time_maxerror += time_tolerance >> SHIFT_USEC;
    if ( time_maxerror > NTP_PHASE_LIMIT ) {
	time_maxerror = NTP_PHASE_LIMIT;
	time_status |= STA_UNSYNC;
    }

    /*
     * Leap second processing. If in leap-insert state at
     * the end of the day, the system clock is set back one
     * second; if in leap-delete state, the system clock is
     * set ahead one second. The microtime() routine or
     * external clock driver will insure that reported time
     * is always monotonic. The ugly divides should be
     * replaced.
     */
    switch (time_state) {

    case TIME_OK:
	if (time_status & STA_INS)
	    time_state = TIME_INS;
	else if (time_status & STA_DEL)
	    time_state = TIME_DEL;
	break;

    case TIME_INS:
	if (xtime.tv_sec % 86400 == 0) {
	    xtime.tv_sec--;
	    time_state = TIME_OOP;
	    printk(KERN_NOTICE "Clock: inserting leap second 23:59:60 UTC\n");
	}
	break;

    case TIME_DEL:
	if ((xtime.tv_sec + 1) % 86400 == 0) {
	    xtime.tv_sec++;
	    time_state = TIME_WAIT;
	    printk(KERN_NOTICE "Clock: deleting leap second 23:59:59 UTC\n");
	}
	break;

    case TIME_OOP:
	time_state = TIME_WAIT;
	break;

    case TIME_WAIT:
	if (!(time_status & (STA_INS | STA_DEL)))
	    time_state = TIME_OK;
    }

    /*
     * Compute the phase adjustment for the next second. In
     * PLL mode, the offset is reduced by a fixed factor
     * times the time constant. In FLL mode the offset is
     * used directly. In either mode, the maximum phase
     * adjustment for each second is clamped so as to spread
     * the adjustment over not more than the number of
     * seconds between updates.
     */
    if (time_offset < 0) {
	ltemp = -time_offset;
	if (!(time_status & STA_FLL))
	    ltemp >>= SHIFT_KG + time_constant;
	if (ltemp > (MAXPHASE / MINSEC) << SHIFT_UPDATE)
	    ltemp = (MAXPHASE / MINSEC) << SHIFT_UPDATE;
	time_offset += ltemp;
	time_adj = -ltemp << (SHIFT_SCALE - SHIFT_HZ - SHIFT_UPDATE);
    } else {
	ltemp = time_offset;
	if (!(time_status & STA_FLL))
	    ltemp >>= SHIFT_KG + time_constant;
	if (ltemp > (MAXPHASE / MINSEC) << SHIFT_UPDATE)
	    ltemp = (MAXPHASE / MINSEC) << SHIFT_UPDATE;
	time_offset -= ltemp;
	time_adj = ltemp << (SHIFT_SCALE - SHIFT_HZ - SHIFT_UPDATE);
    }

    /*
     * Compute the frequency estimate and additional phase
     * adjustment due to frequency error for the next
     * second. When the PPS signal is engaged, gnaw on the
     * watchdog counter and update the frequency computed by
     * the pll and the PPS signal.
     */
    pps_valid++;
    if (pps_valid == PPS_VALID) {	/* PPS signal lost */
	pps_jitter = MAXTIME;
	pps_stabil = MAXFREQ;
	time_status &= ~(STA_PPSSIGNAL | STA_PPSJITTER |
			 STA_PPSWANDER | STA_PPSERROR);
    }
    ltemp = time_freq + pps_freq;
    if (ltemp < 0)
	time_adj -= -ltemp >>
	    (SHIFT_USEC + SHIFT_HZ - SHIFT_SCALE);
    else
	time_adj += ltemp >>
	    (SHIFT_USEC + SHIFT_HZ - SHIFT_SCALE);

#if HZ == 100
    /* Compensate for (HZ==100) != (1 << SHIFT_HZ).
     * Add 25% and 3.125% to get 128.125; => only 0.125% error (p. 14)
     */
    if (time_adj < 0)
	time_adj -= (-time_adj >> 2) + (-time_adj >> 5);
    else
	time_adj += (time_adj >> 2) + (time_adj >> 5);
#endif
}

/* in the NTP reference this is called "hardclock()" */
static void update_wall_time_one_tick(void)
{//更新一次时钟滴答对系统全局时间xtime的影响
	if ( (time_adjust_step = time_adjust) != 0 ) {//校准
	    /* We are doing an adjtime thing. 
	     *
	     * Prepare time_adjust_step to be within bounds.
	     * Note that a positive time_adjust means we want the clock
	     * to run faster.
	     *
	     * Limit the amount of the step to be in the range
	     * -tickadj .. +tickadj
	     */
	     if (time_adjust > tickadj)
		time_adjust_step = tickadj;
	     else if (time_adjust < -tickadj)
		time_adjust_step = -tickadj;
	     
	    /* Reduce by this step the amount of time left  */
	    time_adjust -= time_adjust_step;
	}
	xtime.tv_usec += tick + time_adjust_step;
	/*
	 * Advance the phase, once it gets to one microsecond, then
	 * advance the tick more.
	 */
	time_phase += time_adj;
	if (time_phase <= -FINEUSEC) {
		long ltemp = -time_phase >> SHIFT_SCALE;
		time_phase += ltemp << SHIFT_SCALE;
		xtime.tv_usec -= ltemp;
	}
	else if (time_phase >= FINEUSEC) {
		long ltemp = time_phase >> SHIFT_SCALE;
		time_phase -= ltemp << SHIFT_SCALE;
		xtime.tv_usec += ltemp;
	}
}

/*
 * Using a loop looks inefficient, but "ticks" is
 * usually just one (we shouldn't be losing ticks,
 * we're doing this this way mainly for interrupt
 * latency reasons, not because we think we'll
 * have lots of lost timer ticks
 */
static void update_wall_time(unsigned long ticks)
{//更新记录当前的wall_clock时间的xtime 
	do {//为每一个必须要处理的tick调用update_wall_time_one_tick() 
		ticks--;
		update_wall_time_one_tick();//更新全局变量xtime
	} while (ticks);

	if (xtime.tv_usec >= 1000000) {//已经过了一秒钟
	    xtime.tv_usec -= 1000000;//微秒数成员tv_usec减去百万微秒
	    xtime.tv_sec++;//秒数成员tv_sec的值加1
	    second_overflow();//处理微秒数成员溢出的情况
	}
}

static inline void do_process_times(struct task_struct *p,
	unsigned long user, unsigned long system)
{//更新指定进程的总时间统计信息 
	unsigned long psecs;//psecs指定进程p到目前为止已经运行的总时间长度

	psecs = (p->times.tms_utime += user);
	psecs += (p->times.tms_stime += system);
	if (psecs / HZ > p->rlim[RLIMIT_CPU].rlim_cur) {
		/* Send SIGXCPU every second.. */
		if (!(psecs % HZ))//总运行时间长超过进程P的资源限额 
			send_sig(SIGXCPU, p, 1);//每隔1秒给进程发送一个信号SIGXCPU 
		/* and SIGKILL when we go over max.. */
		if (psecs / HZ > p->rlim[RLIMIT_CPU].rlim_max)
			//运行时间长度超过了进程资源限额的最大值
			send_sig(SIGKILL, p, 1);//发送一个SIGKILL信号杀死该进程 
	}
}

static inline void do_it_virt(struct task_struct * p, unsigned long ticks)
{//为进程的用户态执行时间ITIMER_VIRTUAL软件定时器更新时间间隔
	unsigned long it_virt = p->it_virt_value;

	if (it_virt) {
		it_virt -= ticks;
		if (!it_virt) {//定时器到时 
			it_virt = p->it_virt_incr;//重置初值 
			send_sig(SIGVTALRM, p, 1);//发送信号SIGVTALRM
		}
		p->it_virt_value = it_virt;//定时计数器减去时钟滴答数
	}
}

static inline void do_it_prof(struct task_struct *p)
{//为进程的ITIMER_PROF软件定时器更新时间间隔
	unsigned long it_prof = p->it_prof_value;

	if (it_prof) {
		if (--it_prof == 0) {//定时器到时
			it_prof = p->it_prof_incr;//重置初值 
			send_sig(SIGPROF, p, 1);//发送信号SIGPROF 
		}
		p->it_prof_value = it_prof;//定时计数器减去时钟滴答数
	}
}

void update_one_process(struct task_struct *p, unsigned long user,
			unsigned long system, int cpu)
{//更新当前进程中的所有与时间相关的统计信息以及成员变量，
	//视需要向当前进程发送相应的信号
	p->per_cpu_utime[cpu] += user;//更新进程在当前CPU上的  用户态  执行时间统计
	p->per_cpu_stime[cpu] += system;//更新进程在当前CPU上的  核心态  执行时间统计 
	do_process_times(p, user, system);//更新当前进程的总时间统计
	do_it_virt(p, user);//为当前进程的ITIMER_VIRTUAL软件定时器更新时间间隔 
	do_it_prof(p);//为当前进程的ITIMER_PROF软件定时器更新时间间隔 
}	

/*
 * Called from the timer interrupt handler to charge one tick to the current 
 * process.  user_tick is 1 if the tick is user time, 0 for system.
 */
void update_process_times(int user_tick)
{//在发生时钟中断时更新当前进程以及内核中与时间相关的统计信息，并作出相应的动作
	struct task_struct *p = current;
	int cpu = smp_processor_id(), system = user_tick ^ 1;

	update_one_process(p, user_tick, system, cpu);
	//更新进程中的与时间相关的统计信息及成员变量
	
	if (p->pid) {
		if (--p->counter <= 0) {//当前进程时间片用完
			p->counter = 0;//重置时间片为0
			p->need_resched = 1;//重新调度进程
		}
		if (p->nice > 0)//进程的时间片小于系统的进程基本时间片
			kstat.per_cpu_nice[cpu] += user_tick;
			//将use值加到内核全局统计信息变量cpu_nice上

		else//进程的时间片大于系统的进程基本时间片 
			kstat.per_cpu_user[cpu] += user_tick;
			//将use值加到内核全局统计信息变量cpu_user上
		kstat.per_cpu_system[cpu] += system;//将system变量值加到内核全局统计信息上
	} else if (local_bh_count(cpu) || local_irq_count(cpu) > 1)
		kstat.per_cpu_system[cpu] += system;
}

/*
 * Nr of active tasks - counted in fixed-point numbers
 */
static unsigned long count_active_tasks(void)
{
	struct task_struct *p;
	unsigned long nr = 0;

	read_lock(&tasklist_lock);
	for_each_task(p) {
		if ((p->state == TASK_RUNNING ||
		     (p->state & TASK_UNINTERRUPTIBLE)))
			nr += FIXED_1;
	}
	read_unlock(&tasklist_lock);
	return nr;
}

/*
 * Hmm.. Changed this, as the GNU make sources (load.c) seems to
 * imply that avenrun[] is the standard name for this kind of thing.
 * Nothing else seems to be standardized: the fractional size etc
 * all seem to differ on different machines.
 */
unsigned long avenrun[3];

static inline void calc_load(unsigned long ticks)//更新内核有关系统负载要素的估计值
{
	unsigned long active_tasks; /* fixed-point */
	static int count = LOAD_FREQ;

	count -= ticks;
	if (count < 0) {
		count += LOAD_FREQ;
		active_tasks = count_active_tasks();
		CALC_LOAD(avenrun[0], EXP_1, active_tasks);
		CALC_LOAD(avenrun[1], EXP_5, active_tasks);
		CALC_LOAD(avenrun[2], EXP_15, active_tasks);
	}
}

/* jiffies at the most recent update of wall time */
unsigned long wall_jiffies;

/*
 * This spinlock protect us from races in SMP while playing with xtime. -arca
 */
rwlock_t xtime_lock = RW_LOCK_UNLOCKED;

static inline void update_times(void)//更新系统的平均负载、记录当前时间的全局变量 
{//更新内核的当前进程使用的cpu时间的估计值 
	unsigned long ticks;

	/*
	 * update_times() is run from the raw timer_bh handler so we
	 * just know that the irqs are locally enabled and so we don't
	 * need to save/restore the flags of the local CPU here. -arca
	 */
	write_lock_irq(&xtime_lock);//关闭全局时间xtime的读锁，准备更新xtime 

	ticks = jiffies - wall_jiffies;//取得上次下半部分运行以来发生的定时器滴答的数目
	if (ticks) {//如果定时器滴答数目非空
		wall_jiffies += ticks;
		update_wall_time(ticks);//更新xtime
	}
	write_unlock_irq(&xtime_lock);//xtime更新完毕，开xtime的读锁 
	calc_load(ticks);//更新内核有关系统负载要素的估计值
}

void timer_bh(void)
{//定时器的下半部分
	update_times();//更新内核和进程有关时间的统计数字
	run_timer_list();//处理动态定时器
}

void do_timer(struct pt_regs *regs)
{//定时器的上半部分
	(*(unsigned long *)&jiffies)++;//更新记录了及其启动以来系统时钟滴答的次数jiffies
#ifndef CONFIG_SMP
	/* SMP process accounting uses the local APIC timer */

	update_process_times(user_mode(regs));//发生时钟中断时，更新当前进程相关的所有信息
#endif
	mark_bh(TIMER_BH);//定时器下半部分标记为只要可能就运行 
	if (TQ_ACTIVE(tq_timer))//定时器队列中有任务在等待
		mark_bh(TQUEUE_BH);//定时器队列的下半部分被标记为准备好运行
}

#if !defined(__alpha__) && !defined(__ia64__)

/*
 * For backwards compatibility?  This can be done in libc so Alpha
 * and all newer ports shouldn't need it.
 */
asmlinkage unsigned long sys_alarm(unsigned int seconds)
{//启动进程的ITIMER_REAL间隔定时器 
	struct itimerval it_new, it_old;
	unsigned int oldalarm;

	it_new.it_interval.tv_sec = it_new.it_interval.tv_usec = 0;//ITIMER_REAL间隔定时器是一次性
	it_new.it_value.tv_sec = seconds;//根据参数seconds的值构造一个itimerval结构变量it_new
	it_new.it_value.tv_usec = 0;
	do_setitimer(ITIMER_REAL, &it_new, &it_old);//以it_new启动ITIMER_REAL,将原定时间隔保存到it_old 
	oldalarm = it_old.it_value.tv_sec;//oldalarm表示以秒数计的ITIMER_REAL间隔定时器的原定时间隔值 
	/* ehhh.. We can't return 0 if we have an alarm pending.. */
	/* And we'd better return too much than too little anyway */
	if (it_old.it_value.tv_usec)
		oldalarm++;//不足1秒补足1秒
	return oldalarm;//返回原定时间隔值 
}

#endif

#ifndef __alpha__

/*
 * The Alpha uses getxpid, getxuid, and getxgid instead.  Maybe this
 * should be moved into arch/i386 instead?
 */
 
asmlinkage long sys_getpid(void)
{//取当前进程号pid 
	/* This is SMP safe - current->pid doesn't change */
	return current->tgid;
}

/*
 * This is not strictly SMP safe: p_opptr could change
 * from under us. However, rather than getting any lock
 * we can use an optimistic algorithm: get the parent
 * pid, and go back and check that the parent is still
 * the same. If it has changed (which is extremely unlikely
 * indeed), we just try again..
 *
 * NOTE! This depends on the fact that even if we _do_
 * get an old value of "parent", we can happily dereference
 * the pointer: we just can't necessarily trust the result
 * until we know that the parent pointer is valid.
 *
 * The "mb()" macro is a memory barrier - a synchronizing
 * event. It also makes sure that gcc doesn't optimize
 * away the necessary memory references.. The barrier doesn't
 * have to have all that strong semantics: on x86 we don't
 * really require a synchronizing instruction, for example.
 * The barrier is more important for code generation than
 * for any real memory ordering semantics (even if there is
 * a small window for a race, using the old pointer is
 * harmless for a while).
 */
asmlinkage long sys_getppid(void)
{//取父进程号ppid
	int pid;
	struct task_struct * me = current;
	struct task_struct * parent;

	parent = me->p_opptr;//当前进程的父进程指针赋给parent 
	for (;;) {//无限循环 
		pid = parent->pid;//获取父进程id
#if CONFIG_SMP
{//检查父进程是否被更改 
		struct task_struct *old = parent;
		mb();//防止内存优化的同步机制
		parent = me->p_opptr;
		if (old != parent)//父进程被更改 
			continue;//继续获取父进程id
}
#endif
		break;
	}
	return pid;//返回父进程pid
}

asmlinkage long sys_getuid(void)
{//取用户号uid 
	/* Only we change this so SMP safe */
	return current->uid;
}

asmlinkage long sys_geteuid(void)
{//取有效用户号euid
	/* Only we change this so SMP safe */
	return current->euid;
}

asmlinkage long sys_getgid(void)
{//取组号gid 
	/* Only we change this so SMP safe */
	return current->gid;
}

asmlinkage long sys_getegid(void)
{//取有效组号egid
	/* Only we change this so SMP safe */
	return  current->egid;
}

#endif

/* Thread ID - the internal kernel "pid" */
asmlinkage long sys_gettid(void)
{//线程pid
	return current->pid;
}

asmlinkage long sys_nanosleep(struct timespec *rqtp, struct timespec *rmtp)
{//系统调用，让当前进程睡眠一段时间后继续运行
	struct timespec t;
	unsigned long expire;

	if(copy_from_user(&t, rqtp, sizeof(struct timespec)))
		return -EFAULT;

	if (t.tv_nsec >= 1000000000L || t.tv_nsec < 0 || t.tv_sec < 0)
		return -EINVAL;


	if (t.tv_sec == 0 && t.tv_nsec <= 2000000L &&
	    current->policy != SCHED_OTHER)
	{
		/*
		 * Short delay requests up to 2 ms will be handled with
		 * high precision by a busy wait for all real-time processes.
		 *
		 * Its important on SMP not to do this holding locks.
		 */
		udelay((t.tv_nsec + 999) / 1000);
		return 0;
	}

	expire = timespec_to_jiffies(&t) + (t.tv_sec || t.tv_nsec);

	current->state = TASK_INTERRUPTIBLE;
	expire = schedule_timeout(expire);

	if (expire) {
		if (rmtp) {
			jiffies_to_timespec(expire, &t);
			if (copy_to_user(rmtp, &t, sizeof(struct timespec)))
				return -EFAULT;
		}
		return -EINTR;
	}
	return 0;
}

