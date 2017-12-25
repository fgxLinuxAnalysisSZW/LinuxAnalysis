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


//�ں˶�̬��ʱ��
#include <linux/config.h>
#include <linux/mm.h>//�ڴ����ͷ�ļ�
#include <linux/timex.h>//���������ʱ��,����������ݺ�ϵͳ���ͷ�ļ� 
#include <linux/delay.h>//��ʱͷ�ļ�
#include <linux/smp_lock.h>//SMP������ͷ�ļ� 
#include <linux/interrupt.h>//���������ж���صĴ󲿷ֺ꼰struct�ṹ�Ķ���
#include <linux/kernel_stat.h>//������һЩrstatd/perfmeter */ʹ�õ��ں�ͳ�Ƶ�����Ķ���

#include <asm/uaccess.h>//�û��ռ��ڴ����ͷ�ļ�

/*
 * Timekeeping variables
 */

long tick = (1000000 + HZ/2) / HZ;	/* timer interrupt period �ж�*/
//��ʱ���ж�����

/* The current time */
//Linux�ں�ͨ��timeval�ṹ���͵�ȫ�ֱ���xtime��ά�ֵ�ǰʱ��
struct timeval xtime __attribute__ ((aligned (16)));
//volatile������������α���ͬ�̷߳��ʺ��޸ĵı��� 

/* Don't completely fail for HZ > 500.  */
int tickadj = 500/HZ ? : 1;		/* microsecs */
//ʱ�ӵδ�У׼��΢�룩 

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

long time_adjust;//ֹͣ�������е�adjtime()
long time_adjust_step;

unsigned long event;

extern int do_setitimer(int, struct itimerval *, struct itimerval *);

unsigned long volatile jiffies;//�ӿ�����ʼ����ĵδ���ʱ��ֵ��10msһ�εδ�

unsigned int * prof_buffer;
unsigned long prof_len;
unsigned long prof_shift;

/*
 * Event timer code
 */
#define TVN_BITS 6//64������
#define TVR_BITS 8//256������
#define TVN_SIZE (1 << TVN_BITS)
#define TVR_SIZE (1 << TVR_BITS)
#define TVN_MASK (TVN_SIZE - 1)
#define TVR_MASK (TVR_SIZE - 1)

//��ʱ��������˫��ѭ������
struct timer_vec {//tv2-tv5
	int index;//����
	struct list_head vec[TVN_SIZE];//��ʱ��������64
};

struct timer_vec_root {//tv1
	int index;//������ָ������ɨ��ļĴ���
	struct list_head vec[TVR_SIZE];//��ʱ��������256
};

static struct timer_vec tv5;//0x4000000��interval��0xffffffff����ɢ��ʱ������
static struct timer_vec tv4;//0x100000��interval��0x3ffffff����ɢ��ʱ������
static struct timer_vec tv3;//0x4000��interval��0xfffff����ɢ��ʱ������
static struct timer_vec tv2;//0x100��interval��0x3fff����ɢ��ʱ������
static struct timer_vec_root tv1;//0xff
//ȫ�ֱ���tv1����ʾ�ں������ĵ�ǰ256����ʱ������
//�ں��ڴ����Ƿ��е��ڶ�ʱ��ʱ��
//����ֻ�Ӷ�ʱ����������tv1.vec��256���е�ĳ����ʱ�������ڽ���ɨ�衣

static struct timer_vec * const tvecs[] = {
	(struct timer_vec *)&tv1, &tv2, &tv3, &tv4, &tv5
};//ָ�����飬ָ��tv1�������Ľṹ����

#define NOOF_TVECS (sizeof(tvecs) / sizeof(tvecs[0]))

void init_timervecs (void)//ʵ�ֶԶ�̬��ʱ���ĳ�ʼ��
{
	int i;

	for (i = 0; i < TVN_SIZE; i++) {//��TVN_SIZE��
		//ָtimer_vec�ṹ�����еĶ�ʱ������ָ������vec�ۣݵĴ�С��ֵΪ64��
		INIT_LIST_HEAD(tv5.vec + i);//��ʼ����ʱ��������
		INIT_LIST_HEAD(tv4.vec + i);//��ʼ����ʱ��������
		INIT_LIST_HEAD(tv3.vec + i);//��ʼ����ʱ��������
		INIT_LIST_HEAD(tv2.vec + i);//��ʼ����ʱ��������
	}
	for (i = 0; i < TVR_SIZE; i++)//��TVR_SIZE��
		//ָtimer_vec_root�ṹ�����еĶ�ʱ����������vec�ۣݵĴ�С��ֵΪ256��
		INIT_LIST_HEAD(tv1.vec + i);//��ʼ����ʱ��������
}

static unsigned long timer_jiffies;//��ʾ��һ�����ж�ʱ������ʱ��jiffiesֵ

static inline void internal_add_timer(struct timer_list *timer)
//��һ���������κζ�ʱ�������еĶ�ʱ�����뵽��Ӧ�������Ķ�ʱ��������ȥ
//���ݶ�ʱ����expiresֵ������
{
	/*
	 * must be cli-ed when calling this
	 */
	unsigned long expires = timer->expires;//ָ����ʱ������ʱ��
	unsigned long idx = expires - timer_jiffies;//�ֲ�����
	struct list_head * vec;
	//��ʱ��������ͷ��ָ��vec��ʾ�����ʱ��Ӧ�������Ķ�ʱ����������ͷ����

	if (idx < TVR_SIZE) {//idx���뵽�Ǹ���ʱ��
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
	//����ʱ�����뵽vecָ����ָ��Ķ�ʱ�����е�β����
}

/* Initialize both explicitly - let's try to have them in the same cache line */
spinlock_t timerlist_lock = SPIN_LOCK_UNLOCKED;
//�ں˶�̬��ʱ��������һ��ϵͳȫ�ֹ�����Դ��
//Ϊ��ʵ�ֶ����Ļ�����ʣ�Linux������ר�ŵ�������timerlist_lock������
//�κ���Ҫ���ʶ�̬��ʱ������Ĵ���ζ����ȱ����ȳ��и���������
//�����ڷ��ʽ������ͷŸ���������

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

//��timer
void add_timer(struct timer_list *timer)
//������timerָ����ָ��Ķ�ʱ�����뵽һ�����ʵĶ�ʱ��������
{
	unsigned long flags;

	spin_lock_irqsave(&timerlist_lock, flags);//����
	if (timer_pending(timer))
		//�ж���ָ ���Ķ�ʱ���Ƿ��Ѿ�λ����ĳ����ʱ�������еȴ�ִ��
		goto bug;//���Ѿ��ڣ�bug���ں˾��棩
	internal_add_timer(timer);//�����ڣ����ʵ�ʲ������
	spin_unlock_irqrestore(&timerlist_lock, flags);//����������������
	return;
bug:
	spin_unlock_irqrestore(&timerlist_lock, flags);//����
	printk("bug: kernel timer added twice at %p.\n",//�ں˾���
			__builtin_return_address(0));
}

//����ʱ����ĳ��������ɾ��
static inline int detach_timer (struct timer_list *timer)
{
	if (!timer_pending(timer))
		//�ж���ָ ���Ķ�ʱ���Ƿ��Ѿ�λ����ĳ����ʱ�������еȴ�ִ��
		return 0;//ԭ���Ͳ������κ�����
	list_del(&timer->list);//����ʱ������ԭ��������������ժ��
	return 1;
}

//��timer
//�޸�һ����ʱ����expiresֵ
//��һ����ʱ���Ѿ������뵽�ں˶�̬��ʱ�������к����ǻ������޸ĸö�ʱ����expiresֵ
int mod_timer(struct timer_list *timer, unsigned long expires)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&timerlist_lock, flags);//lock
	timer->expires = expires;//���¶�ʱ����expires
	ret = detach_timer(timer);//���ö�ʱ������ԭ��������������ɾ��
	internal_add_timer(timer);//���ö�ʱ���������µ�expiresֵ���²��뵽��Ӧ��������
	spin_unlock_irqrestore(&timerlist_lock, flags);//����
	return ret;
}

//ɾ��timer
//��һ����ʱ������Ӧ���ں˶�ʱ��������ɾ��
//ʵ�����Ƕ�detach_timer()�����ĸ߲��װ
int del_timer(struct timer_list * timer)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&timerlist_lock, flags);//����
	ret = detach_timer(timer);//��������ɾ��
	timer->list.next = timer->list.prev = NULL;//�޸�ָ��
	spin_unlock_irqrestore(&timerlist_lock, flags);//����
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

//��ʱ��Ǩ��
//jiffiesֵ���󣬶�ʱ����intervalֵ��С
//Ǩ�Ƶ���һ��tv
static inline void cascade_timers(struct timer_vec *tv)
{
	/* cascade all the timers from tv up one level */
	struct list_head *head, *curr, *next;

	head = tv->vec + tv->index;//ͷָ��ָ��ڼ�������
	curr = head->next;//ָ��currָ��ʱ�������еĵ�һ����ʱ��
	/*
	 * We are removing _all_ timers from the list, so we don't  have to
	 * detach them individually, just clear the list afterwards.
	 */
	//ȫɾ��
	while (curr != head) {//ѭ����֮���У���Ϊ˫��ѭ�����У���ֹ������curr=head
		struct timer_list *tmp;//ָ��

		tmp = list_entry(curr, struct timer_list, list);//�ڼ�����ʱ��
		next = curr->next;//ָ�����
		list_del(curr); // not needed��
		//ѭ���嶼�ȵ���list_del()�����ѵ�ǰ��ʱ����������ժ��
		internal_add_timer(tmp); //����ȷ���ö�ʱ��Ӧ�ñ��ŵ��ĸ���ʱ��������ȥ
		curr = next;//ָ�����
	}
	//whileѭ�������󣬶�ʱ������tv->vec��tv->index�����ж�ʱ���Ѿ���Ǩ�ƣ�Ϊ��
	INIT_LIST_HEAD(head);//����INIT_LIST_HEAD()�����Ѷ�ʱ�������ı�ͷ�ṹ��ʼ��Ϊ��
	tv->index = (tv->index + 1) & TVN_MASK;//ƫ����+1
}

//ɨ�貢ִ�е�ǰ�Ѿ����ڵĶ�ʱ��
static inline void run_timer_list(void)
{
	spin_lock_irq(&timerlist_lock);//����
	while ((long)(jiffies - timer_jiffies) >= 0) {
		//timer_jiffies����ʾ���ں���һ��ִ��run_timer_list()������ʱ��
		//��ֵ�ͱ�ʾ���Դ���һ�δ���ʱ���������ڼ�һ�������˶��ٴ�ʱ���ж�
		struct list_head *head, *curr;
		if (!tv1.index) {//�ж�tv1.index�Ƿ�Ϊ0
			//Ϊ�㣬�Ӻ��油��Ĵ���
			//�Ĵ���Ǩ��
			int n = 1;
			do {
				cascade_timers(tvecs[n]);
			} while (tvecs[n]->index == 1 && ++n < NOOF_TVECS);//��
		}
repeat://ִ�ж�ʱ������tv1.vec��tv1.index���е����е��ڶ�ʱ��
		head = tv1.vec + tv1.index;
		curr = head->next;
		if (curr != head) {
			struct timer_list *timer;
			void (*fn)(unsigned long);
			unsigned long data;

			timer = list_entry(curr, struct timer_list, list);
 			fn = timer->function;
 			data= timer->data;

			detach_timer(timer);//�ѵ�ǰ��ʱ���Ӷ�����ժ���󣬿�ִ�п�����
			timer->list.next = timer->list.prev = NULL;
			timer_enter(timer);
			spin_unlock_irq(&timerlist_lock);//����
			fn(data);
			spin_lock_irq(&timerlist_lock);//����
			timer_exit();
			goto repeat;
		}
		++timer_jiffies; 
		tv1.index = (tv1.index + 1) & TVR_MASK;
	}
	spin_unlock_irq(&timerlist_lock);//����
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
static void second_overflow(void)//����΢������Ա�������� 
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
{//����һ��ʱ�ӵδ��ϵͳȫ��ʱ��xtime��Ӱ��
	if ( (time_adjust_step = time_adjust) != 0 ) {//У׼
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
{//���¼�¼��ǰ��wall_clockʱ���xtime 
	do {//Ϊÿһ������Ҫ�����tick����update_wall_time_one_tick() 
		ticks--;
		update_wall_time_one_tick();//����ȫ�ֱ���xtime
	} while (ticks);

	if (xtime.tv_usec >= 1000000) {//�Ѿ�����һ����
	    xtime.tv_usec -= 1000000;//΢������Աtv_usec��ȥ����΢��
	    xtime.tv_sec++;//������Աtv_sec��ֵ��1
	    second_overflow();//����΢������Ա��������
	}
}

static inline void do_process_times(struct task_struct *p,
	unsigned long user, unsigned long system)
{//����ָ�����̵���ʱ��ͳ����Ϣ 
	unsigned long psecs;//psecsָ������p��ĿǰΪֹ�Ѿ����е���ʱ�䳤��

	psecs = (p->times.tms_utime += user);
	psecs += (p->times.tms_stime += system);
	if (psecs / HZ > p->rlim[RLIMIT_CPU].rlim_cur) {
		/* Send SIGXCPU every second.. */
		if (!(psecs % HZ))//������ʱ�䳤��������P����Դ�޶� 
			send_sig(SIGXCPU, p, 1);//ÿ��1������̷���һ���ź�SIGXCPU 
		/* and SIGKILL when we go over max.. */
		if (psecs / HZ > p->rlim[RLIMIT_CPU].rlim_max)
			//����ʱ�䳤�ȳ����˽�����Դ�޶�����ֵ
			send_sig(SIGKILL, p, 1);//����һ��SIGKILL�ź�ɱ���ý��� 
	}
}

static inline void do_it_virt(struct task_struct * p, unsigned long ticks)
{//Ϊ���̵��û�ִ̬��ʱ��ITIMER_VIRTUAL�����ʱ������ʱ����
	unsigned long it_virt = p->it_virt_value;

	if (it_virt) {
		it_virt -= ticks;
		if (!it_virt) {//��ʱ����ʱ 
			it_virt = p->it_virt_incr;//���ó�ֵ 
			send_sig(SIGVTALRM, p, 1);//�����ź�SIGVTALRM
		}
		p->it_virt_value = it_virt;//��ʱ��������ȥʱ�ӵδ���
	}
}

static inline void do_it_prof(struct task_struct *p)
{//Ϊ���̵�ITIMER_PROF�����ʱ������ʱ����
	unsigned long it_prof = p->it_prof_value;

	if (it_prof) {
		if (--it_prof == 0) {//��ʱ����ʱ
			it_prof = p->it_prof_incr;//���ó�ֵ 
			send_sig(SIGPROF, p, 1);//�����ź�SIGPROF 
		}
		p->it_prof_value = it_prof;//��ʱ��������ȥʱ�ӵδ���
	}
}

void update_one_process(struct task_struct *p, unsigned long user,
			unsigned long system, int cpu)
{//���µ�ǰ�����е�������ʱ����ص�ͳ����Ϣ�Լ���Ա������
	//����Ҫ��ǰ���̷�����Ӧ���ź�
	p->per_cpu_utime[cpu] += user;//���½����ڵ�ǰCPU�ϵ�  �û�̬  ִ��ʱ��ͳ��
	p->per_cpu_stime[cpu] += system;//���½����ڵ�ǰCPU�ϵ�  ����̬  ִ��ʱ��ͳ�� 
	do_process_times(p, user, system);//���µ�ǰ���̵���ʱ��ͳ��
	do_it_virt(p, user);//Ϊ��ǰ���̵�ITIMER_VIRTUAL�����ʱ������ʱ���� 
	do_it_prof(p);//Ϊ��ǰ���̵�ITIMER_PROF�����ʱ������ʱ���� 
}	

/*
 * Called from the timer interrupt handler to charge one tick to the current 
 * process.  user_tick is 1 if the tick is user time, 0 for system.
 */
void update_process_times(int user_tick)
{//�ڷ���ʱ���ж�ʱ���µ�ǰ�����Լ��ں�����ʱ����ص�ͳ����Ϣ����������Ӧ�Ķ���
	struct task_struct *p = current;
	int cpu = smp_processor_id(), system = user_tick ^ 1;

	update_one_process(p, user_tick, system, cpu);
	//���½����е���ʱ����ص�ͳ����Ϣ����Ա����
	
	if (p->pid) {
		if (--p->counter <= 0) {//��ǰ����ʱ��Ƭ����
			p->counter = 0;//����ʱ��ƬΪ0
			p->need_resched = 1;//���µ��Ƚ���
		}
		if (p->nice > 0)//���̵�ʱ��ƬС��ϵͳ�Ľ��̻���ʱ��Ƭ
			kstat.per_cpu_nice[cpu] += user_tick;
			//��useֵ�ӵ��ں�ȫ��ͳ����Ϣ����cpu_nice��

		else//���̵�ʱ��Ƭ����ϵͳ�Ľ��̻���ʱ��Ƭ 
			kstat.per_cpu_user[cpu] += user_tick;
			//��useֵ�ӵ��ں�ȫ��ͳ����Ϣ����cpu_user��
		kstat.per_cpu_system[cpu] += system;//��system����ֵ�ӵ��ں�ȫ��ͳ����Ϣ��
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

static inline void calc_load(unsigned long ticks)//�����ں��й�ϵͳ����Ҫ�صĹ���ֵ
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

static inline void update_times(void)//����ϵͳ��ƽ�����ء���¼��ǰʱ���ȫ�ֱ��� 
{//�����ں˵ĵ�ǰ����ʹ�õ�cpuʱ��Ĺ���ֵ 
	unsigned long ticks;

	/*
	 * update_times() is run from the raw timer_bh handler so we
	 * just know that the irqs are locally enabled and so we don't
	 * need to save/restore the flags of the local CPU here. -arca
	 */
	write_lock_irq(&xtime_lock);//�ر�ȫ��ʱ��xtime�Ķ�����׼������xtime 

	ticks = jiffies - wall_jiffies;//ȡ���ϴ��°벿���������������Ķ�ʱ���δ����Ŀ
	if (ticks) {//�����ʱ���δ���Ŀ�ǿ�
		wall_jiffies += ticks;
		update_wall_time(ticks);//����xtime
	}
	write_unlock_irq(&xtime_lock);//xtime������ϣ���xtime�Ķ��� 
	calc_load(ticks);//�����ں��й�ϵͳ����Ҫ�صĹ���ֵ
}

void timer_bh(void)
{//��ʱ�����°벿��
	update_times();//�����ں˺ͽ����й�ʱ���ͳ������
	run_timer_list();//����̬��ʱ��
}

void do_timer(struct pt_regs *regs)
{//��ʱ�����ϰ벿��
	(*(unsigned long *)&jiffies)++;//���¼�¼�˼�����������ϵͳʱ�ӵδ�Ĵ���jiffies
#ifndef CONFIG_SMP
	/* SMP process accounting uses the local APIC timer */

	update_process_times(user_mode(regs));//����ʱ���ж�ʱ�����µ�ǰ������ص�������Ϣ
#endif
	mark_bh(TIMER_BH);//��ʱ���°벿�ֱ��ΪֻҪ���ܾ����� 
	if (TQ_ACTIVE(tq_timer))//��ʱ���������������ڵȴ�
		mark_bh(TQUEUE_BH);//��ʱ�����е��°벿�ֱ����Ϊ׼��������
}

#if !defined(__alpha__) && !defined(__ia64__)

/*
 * For backwards compatibility?  This can be done in libc so Alpha
 * and all newer ports shouldn't need it.
 */
asmlinkage unsigned long sys_alarm(unsigned int seconds)
{//�������̵�ITIMER_REAL�����ʱ�� 
	struct itimerval it_new, it_old;
	unsigned int oldalarm;

	it_new.it_interval.tv_sec = it_new.it_interval.tv_usec = 0;//ITIMER_REAL�����ʱ����һ����
	it_new.it_value.tv_sec = seconds;//���ݲ���seconds��ֵ����һ��itimerval�ṹ����it_new
	it_new.it_value.tv_usec = 0;
	do_setitimer(ITIMER_REAL, &it_new, &it_old);//��it_new����ITIMER_REAL,��ԭ��ʱ������浽it_old 
	oldalarm = it_old.it_value.tv_sec;//oldalarm��ʾ�������Ƶ�ITIMER_REAL�����ʱ����ԭ��ʱ���ֵ 
	/* ehhh.. We can't return 0 if we have an alarm pending.. */
	/* And we'd better return too much than too little anyway */
	if (it_old.it_value.tv_usec)
		oldalarm++;//����1�벹��1��
	return oldalarm;//����ԭ��ʱ���ֵ 
}

#endif

#ifndef __alpha__

/*
 * The Alpha uses getxpid, getxuid, and getxgid instead.  Maybe this
 * should be moved into arch/i386 instead?
 */
 
asmlinkage long sys_getpid(void)
{//ȡ��ǰ���̺�pid 
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
{//ȡ�����̺�ppid
	int pid;
	struct task_struct * me = current;
	struct task_struct * parent;

	parent = me->p_opptr;//��ǰ���̵ĸ�����ָ�븳��parent 
	for (;;) {//����ѭ�� 
		pid = parent->pid;//��ȡ������id
#if CONFIG_SMP
{//��鸸�����Ƿ񱻸��� 
		struct task_struct *old = parent;
		mb();//��ֹ�ڴ��Ż���ͬ������
		parent = me->p_opptr;
		if (old != parent)//�����̱����� 
			continue;//������ȡ������id
}
#endif
		break;
	}
	return pid;//���ظ�����pid
}

asmlinkage long sys_getuid(void)
{//ȡ�û���uid 
	/* Only we change this so SMP safe */
	return current->uid;
}

asmlinkage long sys_geteuid(void)
{//ȡ��Ч�û���euid
	/* Only we change this so SMP safe */
	return current->euid;
}

asmlinkage long sys_getgid(void)
{//ȡ���gid 
	/* Only we change this so SMP safe */
	return current->gid;
}

asmlinkage long sys_getegid(void)
{//ȡ��Ч���egid
	/* Only we change this so SMP safe */
	return  current->egid;
}

#endif

/* Thread ID - the internal kernel "pid" */
asmlinkage long sys_gettid(void)
{//�߳�pid
	return current->pid;
}

asmlinkage long sys_nanosleep(struct timespec *rqtp, struct timespec *rmtp)
{//ϵͳ���ã��õ�ǰ����˯��һ��ʱ����������
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

