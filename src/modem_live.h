#ifndef MODEM_LIVE_H_
#define MODEM_LIVE_H_

#include <stdint.h>
#include "app_config.h"

typedef enum {
    MODEM_STATE_OFF = 0,
    MODEM_STATE_BOOTING,
    MODEM_STATE_LINKING,
    MODEM_STATE_READY,
    MODEM_STATE_SENDING,
    MODEM_STATE_RECOVERING
} modem_state_t;

/* Internal FSM (Modem_Poll) — exposed for Live Expressions. */
typedef enum {
    MODEM_LIVE_FSM_HW_RESET = 0,
    MODEM_LIVE_FSM_INIT_AT,
    MODEM_LIVE_FSM_CHECK_NET,
    MODEM_LIVE_FSM_IDLE,
    MODEM_LIVE_FSM_RECOVERY_WAIT
} modem_live_fsm_t;

typedef struct {
    modem_state_t state;
    modem_live_fsm_t fsm;       /* 0=HW_RESET 1=INIT_AT 2=CHECK_NET 3=IDLE 4=RECOVERY_WAIT */
    int8_t  csq;
    int8_t  cereg;
    uint8_t pdp_active;
    uint8_t socket_open;
    uint8_t ready;
    uint8_t send_pending;       /* 1 = payload waiting in UdpSendNow queue */
    uint8_t send_retry_count;   /* outer fails; reopen path at >= MODEM_SEND_RETRY_MAX (3) */
    uint16_t send_backoff_ms;   /* added to MODEM_SEND_PACE_MS (100) between attempts */
    int16_t last_qiopen_err;
    int16_t last_cme;
    uint32_t qisend_ok;
    uint32_t qisend_fail;
    uint32_t prompt_miss;
    uint32_t prompt_err;
    uint32_t send_fail;
    uint32_t sendok_timeout;
    uint32_t reopen_count;
    uint32_t recovery_count;
    uint32_t rf_restart_count;
    uint32_t ue_reset_count;    /* AT+CFUN=1,1 module reboots (rate limited) */
    uint32_t dma_rearm_count;
    uint32_t at_ok_count;
    uint32_t at_fail_count;
    uint32_t prompt_poll_hit; /* legacy: poll path removed; stays 0 */
    int8_t   cfun;            /* last AT+CFUN? ; 1=full, 0=min (radio off!) */
    uint8_t  cpin_ok;         /* 1 = +CPIN: READY seen */
    uint8_t  warm_skip_qpowd; /* 1 = boot left modem on (Final26 warm flash) */
    uint16_t rx_fill;           /* modem_rx_head */
    uint16_t rx_tail;           /* modem_rx_tail */
    uint16_t dma_old_pos;       /* last pumped DMA write index */
    uint16_t last_send_len;
    uint8_t  rx_dma_circular; /* 1 = USART3 RX DMA circular (expected) */
    uint8_t  uart_rx_busy;    /* 1 = huart->RxState == BUSY_RX (healthy while linked) */
    uint16_t vbat_mv;         /* AT+CBC — VBAT measured at the module, mV (Vmin 3400) */
    uint16_t vbat_min_mv;     /* lowest AT+CBC reading since boot */
    int8_t   airplane_ctl;    /* AT+QCFG="airplanecontrol": 1 = W_DISABLE# pin armed */
    int8_t   airplane_status; /* 1 = module currently held in airplane mode */
    char last_line[48];
    char ceer[40];            /* AT+CEER — network release/reject cause text */
    char oper[24];            /* AT+COPS? — serving operator */
} modem_live_t;

#if APP_MODEM_DEBUG_LOG
#define MODEM_LOG_LINES 8
#define MODEM_LOG_LEN   48
extern char modem_log[MODEM_LOG_LINES][MODEM_LOG_LEN];
#endif

extern volatile modem_live_t modem_live;
extern volatile uint32_t last_modem_rx_tick;

#endif /* MODEM_LIVE_H_ */
