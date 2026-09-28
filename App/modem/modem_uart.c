#include "modem_internal.h"
#include "main.h"

extern DMA_HandleTypeDef hdma_usart3_rx;

UART_HandleTypeDef *modem_huart;
osMutexId_t modem_buffer_mutex;

volatile char modem_rx_ring[MODEM_RING_SIZE];
volatile uint16_t modem_rx_head;
volatile uint16_t modem_rx_tail;
volatile uint8_t modem_dma_buf[MODEM_DMA_RX_SIZE];
volatile uint16_t modem_dma_old_pos;
volatile uint32_t modem_dma_restart_err_cnt;

void Modem_RingPushIsr(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((modem_rx_head + 1u) % MODEM_RING_SIZE);
        if (next != modem_rx_tail) {
            modem_rx_ring[modem_rx_head] = (char)data[i];
            modem_rx_head = next;
        }
    }
}

void Modem_RingFlush(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    modem_rx_tail = modem_rx_head;
    __set_PRIMASK(primask);
}

uint8_t Modem_RingContains(const char *needle)
{
    uint16_t nlen = (uint16_t)strlen(needle);
    if (nlen == 0) return 1;
    uint16_t end = modem_rx_head;
    uint16_t start = modem_rx_tail;
    while (start != end) {
        uint16_t k = start, j = 0;
        while (j < nlen && k != end && modem_rx_ring[k] == needle[j]) {
            j++;
            k = (uint16_t)((k + 1u) % MODEM_RING_SIZE);
        }
        if (j == nlen) return 1;
        start = (uint16_t)((start + 1u) % MODEM_RING_SIZE);
    }
    return 0;
}

void Modem_ArmRxDma(void)
{
    /* Soft rearm like Final26 Gsm_ArmRxDma. Mid-session HAL_DMA_DeInit was
     * paired with WaitPromptPoll and left USART3 RX dead after a few QISENDs.
     * Do not call this while RX is already BUSY_RX+circular (see EnsureRxAlive). */
    modem_dma_old_pos = 0;

    if (hdma_usart3_rx.Init.Mode != DMA_CIRCULAR ||
        hdma_usart3_rx.Instance != DMA1_Channel2) {
        HAL_UART_DMAStop(modem_huart);
        (void)HAL_DMA_DeInit(&hdma_usart3_rx);
        hdma_usart3_rx.Instance = DMA1_Channel2;
        hdma_usart3_rx.Init.Request = DMA_REQUEST_USART3_RX;
        hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;
        (void)HAL_DMA_Init(&hdma_usart3_rx);
        __HAL_LINKDMA(modem_huart, hdmarx, hdma_usart3_rx);
    }

    modem_huart->RxState = HAL_UART_STATE_READY;
    if (HAL_UARTEx_ReceiveToIdle_DMA(modem_huart, (uint8_t *)modem_dma_buf, MODEM_DMA_RX_SIZE) != HAL_OK) {
        __HAL_UART_CLEAR_FLAG(modem_huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        HAL_UART_DMAStop(modem_huart);
        modem_huart->RxState = HAL_UART_STATE_READY;
        (void)HAL_UARTEx_ReceiveToIdle_DMA(modem_huart, (uint8_t *)modem_dma_buf, MODEM_DMA_RX_SIZE);
        modem_dma_restart_err_cnt++;
    }

    modem_live.rx_dma_circular =
        (uint8_t)(hdma_usart3_rx.Init.Mode == DMA_CIRCULAR ? 1u : 0u);
}

void Modem_EnsureRxAlive(void)
{
    if (modem_huart == NULL) return;
    if (modem_huart->RxState == HAL_UART_STATE_BUSY_RX) return;

    /* Rate-limit: at most one rearm / 250 ms. Prefer ForceRxArm so we do not
     * zero dma_old_pos (ArmRxDma would drop bytes already in the DMA buf). */
    static uint32_t last_rearm_tick;
    uint32_t now = HAL_GetTick();
    if ((now - last_rearm_tick) < 250u) return;
    last_rearm_tick = now;
    Modem_ForceRxArm();
}

/* Only when RxState != BUSY_RX. Final26 zeros dma_old_pos on every rearm —
 * preserving old_pos after ReceiveToIdle restart desyncs the circular buffer. */
void Modem_ForceRxArm(void)
{
    if (modem_huart == NULL) return;
    if (modem_huart->RxState == HAL_UART_STATE_BUSY_RX) return;
    Modem_ArmRxDma();
    modem_live.dma_rearm_count++;
}

/* IDLE callback beklemeden DMA yazma konumunu ring'e aktar.
 * QISEND `>` tek byte; IDLE gecikmesi prompt_miss üretir. */
void Modem_RxPump(void)
{
    if (modem_huart == NULL || modem_huart->hdmarx == NULL) return;
    uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(modem_huart->hdmarx);
    uint16_t pos;
    if (remaining >= MODEM_DMA_RX_SIZE) {
        pos = 0;
    } else {
        pos = (uint16_t)(MODEM_DMA_RX_SIZE - remaining);
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Modem_OnUartRxEvent(pos);
    __set_PRIMASK(primask);
}

void Modem_SnapshotRing(char *out, uint16_t out_len)
{
    if (!out || out_len == 0) return;
    memset(out, 0, out_len);
    uint16_t i = modem_rx_tail;
    uint16_t n = 0;
    while (i != modem_rx_head && n + 1u < out_len) {
        char c = modem_rx_ring[i];
        out[n++] = (c >= 32 && c < 127) ? c : '.';
        i = (uint16_t)((i + 1u) % MODEM_RING_SIZE);
    }
    out[n] = '\0';
}

void Modem_UartBringup(void)
{
    HAL_UART_Abort(modem_huart);
    __HAL_UNLOCK(modem_huart);
    modem_huart->gState = HAL_UART_STATE_READY;
    modem_huart->RxState = HAL_UART_STATE_READY;
    modem_huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    HAL_UART_Init(modem_huart);
    __HAL_UART_CLEAR_FLAG(modem_huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    modem_rx_head = 0;
    modem_rx_tail = 0;
    Modem_ArmRxDma();
}

void Modem_OnUartRxEvent(uint16_t size)
{
    if (size > 0) {
        last_modem_rx_tick = HAL_GetTick();
    }
    uint16_t old = modem_dma_old_pos;
    if (size != old) {
        if (size > old) {
            Modem_RingPushIsr(&modem_dma_buf[old], (uint16_t)(size - old));
        } else {
            Modem_RingPushIsr(&modem_dma_buf[old], (uint16_t)(MODEM_DMA_RX_SIZE - old));
            if (size > 0) {
                Modem_RingPushIsr(modem_dma_buf, size);
            }
        }
        modem_dma_old_pos = size;
    }
}

void Modem_OnUartRxIdleDone(void)
{
    /* Final26: no-op. Circular DMA must not be restarted from IDLE callback. */
}

void Modem_OnUartError(void)
{
    Telemetry_SetError(ERR_UART3_RUNTIME);
    modem_live.dma_rearm_count++;
    HAL_UART_DMAStop(modem_huart);
    __HAL_UART_CLEAR_FLAG(modem_huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    Modem_EventPush(MODEM_EVT_UART_ERROR);
    Modem_ArmRxDma();
}

uint8_t Modem_ExtractLine(char *out, uint16_t out_len)
{
    uint16_t temp_tail = modem_rx_tail;
    uint16_t i = 0;
    while (temp_tail != modem_rx_head) {
        char c = modem_rx_ring[temp_tail];
        if (i + 1 < out_len) {
            out[i++] = c;
        }
        temp_tail = (uint16_t)((temp_tail + 1u) % MODEM_RING_SIZE);
        if (c == '\n' || c == '\r') {
            if (c == '\r' && temp_tail != modem_rx_head && modem_rx_ring[temp_tail] == '\n') {
                temp_tail = (uint16_t)((temp_tail + 1u) % MODEM_RING_SIZE);
            }
            out[i] = '\0';
            modem_rx_tail = temp_tail;
            return 1;
        }
        if (i >= out_len - 1u) {
            modem_rx_tail = temp_tail;
            break;
        }
    }
    return 0;
}
