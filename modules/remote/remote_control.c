/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"
#include "message_center.h"
#include "logger.h"

#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data
RC_ctrl_t rc_ctrl;

//receive data, 18 bytes one frame, but set 36 bytes
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
// frame counter
static volatile uint32_t rc_frame_count = 0;
// keep a snapshot of last 18-byte SBUS frame for debugging
static uint8_t last_sbus_frame[RC_FRAME_LENGTH];

uint32_t RC_GetFrameCount(void)
{
  return rc_frame_count;
}

void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
    LOG_INFO(LOG_TAG_RC, "Remote control initialized, waiting for SBUS data on USART3...");
}

void RC_GetLastFrame(uint8_t out[RC_FRAME_LENGTH])
{
    if (!out) return;
    memcpy(out, last_sbus_frame, RC_FRAME_LENGTH);
}

const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

void REMOTE_USART3_IDLE_IRQHandler(void)
{
    static uint32_t irq_count = 0;
    static uint32_t last_log = 0;

    if(huart3.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        irq_count++;

        // Log every 500 interrupts (reduce spam)
        uint32_t now = HAL_GetTick();
        if (irq_count % 500 == 1 && (now - last_log) > 2000) {
            LOG_DEBUG(LOG_TAG_RC, "USART3 IRQ triggered %lu times", (unsigned long)irq_count);
            last_log = now;
        }

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                memcpy(last_sbus_frame, (const void*)sbus_rx_buf[0], RC_FRAME_LENGTH);
                rc_frame_count++;
                (void)MsgCenter_Publish(TOPIC_RC_UPDATE, &rc_ctrl, sizeof(rc_ctrl));

                // Log first frame reception (simple INFO message OK in ISR)
                if (rc_frame_count == 1) {
                    LOG_INFO(LOG_TAG_RC, "First SBUS frame received! RC link active.");
                }
                // NOTE: Don't log CSV data in ISR - let main loop handle it via subscription
            }
            else if (irq_count % 500 == 1 && (now - last_log) > 2000)
            {
                // Log unexpected frame length
                LOG_DEBUG(LOG_TAG_RC, "Wrong frame length: %u (expected %u)",
                          (unsigned int)this_time_rx_len, (unsigned int)RC_FRAME_LENGTH);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
                memcpy(last_sbus_frame, (const void*)sbus_rx_buf[1], RC_FRAME_LENGTH);
                rc_frame_count++;
                (void)MsgCenter_Publish(TOPIC_RC_UPDATE, &rc_ctrl, sizeof(rc_ctrl));

                // Log first frame reception (simple INFO message OK in ISR)
                if (rc_frame_count == 1) {
                    LOG_INFO(LOG_TAG_RC, "First SBUS frame received! RC link active.");
                }
                // NOTE: Don't log CSV data in ISR - let main loop handle it via subscription
            }
            else if (irq_count % 500 == 1 && (now - last_log) > 2000)
            {
                // Log unexpected frame length
                LOG_DEBUG(LOG_TAG_RC, "Wrong frame length: %u (expected %u)",
                          (unsigned int)this_time_rx_len, (unsigned int)RC_FRAME_LENGTH);
            }
        }
    }
}

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);
    rc_ctrl->mouse.press_l = sbus_buf[12];
    rc_ctrl->mouse.press_r = sbus_buf[13];
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);
    rc_ctrl->rc.ch[4] = (sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07ff;

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}


