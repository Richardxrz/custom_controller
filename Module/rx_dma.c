/**
 * @file rx_dma.c
 * @brief 本文件用于构建接收系统
 * @note rx_dma 的职责：把串口的字节流，变成「一帧一帧的数据」，交给业务层。
 *
 * 三层结构：
 *     rx_    入口：DMA 收件箱（硬件写，回调读）
 *     sm_    加工：状态机（只有 Sm_Feed 碰）
 *     out_   出口：成品帧（Sm_Feed 写，rx_dma_fetch 读）*
 *
 * @note 下面是关于接收系统状态机描述的状态机表
 *
 * | 当前状态 | 事件 | 条件 | 下一状态 | 动作 |
 * | -------- | ---- | ------------- | ------ | --------- |
 * | 等帧头1 | 收到字节 | 字节1等于正确帧头 | 等帧头2 | 无 |
 * | 等帧头1 | 收到字节 | 其他 | 等帧头1  | 丢弃刚才收到的字节 |
 * | 等帧头2 | 收到字节 | 字节2等于正确的帧头2 | 等数据长度 | 无 |
 * | 等帧头2 | 收到字节 | 其他 | 等帧头2 | 丢弃刚才收到的字节 |
 * | 等数据长度 | 收到字节 | 字节3的数值表示长度LEN为1～16 | 等数据 | rx_len接收该位数据 |
 * | 等数据长度 | 收到字节 | 字节3的数值不属于LEN的有效范围 | 等数据长度 | 无 |
 * | 等数据 | 收到字节 | length < rx_len | 等数据 |
 * 用length来记录已经接收的数据长度，同时data[]逐位存储字节 | | 等数据 | 收到字节 ｜length == rx_len
 * | 等帧头1 |
 * 拿这一位的数据计算校验和并进行比对判数据是否有效，有效则保留无效则抛弃，如果有效则提取完整序列中的数据帧
 * 交接给发送（因为这个版本中不对数据进行任何处理否则交接给数据处理），然后交接完成后将变量全部重置，
 * 用于下次接收|
 */

#include "rx_dma.h"
#include <string.h>

#define RX_INSTANCE_MAX 2
#define UART_DMA_RX_TIMEOUT_MS 100

typedef enum {
  SM_WAIT_HEAD1,
  SM_WAIT_HEAD2,
  SM_WAIT_LENGTH,
  SM_WAIT_DATA,
} SmState_t;

struct RxDma_t {
  UART_HandleTypeDef *huart;
  uint8_t rx_raw[RX_RAW_SIZE];
  SmState_t sm_state;
  uint8_t sm_len, sm_idx, sm_sum, sm_sum_recv;
  uint8_t sm_payload[SM_PAYLOAD_MAX];
  uint8_t out_buf[SM_PAYLOAD_MAX];
  uint8_t volatile out_len;
  uint32_t last_byte_ms;
  uint16_t rx_err_cnt;
  uint8_t DATA_length;
};

static struct RxDma_t s_instances[RX_INSTANCE_MAX];
static uint8_t s_instances_cnt = 0;

static void rx_start(RxDma_t self);
static void Proto_Sm_Feed(RxDma_t self, uint8_t byte);
static void Proto_Sm_Reset(RxDma_t self);
static SmState_t GetSmState(RxDma_t self);

// rx_dma 私有函数定义
/**
 * @brief  重置状态机内部变量
 * @param  实例结构体指针
 * @retval none
 */
static void Proto_Sm_Reset(RxDma_t self) {
  self->sm_len = 0;
  for (int i = 0; i < self->sm_idx; i++) {
    self->sm_payload[i] = 0;
  }
  self->sm_idx = 0;
  self->sm_sum = 0;
  self->sm_sum_recv = 0;
  self->sm_state = SM_WAIT_HEAD1;
}

/**
 * @brief  将接收到的字节流拆分成字节投喂给状态机
 * @param  实例结构体指针
 * @param  接收到的字节数
 * @retval none
 */
static void RxDmaCB_Feed(RxDma_t self, uint16_t Size) {
  for (int i = 0; i < Size; i++) {
    Proto_Sm_Feed(self, self->rx_raw[i]);
  }
}

/**
 * @brief  接收协议解析
 *         同时将解析出来的各个位的数据存
 * @param  实例结构体指针
 * @param  接收到的字节
 * @retval none
 */
static void Proto_Sm_Feed(RxDma_t self, uint8_t sm_byte) {
  switch (self->sm_state) {
  case SM_WAIT_HEAD1:
    // 如果收到字节等于正确的帧头1
    if (sm_byte == 0xAA) {
      // 不进行任何动作直接进入下一状态
      self->sm_state = SM_WAIT_HEAD2;
    } // 如果收到的字节等于其他
    else {
      // 不做任何动作继续等待帧头1
    }
    break;
  case SM_WAIT_HEAD2:
    // 如果收到的字节等于帧头2
    if (sm_byte == 0x55) {
      // 不做人动作进入下一状态
      self->sm_state = SM_WAIT_LENGTH;
    } // 如果收到字节等于其他
    else {
      // 不做任何动作继续等待帧头2
    }
    break;
  case SM_WAIT_LENGTH:
    // 如果收到的字节在有效长度内
    if (sm_byte >= 1 && sm_byte <= 16) {
      // 存储该字节
      self->sm_len = sm_byte;
      // 同时计算校验和
      self->sm_sum += sm_byte;
      // 将数据暴露给外部
      self->DATA_length = sm_byte;
      // 然后进入下一状态
      self->sm_state = SM_WAIT_DATA;
    } // 如果收到的字节不在有效长度内
    else {
      // 不做任何动作继续等待
    }
    break;
  case SM_WAIT_DATA:
    // 如果接收字节不够s_rx_len
    if (self->sm_idx < self->sm_len) { // s_rx_index = 0,1,2
      // 逐位存取字节
      self->sm_payload[self->sm_idx++] = sm_byte; // s_rx_index = 1,2,3
      // 存取过程中计算校验和
      self->sm_sum += sm_byte;
    } // 如果接收字节够s_rx_len
    else if (self->sm_idx == self->sm_len) {
      // 存储收到的校验和
      self->sm_sum_recv = sm_byte;
      // 如果校验和检验没通过
      if (self->sm_sum != self->sm_sum_recv) {
        self->rx_err_cnt++;
        // 重置接收变量
        Proto_Sm_Reset(self);
      } // 如果通过校验和检验则保留
      else {
        // 不对数据进行处理直接交接数据给输出层
        memcpy(self->out_buf, self->sm_payload, self->sm_len);
        self->out_len = self->sm_len;
        // 交接后就重置变量
        Proto_Sm_Reset(self);
      };
    }
    break;
  default:
    /* Error */
    break;
  }
}

/**
 * @brief   这是一个用来处理以下需求的接收回调函数
 *          1. 多实例
 *          2. 超时检测
 *          3. 错误帧计数
 *          4. 状态机重置
 *          5. 状态机投喂
 *          6. 开启接收
 *          7. 存储收到的帧长度（字节）
 * @param   串口句柄和接收到的帧长度(由HAL自动传递)
 * @note    只有在空闲触发的回调里对超时进行判定，同时满足状态机没有喂完才会认定超时
 * @reteval none
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  for (int i = 0; i < s_instances_cnt; i++) {
    if (s_instances[i].huart == huart) {
      if (huart->RxEventType == HAL_UART_RXEVENT_IDLE) {
        if (HAL_GetTick() - s_instances[i].last_byte_ms > UART_DMA_RX_TIMEOUT_MS &&
            GetSmState(&s_instances[i]) != SM_WAIT_HEAD1) {
          s_instances[i].rx_err_cnt++;
          Proto_Sm_Reset(&s_instances[i]);
        }
        s_instances[i].last_byte_ms = HAL_GetTick();
      } else if (huart->RxEventType == HAL_UART_RXEVENT_TC ||
                 huart->RxEventType == HAL_UART_RXEVENT_HT) {
        s_instances[i].last_byte_ms = HAL_GetTick();
      }
      if (huart->RxEventType == HAL_UART_RXEVENT_IDLE ||
          huart->RxEventType == HAL_UART_RXEVENT_TC) {
        RxDmaCB_Feed(&s_instances[i], Size);
      }
      rx_start(&s_instances[i]);
      return;
    }
  }
}

/**
 * @brief  开启接收中断
 * @param  实例结构体指针
 * @retval none
 */
static void rx_start(RxDma_t self) {
  HAL_UARTEx_ReceiveToIdle_DMA(self->huart, self->rx_raw, RX_RAW_SIZE);
}
/**
 * @brief  获取接收协议解析状态机的当前状态
 * @praam  实例结构体指针
 * @retval 状态机当前状态
 */
static SmState_t GetSmState(RxDma_t self) { return self->sm_state; }

// rx_dma 公有函数定义

RxDma_t RxDma_open(UART_HandleTypeDef *huart) {
  RxDma_t out;

  if (s_instances_cnt < RX_INSTANCE_MAX) {
    out = &s_instances[s_instances_cnt++];
  } else {
    out = NULL;
    return out;
  }

  out->huart = huart;
  out->sm_state = SM_WAIT_HEAD1; // 初始化状态
  out->out_len = 0;
  Proto_Sm_Reset(out); // 清内部变量
  out->last_byte_ms = 0;

  rx_start(out); // 启动第一次接收

  return out;
}

// TODO: 添加环形缓冲
uint16_t RxDma_fetch(RxDma_t self, uint8_t *out, uint16_t max_len) {
  if (self->out_len == 0) {
    return 0;
  }

  __disable_irq(); /* 短临界区：防止拷到一半被新帧覆盖 */
  uint16_t n = self->out_len;
  if (n > max_len)
    n = max_len;
  memcpy(out, self->out_buf, n);
  for (int i = 0; i < self->out_len; i++) {
    self->out_buf[i] = 0;
  }
  self->out_len = 0; /* 取走了 */
  __enable_irq();

  return n;
}

uint8_t RxDma_GetErrCnt(RxDma_t self) { return self->rx_err_cnt; }
uint8_t RxDma_GetDATALength(RxDma_t self) { return self->DATA_length; }
