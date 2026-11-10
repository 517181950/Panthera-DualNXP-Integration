/* SPDX-License-Identifier: Apache-2.0 */

#ifndef INCLUDE_WIRELINK_TYPE_H_
#define INCLUDE_WIRELINK_TYPE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

typedef int32_t wl_err_t;
enum {
  WL_OK = 0, /**< 成功 (Success) */

  /* --- 通用系统级错误 (-1 ~ -9) --- */
  WL_ERR_INVALID_ARG = -1,     /**< 无效的参数 (例如空指针、越界数值) */
  WL_ERR_NOT_INITIALIZED = -2, /**< 模块未初始化 */
  WL_ERR_NOT_SUPPORTED = -3,   /**< 不支持的功能/未知的指令 ID */

  /* --- 缓冲区与内存错误 (-10 ~ -19) --- */
  /* WL_ERR_NO_MEM is reserved for allocation failures, not a full buffer. */
  WL_ERR_NO_MEM = -10, /**< 内存分配失败 */
  WL_ERR_BUF_TOO_SMALL =
      -11, /**< 目标缓冲区太小，无法容纳序列化/反序列化后的数据 */
  WL_ERR_NO_SPACE = -12, /**< 缓冲区没有足够的连续可写空间 */
  WL_ERR_NO_DATA = -13,  /**< 缓冲区的可读数据不足 */

  /* --- 协议与分帧错误 (COBS/Framing) --- */
  WL_ERR_BAD_FRAME = -20,   /**< 帧格式错误 (无法找到包头包尾，或包结构非法) */
  WL_ERR_COBS_DECODE = -21, /**< COBS 解码失败 (数据在传输中可能损坏) */
  WL_ERR_CRC = -22,         /**< CRC 校验失败 (数据完整性校验未通过) */
  WL_ERR_FRAME_TOO_LONG =
      -23, /**< 帧长度超限 (超过了 BipBuffer 或单包的最大限制) */

  /* --- 可靠传输与状态机错误 (ARQ/Transport) (-30 ~ -39) --- */
  WL_ERR_BUSY =
      -30, /**< 设备忙 (例如停等协议中，前一个 Reliable 包还未收到 ACK) */
  WL_ERR_TIMEOUT = -31,       /**< 可靠传输超时 (未在规定时间内收到 ACK) */
  WL_ERR_TX_FAILED = -32,     /**< 传输失败 (重试次数已达最大上限) */
  WL_ERR_INVALID_STATE = -33, /**< 状态机处于非法状态，无法执行此操作 */
  WL_ERR_WOULD_BLOCK = -34, /**< 操作暂时无法完成，稍后重试 */
  WL_ERR_QUEUE_FULL = -35,  /**< 队列/槽位已满 */
  WL_ERR_NOT_FOUND = -36,   /**< 找不到对象（句柄/令牌） */
  WL_ERR_PAYLOAD_TOO_LONG = -37, /**< 载荷长度超过协议上限 */
  WL_ERR_IO = -38,               /**< 发送路径发生确定性 I/O 失败 */
  WL_ERR_CANCELLED = -39,         /**< 事务已取消 */
  WL_ERR_PROTOCOL_VERSION = -40,  /**< 帧版本不兼容 */
  WL_ERR_REENTRANT = -41,        /**< 回调重入或并发访问 */
  WL_ERR_ID_EXHAUSTED = -42,     /**< 需在安全关闭后重建通信实例 */

  /* --- 序列化与反序列化错误 (-40 ~ -49) --- */
  WL_ERR_CORRUPT_PAYLOAD =
      -50, /**< Payload 解析失败 (比如反序列化时发现字段越界或类型不匹配) */

};

const char *wl_err_str(int err);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // INCLUDE_WIRELINK_TYPE_H_
