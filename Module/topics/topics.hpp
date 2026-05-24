/**
 * @file topics.hpp
 * @author Keten (2863861004@qq.com)
 * @brief 队列实现
 * @version 0.1
 * @date 2026-05-24 修订版
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * 每个topic维护订阅者链表;发布时对每个订阅者做一次深拷贝并push进其独立环形缓冲;
 *        订阅者取消息时从自己的缓冲取出,复制到内部暂存区并释放原拷贝,保证内存安全与互不影响
 * @versioninfo :
 */
#pragma once

#include <cstdint>
#include <cstring>

/* 静态内存配置 */

// 最大消息结构体字节数
#ifndef TOPICS_MAX_MESSAGE_SIZE
#define TOPICS_MAX_MESSAGE_SIZE 32U
#endif

// 最大历史长度(订阅者缓冲队列最大长度)
#ifndef TOPICS_MAX_HISTORY_LEN
#define TOPICS_MAX_HISTORY_LEN 8U
#endif

// 最大topic数量
#ifndef TOPICS_MAX_TOPICS
#define TOPICS_MAX_TOPICS 8U
#endif

// 某个topics上能挂载订阅者的最大数目
#ifndef TOPICS_MAX_SUBS_PER_TOPIC
#define TOPICS_MAX_SUBS_PER_TOPIC 4U
#endif

typedef struct publish_data_t {
  uint8_t *data;
  int len;
} publish_data;

struct internal_topic;
struct subscriber_state;

class TopicPublisher {
public:
  explicit TopicPublisher(const char *topic);

  bool IsValid() const { return topic_ != nullptr; }

  bool Publish(uint8_t *data, int len) const;

private:
  internal_topic *topic_{nullptr};
};

class TopicSubscriber {
public:
  TopicSubscriber(const char *topic, uint32_t buffer_len);

  bool IsValid() const { return sub_ != nullptr; }

  // Returned data is owned by the subscriber and valid until the next TryGet.
  bool TryGet(publish_data *out) const;

private:
  subscriber_state *sub_{nullptr};
};

// 模板类,通过消息类型构造发布者
template <typename T> class TypedTopicPublisher {
public:
  explicit TypedTopicPublisher(const char *topic) : pub_(topic) {}

  bool IsValid() const { return pub_.IsValid(); }

  bool Publish(const T &value) const {
    return pub_.Publish(
        const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(&value)),
        static_cast<int>(sizeof(T)));
  }

private:
  TopicPublisher pub_;
};

// 通过消息类型构造订阅者
template <typename T> class TypedTopicSubscriber {
public:
  TypedTopicSubscriber(const char *topic, uint32_t buffer_len)
      : sub_(topic, buffer_len) {}

  bool IsValid() const { return sub_.IsValid(); }

  bool TryGet(T *out) const {
    if (out == nullptr) {
      return false;
    }

    publish_data packet{};
    if (!sub_.TryGet(&packet)) {
      return false;
    }

    if (packet.len != static_cast<int>(sizeof(T))) {
      return false;
    }

    std::memcpy(out, packet.data, sizeof(T));
    return true;
  }

private:
  TopicSubscriber sub_;
};
