/**
 * @file topics.cpp
 * @author Keten (2863861004@qq.com)
 * @brief Lock-free pub-sub mechanism for inter-task communication.
 * @version 0.3
 * @date 2024-09-26
 */
#include "topics.hpp"

#include <atomic>
#include <cstdint>
#include <cstring>

// 统一空返回值
static publish_data MakeEmptyData() { return publish_data{nullptr, -1}; }

// 固定slot
struct publish_slot {
  uint8_t data[TOPICS_MAX_MESSAGE_SIZE];
  int len;
};

// 深拷贝到slot,超长直接拒绝
static bool CopyToSlot(const publish_data &src, publish_slot *out) {
  if (out == nullptr || src.data == nullptr || src.len <= 0) {
    return false;
  }
  if (src.len > static_cast<int>(TOPICS_MAX_MESSAGE_SIZE)) {
    return false;
  }
  std::memcpy(out->data, src.data, static_cast<size_t>(src.len));
  out->len = src.len;
  return true;
}

// 每个订阅者维护一个spsc环形队列（固定静态容量）
template <typename T, uint32_t MaxCapacity> class SpscOverwriteRing {
public:
  SpscOverwriteRing() = default;

  bool Init(uint32_t capacity) {
    if (capacity == 0U) {
      capacity = 1U;
    } else if (capacity > MaxCapacity) {
      capacity = MaxCapacity;
    }
    capacity_ = capacity;
    head_.store(0U, std::memory_order_relaxed);
    tail_.store(0U, std::memory_order_relaxed);
    return true;
  }

  bool IsReady() const { return capacity_ > 0U; }

  void Clear() {
    if (!IsReady()) {
      return;
    }

    head_.store(0U, std::memory_order_relaxed);
    tail_.store(0U, std::memory_order_relaxed);
  }

  void Push(const T &item) {
    if (!IsReady()) {
      return;
    }

    uint32_t tail = tail_.load(std::memory_order_relaxed);
    uint32_t next_tail = Inc(tail);
    uint32_t head = head_.load(std::memory_order_acquire);

    // 满时覆盖最旧元素。
    if (next_tail == head) {
      head_.store(Inc(head), std::memory_order_release);
    }

    items_[tail] = item;
    tail_.store(next_tail, std::memory_order_release);
  }

  bool Pop(T *out) {
    if (out == nullptr || items_ == nullptr || capacity_ == 0U) {
      return false;
    }

    uint32_t head = head_.load(std::memory_order_relaxed);
    uint32_t tail = tail_.load(std::memory_order_acquire);
    if (head == tail) {
      return false;
    }

    *out = items_[head];
    head_.store(Inc(head), std::memory_order_release);
    return true;
  }

private:
  uint32_t Inc(uint32_t index) const { return (index + 1U) % capacity_; }

  T items_[MaxCapacity]{};
  uint32_t capacity_{0};
  std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> tail_{0};
};

struct topic_queue_t {
  SpscOverwriteRing<publish_slot, TOPICS_MAX_HISTORY_LEN + 1U> ring;
};

struct subscriber_state {
  const char *sub_topic;
  struct internal_topic *topic;
  topic_queue_t queue;
  uint8_t scratch[TOPICS_MAX_MESSAGE_SIZE];
  struct subscriber_state *next;
  bool in_use;
};

// topic 内部状态
struct internal_topic {
  subscriber_state *subs;
  const char *topic_str;
  struct internal_topic *next;
  uint32_t sub_count;
  bool in_use;
};

static internal_topic g_topic_pool[TOPICS_MAX_TOPICS]{};
static subscriber_state
    g_sub_pool[TOPICS_MAX_TOPICS * TOPICS_MAX_SUBS_PER_TOPIC]{};

static void queue_init(topic_queue_t *queue, uint32_t capacity);

// 从静态地址中申请内存,创建话题
static internal_topic *AllocateTopic(const char *topic) {
  for (auto &item : g_topic_pool) {
    if (!item.in_use) {
      item.in_use = true;
      item.topic_str = topic;
      item.subs = nullptr;
      item.sub_count = 0U;
      item.next = nullptr;
      return &item;
    }
  }
  return nullptr;
}

// 从静态地址中申请内存,创建订阅者
static subscriber_state *AllocateSubscriber(internal_topic *topic,
                                            const char *sub_topic,
                                            uint32_t buffer_len) {
  if (topic == nullptr || sub_topic == nullptr) {
    return nullptr;
  }
  if (topic->sub_count >= TOPICS_MAX_SUBS_PER_TOPIC) {
    return nullptr;
  }

  for (auto &item : g_sub_pool) {
    if (!item.in_use) {
      item.in_use = true;
      item.sub_topic = sub_topic;
      item.topic = topic;
      item.next = nullptr;
      queue_init(&item.queue, buffer_len);
      topic->sub_count += 1U;
      return &item;
    }
  }
  return nullptr;
}

// 发布订阅总线管理类,单例模式
class TopicBus {
public:
  static TopicBus &Instance() {
    static TopicBus instance;
    return instance;
  }

  struct internal_topic *RegisterTopic(const char *topic) {
    struct internal_topic *now = nullptr;

    if (topic == nullptr) {
      return nullptr;
    }

    for (now = topics_; now != nullptr; now = now->next) {
      if (now->topic_str != nullptr &&
          std::strcmp(now->topic_str, topic) == 0) {
        return now;
      }
    }

    now = AllocateTopic(topic);
    if (now == nullptr) {
      return nullptr;
    }

    now->next = topics_;
    topics_ = now;
    return now;
  }

private:
  TopicBus() = default;
  struct internal_topic *topics_{nullptr};
};

static void PublishToSubscribers(internal_topic *topic, publish_data data);
static publish_data GetSubscriberData(subscriber_state *sub);
static void queue_push(topic_queue_t *queue, publish_data data);
static int queue_pop(topic_queue_t *queue, publish_slot *out);

static void queue_init(topic_queue_t *queue, uint32_t capacity) {
  if (queue == nullptr) {
    return;
  }
  // Capacity is "history length"; ring needs one extra slot to detect full.
  uint32_t history = capacity;
  if (history == 0U) {
    history = 1U;
  } else if (history > TOPICS_MAX_HISTORY_LEN) {
    history = TOPICS_MAX_HISTORY_LEN;
  }
  uint32_t ring_capacity = history + 1U;
  if (ring_capacity > TOPICS_MAX_HISTORY_LEN + 1U) {
    ring_capacity = TOPICS_MAX_HISTORY_LEN + 1U;
  }
  (void)queue->ring.Init(ring_capacity);
}

static void queue_push(topic_queue_t *queue, publish_data data) {
  if (queue == nullptr || !queue->ring.IsReady()) {
    return;
  }
  // 先做深拷贝,再push到spsc中
  publish_slot slot{};
  if (!CopyToSlot(data, &slot)) {
    return;
  }
  queue->ring.Push(slot);
}

static int queue_pop(topic_queue_t *queue, publish_slot *out) {
  if (queue == nullptr || out == nullptr) {
    return 0;
  }
  return queue->ring.Pop(out) ? 1 : 0;
}

static void PublishToSubscribers(internal_topic *topic, publish_data data) {
  if (topic == nullptr) {
    return;
  }

  subscriber_state *node = topic->subs;
  while (node != nullptr) {
    queue_push(&node->queue, data);
    node = node->next;
  }
}

static publish_data GetSubscriberData(subscriber_state *sub) {
  publish_data now = MakeEmptyData();

  if (sub == nullptr) {
    return now;
  }

  publish_slot slot{};
  if (queue_pop(&sub->queue, &slot) == 0) {
    return now;
  }

  if (slot.len <= 0 || slot.len > static_cast<int>(TOPICS_MAX_MESSAGE_SIZE)) {
    return MakeEmptyData();
  }

  std::memcpy(sub->scratch, slot.data, static_cast<size_t>(slot.len));
  return publish_data{sub->scratch, slot.len};
}

TopicPublisher::TopicPublisher(const char *topic)
    : topic_(TopicBus::Instance().RegisterTopic(topic)) {}

bool TopicPublisher::Publish(uint8_t *data, int len) const {
  if (topic_ == nullptr || data == nullptr || len <= 0) {
    return false;
  }
  if (len > static_cast<int>(TOPICS_MAX_MESSAGE_SIZE)) {
    return false;
  }
  publish_data packet{data, len};
  PublishToSubscribers(topic_, packet);
  return true;
}

TopicSubscriber::TopicSubscriber(const char *topic, uint32_t buffer_len) {
  internal_topic *now_topic = TopicBus::Instance().RegisterTopic(topic);
  if (now_topic == nullptr) {
    return;
  }

  subscriber_state *obj = AllocateSubscriber(now_topic, topic, buffer_len);
  if (obj == nullptr) {
    return;
  }

  obj->next = now_topic->subs;
  now_topic->subs = obj;
  sub_ = obj;
}

bool TopicSubscriber::TryGet(publish_data *out) const {
  if (sub_ == nullptr || out == nullptr) {
    return false;
  }

  publish_data packet = GetSubscriberData(sub_);
  if (packet.data == nullptr || packet.len < 0) {
    return false;
  }

  *out = packet;
  return true;
}
