# 5 ROS2 Jazzy 通信数据链源码分析

机内与机间链路上拷贝、序列化与延迟的源码级分析

> 摘要：本文沿着一条 topic 消息从 `publish()` 到订阅回调的完整路径，逐段核对 ROS 2 Jazzy 的 `rclcpp`、`rcl`、`rmw_fastrtps` 与 Fast DDS 2.14 源码，回答三个问题：数据在哪一层被拷贝、在哪一层被序列化/反序列化、哪些环节真正影响延迟。结论是：进程内是唯一可做到 0 拷贝的路径；同机跨进程默认走 SHM transport 但仍有 4 次整块拷贝；跨机还要叠加内核收发、分片与重传；`Image`/`PointCloud2` 因含变长数组，无法使用 Fast DDS 的 loan 零拷贝。
>
> 版本基线：ROS 2 Jazzy（`rclcpp`/`rcl` 的 `jazzy` 分支）、`rmw_fastrtps` 8.4.5（`jazzy`）、Fast DDS `2.14.x`、Fast CDR `2.2.x`；对比部分使用 CycloneDDS `releases/0.10.x`、`rmw_cyclonedds` `jazzy`、iceoryx `v2.0.5`。
> 证据约定：**【源码】**表示结论可由文中逐字代码直接支撑；**【推断】**表示由代码结构导出、但未单独测量；**【待确认】**表示本次未取到对应实现，不作确定结论。
> 范围：C++ `rclcpp` 的 topic 发布/订阅（不含 service/action、参数、TF、执行器实现细节的完整剖析）。主线是 Jazzy 默认的 `rmw_fastrtps_cpp`，§3.7 给出 CycloneDDS 的对照。
> 定位方式：源码引用统一用「仓库 + 文件 + 函数/类名」，不写行号——`jazzy` 是滚动分支，行号会随提交漂移；文中代码均为抓取时刻的逐字片段。

---

## 0. 结论先行

1. **ROS 2 的 topic 数据链不是一条零拷贝管道**，而是 `rclcpp → rcl → rmw → DDS → 传输` 的多段流水线。每一段是否拷贝、是否序列化，由**进程布局**（同进程/同机跨进程/跨机）和**选项**（intra-process、loan、serialized message、QoS）共同决定。
2. **只有进程内（intra-process）存在真正的 0 拷贝路径**：发布端用 `std::unique_ptr` 移交所有权，订阅回调形参为 `shared_ptr<const T>` 或 `const T&`，且该 topic 没有跨进程订阅者、发布端不是 TransientLocal。此时不序列化、不进 DDS、不做 memcpy。
3. **同机跨进程默认仍走 Fast DDS**。`rmw_fastrtps` 在默认 QoS 路径上显式执行 `data_sharing().off()`，所以 **ROS 2 Jazzy 默认并没有启用 Fast DDS 的 Data-sharing 零拷贝**；同机在 SHM transport 可用时走 SHM（否则回落 UDP loopback），它依旧要序列化、组装 RTPS 报文、把整条报文 memcpy 进共享内存段。
4. **跨机走 UDP + RTPS**：序列化一次，RTPS 报文组装（Fast DDS 2.14 常见路径是两跳拷贝），内核发送/接收各一次拷贝，接收端反序列化一次。大消息还要分片/重组。**CycloneDDS 在报文组装阶段用 iovec 引用 `serdata`，没有这两跳拷贝**；但它每帧 `new` 一块 `serdata` 缓冲，分配压力更大。
5. **大消息（Image/PointCloud2）的主要成本往往不是“网线”**，而是：逐字段序列化、多次整块 memcpy、反序列化时为 `sequence` 分配新缓冲并整块复制。1 Gbps 链路对 4 MB 图像的带宽下限约 32 ms，这个下限之外还有上述 CPU 与内存带宽开销。
6. **loaned message 在 `rmw_fastrtps` 中是“发布侧零拷贝”，但不是端到端零拷贝**：它跳过取池与序列化，但 RTPS 组装仍要拷贝，接收端 `StatefulReader` 还会把 payload 复制进 reader 池。真正的端到端零拷贝是 Data-sharing，而它在默认 QoS 路径上被 `data_sharing().off()` 关闭。且 loan 要求 plain 类型（不能含 `sequence<>` 或无界字符串），`sensor_msgs/msg/Image`、`PointCloud2` 不满足；订阅侧默认还关闭 loan（`RCL_DISABLE_LOANED_MESSAGES`）。

---

## 1. 分层数据链总览

### 1.1 发送侧

```text
用户线程
  Publisher::publish(...)                         rclcpp/include/rclcpp/publisher.hpp
    ├─ intra-process 分支（可选） ──────────────► IntraProcessManager（同进程，见 §3）
    └─ do_inter_process_publish(const T&)
         └─ rcl_publish()                         rcl/src/rcl/publisher.c
              └─ rmw_publish()                    rmw_fastrtps_shared_cpp/src/rmw_publish.cpp
                   └─ DataWriter::write_w_timestamp()
                        └─ DataWriterImpl::perform_create_new_change()
                             ├─ get_free_payload_from_pool()      ← payload 池取块
                             ├─ type_->serialize()                 ← 序列化写进 payload
                             └─ payload.move_into_change(*ch)      ← 所有权搬移
                        └─ WriterHistory::add_change_()            ← writer 锁
                             └─ StatefulWriter::unsent_change_added_to_history()
                                  └─ flow_controller_->add_new_sample()
                                       └─ RTPSMessageGroup::add_data()/add_data_frag()
                                            └─ transport->send()  ← SHM memcpy / UDP sendmsg
```

### 1.2 接收侧

```text
Executor 线程
  rmw_wait() → Executor::execute_subscription()
    ├─ subscription->create_message()             ← 默认策略：每帧新分配一条消息
    ├─ rcl_take()                                 rcl/src/rcl/subscription.c
    │    └─ rmw_take()                            rmw_fastrtps_shared_cpp/src/rmw_take.cpp
    │         └─ DataReader::take()
    │              └─ TypeSupport::deserialize()  ← cdr_deserialize 逐字段写用户消息
    └─ handle_message() → any_callback_.dispatch() → 用户回调
```

进程内订阅不走上图：它作为独立的 `Waitable` 挂在 executor 的 wait set 上，`take_data()` 从环形缓冲取 `shared_ptr`/`unique_ptr`，`execute()` 直接分发（见 §2.5）。

### 1.3 三条链路的分叉点

| 链路 | 分叉条件 | 是否序列化 | 是否经过 DDS | 典型拷贝次数 |
|---|---|---|---|---|
| 进程内 intra-process | 发布/订阅同进程、同一 Context、均开启 intra-process | 否 | 否（除非存在跨进程订阅者） | 0～N（取决于回调签名与订阅数） |
| 同机跨进程 | 同主机、不同进程 | 是 | 是（Fast DDS） | 序列化 1 次 + 发送侧 3 次 + reader 池 1 次 + 反序列化 1 次（详见 §6） |
| 跨机 | 不同主机 | 是 | 是（Fast DDS/UDP） | 在上一行基础上再加内核收发与分片/重组 |

### 1.4 一条消息的时序（同步发布、同机 SHM 为例）

| 时刻 | 线程 | 发生的事 | 是否阻塞 `publish()` |
|---|---|---|---|
| t0 | 发布者线程 | `Publisher::publish()` → `rcl_publish()` → `rmw_publish()`，只传指针 | 否 |
| t1 | 发布者线程 | `perform_create_new_change()`：取 payload、`cdr_serialize` 逐字段写入 | 是（CPU） |
| t2 | 发布者线程 | `WriterHistory::add_change_()`：writer 锁、入历史 | 可能（锁/历史满） |
| t3 | 发布者线程 | `RTPSMessageGroup`：payload → submessage → full message（两跳拷贝） | 是（CPU） |
| t4 | 发布者线程 | `SharedMemTransport::send()`：整条报文 memcpy 进共享内存段 | 可能（段满/队列满） |
| t4.5 | DDS 接收线程 | `StatefulReader::processDataMsg()`：`payload_pool_->get_payload()` 把 payload 复制进 reader 池 | — |
| t5 | 订阅者线程 | `rmw_wait()` 返回 → `execute_subscription()` → `create_message()` 分配新消息 | — |
| t6 | 订阅者线程 | `DataReader::take()` → `cdr_deserialize` 逐字段写入用户消息（`sequence` 重新分配） | — |
| t7 | 订阅者线程 | `handle_message()` → 回调 | — |

因此 `publish()` 的返回延迟包含 t1–t4；订阅回调的延迟包含 t5–t7。两者之间还有 transport 传递与线程唤醒，通常不在 `publish()` 的计时内。测量时必须分别打点，否则会把“发布端 CPU 开销”误判为“网络延迟”。

### 1.5 延迟分解

```text
T_end_to_end = T_publish_api + T_transport + T_wakeup + T_take + T_dispatch + T_callback

T_publish_api = T_serialize + T_history_lock + T_rtps_assemble + T_transport_send
T_take        = T_parse + T_deserialize + T_alloc
```

- 进程内：`T_serialize = 0`、`T_rtps_assemble = 0`、`T_transport = 0`，只剩 `T_enqueue + T_wakeup + T_dispatch`；
- 同机跨进程：`T_transport` 是共享内存段拷贝 + 对端解析，通常远小于序列化/反序列化；
- 跨机：`T_transport` 由带宽、分片、重传、排队共同决定，方差通常最大。

---

## 2. 进程内通信（intra-process）

### 2.1 开关与约束

`resolve_use_intra_process()` 按发布端/订阅端各自的 `IntraProcessSetting` 解析，默认回落到节点选项：

```cpp
// rclcpp/include/rclcpp/detail/resolve_use_intra_process.hpp
case IntraProcessSetting::Enable:
  use_intra_process = true;
  break;
case IntraProcessSetting::Disable:
  use_intra_process = false;
  break;
case IntraProcessSetting::NodeDefault:
  use_intra_process = node_base.get_use_intra_process_default();
  break;
```

进程内通信只共享**同一个 `rclcpp::Context`** 下的 `IntraProcessManager` 实例，跨 Context 的节点即使在同一进程也不会命中该路径。另外，Jazzy 的 `rclpy` 没有等价的 intra-process 通道，Python 节点无法利用这条路径；要享受零拷贝，发布与订阅都必须是 C++ 节点。

约束条件（`publisher.hpp` 与 `subscription.hpp` 构造期检查，相同）：

- `history` 必须是 `KeepLast`；
- `depth` 不能为 0；
- 订阅端只要开启 intra-process 就创建接收缓冲；发布端只有 `durability == TransientLocal` 才创建发布缓冲。

违反前两条会在构造期直接 `throw std::invalid_argument`。

### 2.2 发布：`const T&` 与 `unique_ptr` 的拷贝差异

`publish(const T&)` 在 intra-process 关闭时把引用直接传到 `rcl_publish`，rclcpp/rcl 层**零拷贝**；开启时则**必定先深拷贝一份**：

```cpp
// rclcpp/include/rclcpp/publisher.hpp
template<typename T>
typename std::enable_if_t<
  rosidl_generator_traits::is_message<T>::value &&
  std::is_same<T, ROSMessageType>::value
>
publish(const T & msg)
{
  // Avoid allocating when not using intra process.
  if (!intra_process_is_enabled_) {
    this->do_inter_process_publish(msg);
    return;
  }
  // Otherwise we have to allocate memory in a unique_ptr and pass it along.
  // As the message is not const, a copy should be made.
  auto unique_msg = this->duplicate_ros_message_as_unique_ptr(msg);
  this->publish(std::move(unique_msg));
}
```

`publish(std::unique_ptr<T>)` 才是进程内零拷贝的入口。它先计算是否需要同时走 DDS：

```cpp
// rclcpp/include/rclcpp/publisher.hpp
bool inter_process_publish_needed =
  get_subscription_count() > get_intra_process_subscription_count() || buffer_;

if (inter_process_publish_needed) {
  auto shared_msg =
    this->do_intra_process_ros_message_publish_and_return_shared(std::move(msg));
  if (buffer_) {
    buffer_->add_shared(shared_msg);
  }
  this->do_inter_process_publish(*shared_msg);
} else {
  if (buffer_) {
    auto shared_msg =
      this->do_intra_process_ros_message_publish_and_return_shared(std::move(msg));
    buffer_->add_shared(shared_msg);
  } else {
    this->do_intra_process_ros_message_publish(std::move(msg));
  }
}
```

关键点：

- **顺序是“先进程内、后进程间”**，源码注释明确说明这是为了降低 publish→subscribe 延迟。
- **进程内的入队与唤醒是在 `publish()` 调用线程里同步完成的**：`do_intra_process_publish()` 遍历订阅、把消息放进各自的环形缓冲，并立刻 `trigger_guard_condition()`；订阅侧回调本身由 executor 线程稍后执行。因此 `publish()` 的耗时包含“写 N 个订阅缓冲 + N 次 guard 触发”，而回调延迟还要加上 executor 唤醒与调度。
- 是否走 DDS 由 `get_subscription_count() > get_intra_process_subscription_count()` 判定：前者是 rmw 视角的匹配订阅数，后者是 IPM 内部登记的进程内订阅数；差值 >0 说明存在只能走 DDS 的订阅者。订阅端即使开启 intra-process 也会无条件创建 rmw subscription，因此同一进程内的订阅通常同时计入两者。
- `|| buffer_`：发布端为 TransientLocal 时即使全是进程内订阅也强制走 DDS，以保证晚加入者能收到历史数据。
- 序列化消息（`rcl_serialized_message_t`）在 intra-process 下直接抛异常：`storing serialized messages in intra process is not supported yet`。

### 2.3 IPM 内部的拷贝决策

`IntraProcessManager::do_intra_process_publish()` 根据订阅缓冲的类型把订阅者分成两类：

- `take_shared_subscriptions`：缓冲存 `shared_ptr<const T>`；
- `take_ownership_subscriptions`：缓冲存 `unique_ptr<T>`。

```cpp
// rclcpp/include/rclcpp/experimental/intra_process_manager.hpp
if (sub_ids.take_ownership_subscriptions.empty()) {
  // None of the buffers require ownership, so we promote the pointer
  std::shared_ptr<MessageT> msg = std::move(message);
  this->add_shared_msg_to_buffers<...>(msg, sub_ids.take_shared_subscriptions);
} else if (!sub_ids.take_ownership_subscriptions.empty() &&
  sub_ids.take_shared_subscriptions.size() <= 1)
{
  // There is at maximum 1 buffer that does not require ownership.
  // So this case is equivalent to all the buffers requiring ownership
  ...
  this->add_owned_msg_to_buffers<...>(std::move(message), concatenated_vector, allocator);
} else if (!sub_ids.take_ownership_subscriptions.empty() &&
  sub_ids.take_shared_subscriptions.size() > 1)
{
  // Construct a new shared pointer from the message
  // for the buffers that do not require ownership
  auto shared_msg = std::allocate_shared<MessageT, MessageAllocatorT>(allocator, *message);
  this->add_shared_msg_to_buffers<...>(shared_msg, sub_ids.take_shared_subscriptions);
  this->add_owned_msg_to_buffers<...>(std::move(message), sub_ids.take_ownership_subscriptions, allocator);
}
```

由此得到进程内发布的拷贝账本（`publish(const T&)` 已经先付 1 次深拷贝，下表从 `unique_ptr` 入口算起）：

| 订阅构成（intra-process 开启） | IPM 内追加拷贝 | 说明 |
|---|---|---|
| 全部是 `shared_ptr<const T>` / `const T&` 回调 | 0 | `unique_ptr → shared_ptr` 是所有权转移 |
| 有 N 个 `unique_ptr<T>` 回调（owning），无 shared | N−1 | 最后一个订阅直接 move，其余各拷一份 |
| 同时有 owning 与多个 shared | 1（`allocate_shared`）+ (N−1) | 为 shared 订阅复制一份，owning 再各自拷 |

**因此，“用了 intra-process”不等于“零拷贝”**：只要订阅侧的回调签名要求独占所有权，或者发布端用了 `publish(const T&)`，拷贝就会发生。

### 2.4 缓冲：容量等于 QoS depth，满时覆盖最旧

```cpp
// rclcpp/include/rclcpp/experimental/create_intra_process_buffer.hpp
size_t buffer_size = qos.depth();
...
auto buffer_implementation =
  std::make_unique<rclcpp::experimental::buffers::RingBufferImplementation<BufferT>>(
    buffer_size);
```

`RingBufferImplementation` 的 `enqueue` 在满时前移读指针，**静默覆盖最旧帧**：

```cpp
// rclcpp/include/rclcpp/experimental/buffers/ring_buffer_implementation.hpp
void enqueue(BufferT request) override
{
  std::lock_guard<std::mutex> lock(mutex_);

  write_index_ = next_(write_index_);
  ring_buffer_[write_index_] = std::move(request);

  if (is_full_()) {
    read_index_ = next_(read_index_);
  } else {
    size_++;
  }
}
```

要点：

- 每个订阅缓冲一把普通 `std::mutex`，`enqueue`/`dequeue` 都加锁；
- **没有 `condition_variable`**，跨线程唤醒靠 `rclcpp::GuardCondition`；
- **没有背压**：发布端不检查 `available_capacity()`，满了就丢最旧；
- `depth=1` 时，消费者永远只能拿到“最新一帧”，旧帧被覆盖——这是“保新鲜”而不是“保完整”。

### 2.5 订阅：Waitable 路径与 `from_intra_process`

进程内订阅在 `Subscription` 构造时创建 `SubscriptionIntraProcess` 并注册到 IPM。它继承 `rclcpp::Waitable`，因此 executor 走的是 `waitable` 分支而不是 `subscription` 分支：

```cpp
// rclcpp/src/rclcpp/executor.cpp（execute_any_executable，节选）
if (any_exec.subscription) {
  execute_subscription(any_exec.subscription);
}
...
if (any_exec.waitable) {
  const std::shared_ptr<void> & const_data = any_exec.data;
  any_exec.waitable->execute(const_data);
}
```

取数与分发：

```cpp
// rclcpp/include/rclcpp/experimental/subscription_intra_process.hpp
std::shared_ptr<void>
take_data() override
{
  ConstMessageSharedPtr shared_msg;
  MessageUniquePtr unique_msg;

  if (any_callback_.use_take_shared_method()) {
    shared_msg = this->buffer_->consume_shared();
    if (!shared_msg) {
      return nullptr;
    }
  } else {
    unique_msg = this->buffer_->consume_unique();
    if (!unique_msg) {
      return nullptr;
    }
  }

  if (this->buffer_->has_data()) {
    this->trigger_guard_condition();
  }

  return std::static_pointer_cast<void>(
    std::make_shared<std::pair<ConstMessageSharedPtr, MessageUniquePtr>>(
      std::pair<ConstMessageSharedPtr, MessageUniquePtr>(shared_msg, std::move(unique_msg))));
}
```

`execute()` 里伪造 `MessageInfo`，然后按回调签名分发：

```cpp
rmw_message_info_t msg_info;
msg_info.publisher_gid = {0, {0}};
msg_info.from_intra_process = true;
```

注意：

- `take_data()` 里有一个**每条消息一次的小堆分配**（`std::make_shared<std::pair<...>>`），这不是消息体拷贝，但仍是分配；
- 缓冲里还有数据时会再次 `trigger_guard_condition()`，保证同一轮 wait 不丢唤醒；
- `from_intra_process = true` 是应用层区分“进程内/进程间”的唯一可靠标记。

### 2.6 回调签名决定最终拷贝次数

`resolve_intra_process_buffer_type()` 在用户未显式指定时，按回调签名选择缓冲元素类型：

```cpp
// rclcpp/include/rclcpp/detail/resolve_intra_process_buffer_type.hpp
if (resolved_buffer_type == IntraProcessBufferType::CallbackDefault) {
  if (any_subscription_callback.use_take_shared_method()) {
    resolved_buffer_type = IntraProcessBufferType::SharedPtr;
  } else {
    resolved_buffer_type = IntraProcessBufferType::UniquePtr;
  }
}
```

- 回调形参为 `std::shared_ptr<const T>`（或 `const std::shared_ptr<const T>&`）→ `use_take_shared_method() == true` → 缓冲存 `shared_ptr`，分发时**0 拷贝**；
- 回调形参为 `const T&` → 默认 `UniquePtr` 缓冲，`dispatch_intra_process` 里 `callback(*message)`，**0 拷贝**；
- 回调形参为 `std::unique_ptr<T>` → `UniquePtr` 缓冲，最后一个订阅 move，其余订阅各拷一份；
- 若缓冲类型与回调不匹配（例如显式把缓冲设成 `SharedPtr` 却用 `unique_ptr` 回调），`TypedIntraProcessBuffer` 会**静默深拷贝**一次。

`TypedIntraProcessBuffer` 中两处“兜底拷贝”值得警惕：

```cpp
// rclcpp/include/rclcpp/experimental/buffers/intra_process_buffer.hpp
// MessageSharedPtr to MessageUniquePtr
add_shared_impl(MessageSharedPtr shared_msg)
{
  // This should not happen: here a copy is unconditionally made, while the intra-process manager
  // can decide whether a copy is needed depending on the number and the type of buffers
  ...
  MessageAllocTraits::construct(*message_allocator_.get(), ptr, *shared_msg);
```

### 2.7 进程内仍可能走 DDS：重复投递的代价

当存在跨进程订阅者、或发布端是 TransientLocal 时，发布端会**同时**做进程内投递和 DDS 发布。订阅端即使开启了 intra-process，也仍然持有一个真实的 rmw subscription（`SubscriptionBase` 构造函数无条件调用 `rcl_subscription_init`）。于是同一帧可能出现两次：

```cpp
// rclcpp/src/rclcpp/subscription_base.cpp
bool
SubscriptionBase::take_type_erased(void * message_out, rclcpp::MessageInfo & message_info_out)
{
  rcl_ret_t ret = rcl_take(...);          // ← 先反序列化进 message_out
  ...
  if (matches_any_intra_process_publishers(
      &message_info_out.get_rmw_message_info().publisher_gid))
  {
    // In this case, the message will be delivered via intra-process and
    // we should ignore this copy of the message.
    return false;
  }
  return true;
}
```

**丢弃发生在 `rcl_take` 之后**：rmw 已经完成了接收、解析、反序列化，成本已经付出。`handle_message` 开头的 `matches_any_intra_process_publishers()` 早退也是同样的“先做后丢”。

这带来一个反直觉的结论：**混合进程内/跨进程订阅的 topic，可能比纯跨进程更慢**——发布端多做一次 DDS 序列化/发送，订阅端多做一次 DDS 接收/反序列化，然后把它扔掉。

### 2.8 进程内延迟影响因素

| 因素 | 机制 | 源码依据 |
|---|---|---|
| executor 唤醒延迟 | `GuardCondition::trigger()` → `rmw_wait` 返回 → 调度到 executor 线程 | `subscription_intra_process.hpp` |
| 回调组串行化 | `MutuallyExclusive` 组同一时刻只允许一个回调 | `executor.cpp` / `callback_group` |
| waitable 优先级最低 | executor 取 executable 的顺序是 timer → subscription → service → client → waitable | `executor.cpp` |
| 缓冲深度 | 满时覆盖最旧；深度越大，排队延迟越大但越不容易丢帧 | `ring_buffer_implementation.hpp` |
| 回调签名 | owning 回调触发 N−1 次深拷贝 | `intra_process_manager.hpp` |
| 混合 DDS 订阅 | 重复投递导致额外的序列化/反序列化 | `take_type_erased` / `handle_message` |
| 每条消息的小分配 | `take_data()` 的 `make_shared<pair<...>>` | `subscription_intra_process.hpp` |
| 发布端锁 | 每个订阅缓冲的 `std::mutex`，以及 IPM 的 `shared_timed_mutex` 读锁 | `intra_process_manager.hpp` |

---

## 3. 同机跨进程通信（Fast DDS / SHM）

### 3.1 `rmw_fastrtps` 对 QoS 的默认覆盖

`rmw_fastrtps_cpp` 在创建 writer/reader 时，只要 `leave_middleware_default_qos` 为假，就会强制设置：

```cpp
// rmw_fastrtps_cpp/src/publisher.cpp
if (!participant_info->leave_middleware_default_qos) {
  if (participant_info->publishing_mode == publishing_mode_t::ASYNCHRONOUS) {
    writer_qos.publish_mode().kind = eprosima::fastrtps::ASYNCHRONOUS_PUBLISH_MODE;
  } else if (participant_info->publishing_mode == publishing_mode_t::SYNCHRONOUS) {
    writer_qos.publish_mode().kind = eprosima::fastrtps::SYNCHRONOUS_PUBLISH_MODE;
  }

  writer_qos.endpoint().history_memory_policy =
    eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

  writer_qos.data_sharing().off();
}
```

订阅侧同样：

```cpp
// rmw_fastrtps_cpp/src/subscription.cpp
reader_qos.endpoint().history_memory_policy =
  eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

reader_qos.data_sharing().off();
```

这两段代码有三个直接后果：

1. **Data-sharing 被关闭**：ROS 2 Jazzy 默认不会走 Fast DDS 的“payload 留在 writer 历史、reader 直接引用”的共享内存零拷贝；同机通信仍走 SHM **transport**，而 transport 是要拷贝的。
2. **payload 池策略固定为 `PREALLOCATED_WITH_REALLOC`**：稳态下复用池中块，减少 malloc；但消息变大时会 realloc。
3. **发布模式由 `publishing_mode` 决定**：默认同步，只有显式配置为异步时才把发送交给异步线程。

> 只有当 `leave_middleware_default_qos` 为真（让 rmw 不覆盖中间件 QoS，通常通过 XML QoS 配置路径）时，上述 `data_sharing().off()` 才可能被绕过。这属于非默认路径，需要按部署自行验证。

若真的启用 Data-sharing，同机 reader 可以直接引用 writer 历史中的 payload，从而省掉“SHM 段拷贝”和“reader 池拷贝”；它要求 reader/writer 同 host、SHM 可用、QoS 兼容，且类型支持共享内存池。它与 SHM transport 是两套独立机制：前者共享 payload 所有权，后者只是把序列化后的报文搬进共享内存段。

### 3.2 序列化落点：直接写进 DDS payload

`rcl_publish` 只做参数校验与 tracepoint，把 `const void *` 原样交给 `rmw_publish`：

```cpp
// rcl/src/rcl/publisher.c
rcl_ret_t
rcl_publish(const rcl_publisher_t * publisher, const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  ...
  if (rmw_publish(publisher->impl->rmw_handle, ros_message, allocation) != RMW_RET_OK) {
    ...
  }
  return RCL_RET_OK;
}
```

`rmw_publish` 也只把指针塞进栈上的 `SerializedData`，不做拷贝：

```cpp
// rmw_fastrtps_shared_cpp/src/rmw_publish.cpp
rmw_fastrtps_shared_cpp::SerializedData data;
data.type = FASTRTPS_SERIALIZED_DATA_TYPE_ROS_MESSAGE;
data.data = const_cast<void *>(ros_message);
data.impl = info->type_support_impl_;
...
if (!info->data_writer_->write_w_timestamp(&data, eprosima::fastdds::dds::HANDLE_NIL, stamp)) {
```

真正的序列化在 `TypeSupport::serialize()`：它用 `FastBuffer` 直接包住 `payload->data`，把字段逐字段写进 DDS 的 payload 缓冲，**没有临时 buffer、没有事后 memcpy**：

```cpp
// rmw_fastrtps_shared_cpp/src/TypeSupport_impl.cpp
case FASTRTPS_SERIALIZED_DATA_TYPE_ROS_MESSAGE:
  {
    eprosima::fastcdr::FastBuffer fastbuffer(
      reinterpret_cast<char *>(payload->data), payload->max_size);
    eprosima::fastcdr::Cdr ser(
      fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
      eprosima::fastcdr::CdrVersion::XCDRv1);
    ser.set_encoding_flag(eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR);
    if (this->serializeROSmessage(ser_data->data, ser, ser_data->impl)) {
      payload->encapsulation = ser.endianness() ==
        eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
      payload->length = (uint32_t)ser.get_serialized_data_length();
      return true;
    }
    break;
  }
```

`serializeROSmessage()` 先写 4 字节封装头，再调用生成代码的 `cdr_serialize`：

```cpp
// rmw_fastrtps_cpp/src/type_support_common.cpp
bool TypeSupport::serializeROSmessage(
  const void * ros_message, eprosima::fastcdr::Cdr & ser, const void * impl) const
{
  ser.serialize_encapsulation();
  if (has_data_) {
    auto callbacks = static_cast<const message_type_support_callbacks_t *>(impl);
    return callbacks->cdr_serialize(ros_message, ser);
  }
  ser << (uint8_t)0;
  return true;
}
```

Fast DDS 侧的调用者 `DataWriterImpl::perform_create_new_change()`：

```cpp
// Fast-DDS 2.14.x src/cpp/fastdds/publisher/DataWriterImpl.cpp（节选）
PayloadInfo_t payload;
bool was_loaned = check_and_remove_loan(data, payload);
if (!was_loaned)
{
    ...
    if (!get_free_payload_from_pool(type_->getSerializedSizeProvider(data), payload))
    {
        return ReturnCode_t::RETCODE_OUT_OF_RESOURCES;
    }

    if (!type_->serialize(data, &payload.payload, data_representation_))
    {
        ...
    }
}
...
CacheChange_t* ch = writer_->new_change(change_kind, handle);
if (ch != nullptr)
{
    payload.move_into_change(*ch);
```

结论：**普通路径 = 1 次序列化写入 payload**，`move_into_change` 是所有权搬移，不是字节拷贝。序列化在调用 `publish()` 的线程内同步完成（同步发布模式下）。

### 3.3 RTPS 报文组装：Fast DDS 2.14 的两跳拷贝

payload 进入 transport 之前，还要被组装成 RTPS 报文。`RTPSMessageGroup::add_data()` 先把 DATA 子报文写进 `submessage_msg_`，再 `append` 到完整报文：

```cpp
// Fast-DDS 2.14.x rtps/messages/submessages/DataMsg.hpp
// Add Serialized Payload
if (dataFlag)
{
    added_no_error &= CDRMessage::addData(
      msg, change->serializedPayload.data, change->serializedPayload.length);
}
```

```cpp
// Fast-DDS 2.14.x src/cpp/rtps/messages/RTPSMessageGroup.cpp（节选）
CacheChange_t change_to_add;
change_to_add.copy_not_memcpy(&change);
change_to_add.serializedPayload.data = change.serializedPayload.data;
...
RTPSMessageCreator::addSubmessageData(submessage_msg_, &change_to_add, ...);
```

```cpp
// 同文件：append_message()
bool ret_val = CDRMessage::appendMsg(full_msg, submsg);
```

也就是说，**payload 在发送侧至少被整块写/拷三次**：

1. `cdr_serialize` 写进 payload 缓冲；
2. `addSubmessageData` 把 payload memcpy 进 `submessage_msg_`；
3. `appendMsg` 把含 payload 的子报文再 memcpy 进 `full_msg_`。

之后才是传输层拷贝（SHM 或内核）。

### 3.4 SHM transport：显式 memcpy 进共享内存段

Fast DDS 的 builtin transports 通常包含 SHM 与 UDPv4；同机通信在同名 SHM 端口可用时通常优先选 SHM locator，否则回落到 UDP loopback（多两次内核收发）。SHM 发送端的核心是下面这次 `memcpy`：

```cpp
// Fast-DDS 2.14.x src/cpp/rtps/transport/shared_mem/SharedMemTransport.cpp
std::shared_ptr<SharedMemManager::Buffer> SharedMemTransport::copy_to_shared_buffer(
        const octet* send_buffer, uint32_t send_buffer_size,
        const std::chrono::steady_clock::time_point& max_blocking_time_point)
{
    assert(shared_mem_segment_);

    std::shared_ptr<SharedMemManager::Buffer> shared_buffer =
            shared_mem_segment_->alloc_buffer(send_buffer_size, max_blocking_time_point);

    memcpy(shared_buffer->data(), send_buffer, send_buffer_size);

    return shared_buffer;
}
```

默认段大小：

```cpp
// 同文件
static constexpr uint32_t shm_default_segment_size = 512 * 1024;
...
if (configuration_.segment_size() < configuration_.max_message_size())
{
    EPROSIMA_LOG_ERROR(RTPS_MSG_OUT, "max_message_size cannot be greater than segment_size");
    return false;
}
```

要点：

- 一个 RTPS 报文一次 `memcpy` 进共享内存段；多播/多 locator 时只拷一次，之后共享同一个 `Buffer`。
- 段大小默认 512 KiB，必须 ≥ `max_message_size`；否则 SHM 初始化失败。
- **大消息不会因此需要 4 MB 的段**：RTPS 会先按分片阈值（约 64 KiB）切成 `DATA_FRAG`，每个 fragment 单独装报文、单独拷进段。分片在上层完成，SHM 段只需容纳单条报文。
- SHM 接收端分两步：`SharedMemChannelResource::perform_listen_operation` 把段内 `message->data()` 直接交给 `ReceiverResource::OnDataReceived`，后者用 `msg.wraps = true; msg.buffer = data;` 做**零拷贝包装**，`MessageReceiver::proc_Submsg_Data` 只把 `serializedPayload.data` 指向段内偏移；但随后 `StatefulReader::processDataMsg` 会调用 `payload_pool_->get_payload(...)`，**把 payload 复制进 reader 的 payload 池**（源码注释：`Ask payload pool to copy the payload`）。因此 **SHM 接收端在“解析”这一步是零拷贝，但端到端仍不是零拷贝**。
- 真正的端到端零拷贝是 Data-sharing（reader 直接引用 writer 历史中的 payload），它与 SHM transport 是两套机制；本文 §3.1 已说明 ROS 2 默认把它关掉了。

### 3.5 接收：每帧新分配一条消息

`Executor::execute_subscription()` 先调用 `create_message()`：

```cpp
// rclcpp/include/rclcpp/message_memory_strategy.hpp
/// By default, dynamically allocate a new message.
virtual std::shared_ptr<MessageT> borrow_message()
{
  return std::allocate_shared<MessageT, MessageAlloc>(*message_allocator_.get());
}
```

随后 `rcl_take` → `rmw_take` → `data_reader_->take()`。`rmw_take` 的普通路径用栈上的单元素序列：

```cpp
// rmw_fastrtps_shared_cpp/src/rmw_take.cpp
eprosima::fastdds::dds::StackAllocatedSequence<void *, 1> data_values;
const_cast<void **>(data_values.buffer())[0] = &data;
eprosima::fastdds::dds::SampleInfoSeq info_seq{1};

while (ReturnCode_t::RETCODE_OK == info->data_reader_->take(data_values, info_seq, 1)) {
```

反序列化同样是“包住 payload、逐字段写用户消息”，没有整块 memcpy：

```cpp
// rmw_fastrtps_shared_cpp/src/TypeSupport_impl.cpp
case FASTRTPS_SERIALIZED_DATA_TYPE_ROS_MESSAGE:
  {
    eprosima::fastcdr::FastBuffer fastbuffer(
      reinterpret_cast<char *>(payload->data), payload->length);
    eprosima::fastcdr::Cdr deser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN);
    return deserializeROSmessage(deser, ser_data->data, ser_data->impl);
  }
```

但对 `sequence` 字段，`cdr_deserialize` 会 `resize(size)` 并逐元素/整块写入——**对 `Image::data` 这类 `uint8[]`，这相当于每帧分配一个 `W×H×3` 的新缓冲并复制一遍**。默认内存策略每帧新分配消息，所以这个分配无法避免（除非自定义 `MessageMemoryStrategy` 复用消息，或使用 loan）。

### 3.6 同机跨进程的拷贝账本（普通路径，已确认部分）

| 阶段 | 操作 | 拷贝性质 |
|---|---|---|
| `publish()` → `rcl_publish` → `rmw_publish` | 仅传指针 | 0 |
| `perform_create_new_change` | `cdr_serialize` 写 payload | 1 次写（读用户消息） |
| `RTPSMessageGroup::add_data` | payload → `submessage_msg_` | 1 次 memcpy |
| `RTPSMessageGroup::append_message` | submessage → `full_msg_` | 1 次 memcpy |
| `SharedMemTransport::send` | `full_msg_` → 共享内存段 | 1 次 memcpy |
| `SharedMemChannelResource` / `ReceiverResource` | 段内 `Buffer` 包装成 `ReceiveBuffer` | 0（`msg.wraps = true`） |
| `StatefulReader::processDataMsg` | `payload_pool_->get_payload(...)` 复制 payload | 1 次 memcpy |
| `cdr_deserialize` | payload → 用户消息（含 sequence 分配） | 1 次写/拷贝 |

即：**4 MB 图像在同机跨进程普通路径上，至少有 4 次整块 memcpy（addData、appendMsg、SHM 段、reader 池）+ 1 次序列化写 + 1 次反序列化写，合计约 6 遍内存流量**；此外还有每帧一条消息对象和 `data` vector 的分配。这个成本与“是否共享内存”关系不大，主要取决于序列化/反序列化和报文组装。

> reader 池拷贝发生在接收侧、且在 `DataReader::take` 之前（`StatefulReader::processDataMsg` 收包时完成）；反序列化发生在 executor 线程 `take` 时。两者不在同一个线程，测量时容易漏掉前者。

### 3.7 对比：CycloneDDS 的写路径为什么不一样

Jazzy 默认 RMW 是 `rmw_fastrtps_cpp`；如果设置 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，数据链会换成 CycloneDDS 0.10.x。两者最大的结构差异在**报文组装**：

- Fast DDS 2.14：payload 先写进 `submessage_msg_`，再 `appendMsg` 进 `full_msg_`，最后把连续报文交给 transport——**两跳拷贝**；
- CycloneDDS：`nn_xmsg` 只保存 `serdata` 的指针 + `ddsrt_iovec_t`，`nn_xpack` 把多个 iovec 拼给 `sendmsg`——**零中间拷贝**。

CycloneDDS 的写路径（`rmw_cyclonedds_cpp` 的 `serdata_rmw`）：

```cpp
// rmw_cyclonedds_cpp/src/serdata.cpp
static void serialize_into_serdata_rmw(serdata_rmw * d, const void * sample)
{
  ...
  size_t sz = type->cdr_writer->get_serialized_size(sample);
  d->resize(sz);
  type->cdr_writer->serialize(d->data(), sample);
}
```

`resize()` 每次 `new byte[]`（不复用），随后 `to_ser_ref` 只交出指针：

```cpp
// rmw_cyclonedds_cpp/src/serdata.cpp
static struct ddsi_serdata * serdata_rmw_to_ser_ref(
  const struct ddsi_serdata * dcmn, size_t off, size_t sz, ddsrt_iovec_t * ref)
{
  auto d = static_cast<const serdata_rmw *>(dcmn);
  serialize_into_serdata_rmw_on_demand(const_cast<serdata_rmw *>(d));
  ref->iov_base = byte_offset(d->data(), off);
  ref->iov_len = (ddsrt_iov_len_t) sz;
  return ddsi_serdata_ref(d);
}
```

```c
/* CycloneDDS src/core/ddsi/src/q_xmsg.c */
void nn_xmsg_serdata(struct nn_xmsg *m, struct ddsi_serdata *serdata,
                     size_t off, size_t len, struct ddsi_writer *wr)
{
  ...
  m->refd_payload = ddsi_serdata_to_ser_ref(serdata, off, len4, &m->refd_payload_iov);
}
```

```c
/* 同文件：nn_xpack_addmsg() */
if (m->refd_payload)
{
  xp->iov[niov] = m->refd_payload_iov;
  sz += m->refd_payload_iov.iov_len;
  niov++;
}
```

```c
/* src/core/ddsi/src/ddsi_udp.c */
ddsrt_msghdr_t msg = {
  .msg_name = &dstaddr.x,
  .msg_namelen = (socklen_t) ddsrt_sockaddr_get_size (&dstaddr.a),
  .msg_iov = (ddsrt_iovec_t *) iov,
  .msg_iovlen = (ddsrt_msg_iovlen_t) niov
};
...
rc = ddsrt_sendmsg(conn->m_sock, &msg, sendflags, &nsent);
```

接收侧则必然有一次“网络缓冲 → serdata”的拷贝：

```cpp
// rmw_cyclonedds_cpp/src/serdata.cpp
static struct ddsi_serdata * serdata_rmw_from_ser(..., const struct nn_rdata * fragchain, size_t size)
{
  ...
  d->resize(size);
  auto cursor = d->data();
  while (fragchain) {
    ...
    memcpy(cursor, src, n_bytes);
    ...
  }
}
```

对比结论：

| 环节 | Fast DDS 2.14 | CycloneDDS 0.10.x |
|---|---|---|
| 序列化 | 写进 payload 池（复用） | 写进 `serdata_rmw` 新 `new byte[]`（每帧分配） |
| 报文组装 | `addData` + `appendMsg` 两跳拷贝 | `xmsg/xpack` 只存 iovec，零拷贝 |
| 发送 | 连续 `full_msg_` → SHM/UDP | iovec → 一次 `sendmsg` |
| 接收 | 段内包装 + reader 池拷贝 → 反序列化 | `recvmsg` → `from_ser` memcpy → 反序列化 |
| 零拷贝写 | loan（仅 plain）+ 默认 data-sharing off | iceoryx loan + 固定大小 + `use_only_iceoryx` |
| 默认是否启用 | 是（Jazzy 默认 RMW） | 否，需 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |

**工程含义**：

- CycloneDDS 省掉了 Fast DDS 报文组装的两跳拷贝，对大消息的 CPU/内存带宽更友好；代价是 `serdata_rmw::resize()` 每帧 `new`，分配压力更大（Fast DDS 用 payload 池复用）。
- 两者的“零拷贝”都不是默认给任意 ROS 消息的：Fast DDS 受 `is_plain` 限制，CycloneDDS 受 `fixed_size` + iceoryx + `use_only_iceoryx` 限制；`Image`/`PointCloud2` 都不满足。
- 如果瓶颈是“组装拷贝”，换 CycloneDDS 可能有效；如果瓶颈是“反序列化分配”或“带宽”，换 RMW 不会改变量级。

---

## 4. 跨机通信（UDP）

### 4.1 分片

当序列化后的 payload 超过 writer 的最大数据尺寸时，`WriterHistory::set_fragments()` 在 `CacheChange_t` 上打分片标记：

```cpp
// Fast-DDS 2.14.x src/cpp/rtps/history/WriterHistory.cpp
// If it is big data, fragment it.
if (change->serializedPayload.length > final_high_mark_for_frag)
{
    // Fragment the data.
    change->setFragmentSize(static_cast<uint16_t>(
        (std::min)(final_high_mark_for_frag, RTPSMessageGroup::get_max_fragment_payload_size())));
}
```

分片上限：

```cpp
// Fast-DDS 2.14.x include/fastdds/rtps/messages/RTPSMessageGroup.h
static inline constexpr uint32_t get_max_fragment_payload_size()
{
    return std::numeric_limits<uint16_t>::max() - data_frag_header_size_ - max_inline_qos_size_ - 3;
}
// data_frag_header_size_ = 28, max_inline_qos_size_ = 32
// 65535 - 28 - 32 - 3 = 65472 字节
```

实际 fragment_size 取 `min(getMaxDataSize(), 65472)`，通常受传输层 `max_message_size` 约束而更小。发送时 `deliver_sample_to_network()` 按 fragment 循环调用 `group.add_data_frag()`，每个 fragment 都走一次报文组装和 transport 发送。

**延迟含义**：

- 一个 4 MB 消息在 64 KiB 分片下约 64 个 fragment，每个 fragment 都要组装、发送、确认；
- 接收端要重组，重组通常意味着把 fragment 写进一个完整的 `CacheChange` payload（又一次整块拷贝）；
- 只要有一个 fragment 丢失/乱序，可靠 QoS 会触发 NACK/重传，尾延迟显著上升。

### 4.2 可靠性：HEARTBEAT / ACKNACK 与默认周期

`StatefulWriter` 默认是“推送 + 可靠”：

```cpp
// Fast-DDS 2.14.x include/fastdds/rtps/attributes/WriterAttributes.h
struct WriterTimes
{
    //! Initial heartbeat delay. Default value ~11ms.
    Duration_t initialHeartbeatDelay;
    //! Periodic HB period, default value 3s.
    Duration_t heartbeatPeriod;
    //!Delay to apply to the response of a ACKNACK message, default value ~5ms.
    Duration_t nackResponseDelay;
    ...
    WriterTimes()
    {
        initialHeartbeatDelay.nanosec = 12 * 1000 * 1000;
        heartbeatPeriod.seconds = 3;
        nackResponseDelay.nanosec = 5 * 1000 * 1000;
    }
```

- 正常路径：writer 主动推送 DATA；reader 通过 ACKNACK 确认，丢失时请求重传。
- `heartbeatPeriod = 3 s` 意味着**周期性心跳很稀**；它主要影响“末尾丢包/对端无数据可推时”的恢复延迟，而不是每个消息的常规延迟。真正丢包时，reader 发现序号缺口会立即发 ACKNACK，writer 在 `nackResponseDelay`（约 5 ms）后响应。
- `disable_positive_acks = true` 可减少正向 ACK 流量，适合高频、可容忍丢失的场景；它不改变单条消息的首次传输路径。
- 历史深度不足/`max_blocking_time` 到期时，`write()` 可能阻塞或失败——这是“可靠性 + 队列”造成的尾延迟来源。

### 4.3 同步 vs 异步发布

```cpp
// Fast-DDS 2.14.x src/cpp/rtps/writer/RTPSWriter.cpp
: is_async_(att.mode == SYNCHRONOUS_WRITER ? false : true)
```

默认 `WriterAttributes::mode = SYNCHRONOUS_WRITER`。同步模式下，`write()` 在调用线程完成序列化、入历史、组装、发送；异步模式把“发送”交给异步线程/流控器：

- 同步：`publish()` 返回时数据已交给 transport，**调用线程承担序列化 + 发送开销**，但延迟可预测；
- 异步：`publish()` 更快返回，发送被排队/合批，**增加调度与排队延迟**，吞吐可能更高。

控制链路通常应保持同步；高频率大消息、且能容忍额外排队时才考虑异步。

### 4.4 网络与内核

跨机路径在 transport 之后还有：

- `sendmsg` 把 RTPS 报文从用户缓冲拷进内核 socket 缓冲；
- 网卡/驱动、交换机、接收端内核缓冲、`recvmsg` 再拷回用户态；
- MTU 1500 时，一个 64 KiB 的 RTPS 报文还要被 IP 分片（或依赖 GSO/GRO），增加丢包放大效应；
- 1 Gbps 链路对 4 MB 有效载荷的理论下限约 `4 MB / 125 MB/s ≈ 32 ms`，这还没算协议头、分片、重传和接收端反序列化。

因此跨机大消息的第一优化目标通常是**减少字节数**（缩放、ROI、降采样、压缩），而不是换中间件。

补充：首次通信前还要完成发现（SPDP 多播/初始 peer、SEDP 端点匹配）。发现成本主要落在启动、重连、节点动态加入时，不影响稳态单帧延迟；但在大节点图或无线网络里，重新匹配可能造成秒级空窗，测量端到端延迟时应把“首帧”和“稳态”分开统计。

---

## 5. 序列化与反序列化的真实成本

### 5.1 生成代码逐字段写

`cdr_serialize` 由 `rosidl_typesupport_fastrtps_cpp` 生成，对每个字段执行 `cdr << field`；字符串写长度 + 字节，`sequence` 写长度 + 逐元素（对 POD 数组通常是一次 `serialize_array` 循环/memcpy）。它不是“把整块结构体 memcpy 过去”，而是**按 CDR 规则逐字段编码**，包含对齐、字节序、长度前缀。

### 5.2 尺寸预扫描被算了两遍

非 plain 类型每次 publish 会遍历整条消息两次：

- 一次是 `getSerializedSizeProvider(data)`，为 payload 池确定尺寸；
- 一次是 `cdr_serialize` 真正写入。

```cpp
// rmw_fastrtps_cpp/src/type_support_common.cpp
size_t TypeSupport::getEstimatedSerializedSize(const void * ros_message, const void * impl) const
{
  if (is_plain_) {
    return m_typeSize;
  }
  ...
  return 4 + callbacks->get_serialized_size(ros_message);
}
```

对含大 `sequence` 的消息，这是一次额外的全量遍历（纯读，但会触缓存）。

### 5.3 反序列化的分配点

- 消息对象本身：默认 `borrow_message()` 每帧 `allocate_shared`；
- 每个 `string`：一次分配 + 拷贝；
- 每个 `sequence`：`resize(size)`（可能一次分配/扩容）+ 整块写入；
- 嵌套消息：递归。

对 `Image`/`PointCloud2`，`data` 字段的分配和拷贝是接收侧最大的一项固定成本；它发生在 `cdr_deserialize` 内，和传输层是否共享内存无关。

### 5.4 serialized message 路径

- 发布 `rcl_publish_serialized_message()`：跳过逐字段序列化，但 `TypeSupport::serialize()` 的 `CDR_BUFFER` 分支会 `memcpy` 一次进 payload；
- 订阅 `rcl_take_serialized_message()`：`CDR_BUFFER` 分支先 `reserve` 再 `memcpy`，rmw 侧一次，`rmw_take` 再 `memcpy` 到 `serialized_message->buffer`，共两次；
- 适合“只转发、不解码”的桥接节点，不适合需要反序列化后再处理的算法节点。

### 5.5 loan / zero-copy 的真实边界

`rmw_fastrtps` 的 loan 路径在**发布侧**确实是零拷贝：

```cpp
// Fast-DDS 2.14.x src/cpp/fastdds/publisher/DataWriterImpl.cpp（loan_sample）
if (!type_->is_plain(data_representation_) ||
    SerializedPayload_t::representation_header_size > type_->m_typeSize)
{
    return ReturnCode_t::RETCODE_ILLEGAL_OPERATION;
}
...
// Sample starts after representation header
sample = payload.payload.data + SerializedPayload_t::representation_header_size;
```

发布时 `check_and_remove_loan(data, payload)` 命中，就跳过取池与序列化；订阅时 `GenericSequence::resize()` 直接抛 `bad_alloc` 强制走 loan，`loaned_message` 指向 reader 池中的样本，用户原位读，省掉“反序列化到用户消息”这一次拷贝。

**但它不是端到端零拷贝**：发布侧仍有 RTPS 组装的两跳拷贝与 SHM 段拷贝；接收侧 `StatefulReader::processDataMsg` 仍会把 payload 复制进 reader 池。loan 省掉的是“序列化 + 发布侧取池”和“接收侧反序列化”，不是“线上字节的搬运”。真正的端到端零拷贝是 Data-sharing（reader 直接引用 writer 历史中的 payload），而 ROS 2 默认关闭它。

但限制很硬：

1. `can_loan_messages = type_support_->is_plain()`；生成器判定只要含 `sequence<>` 或无界字符串就 `is_plain = false`；
2. 订阅端 `rcl_subscription_get_default_options()` 默认 `disable_loaned_message = true`，需要 `RCL_DISABLE_LOANED_MESSAGES=0` 或手动关闭；
3. `data_sharing().off()` 后没有 payload 共享池的零拷贝；loan 只省掉“发布侧取池 + 序列化”和“接收侧反序列化”，不省 RTPS 组装、SHM 段拷贝、reader 池拷贝；
4. `LoanManager` 限制在发 loan 数量，未归还的 loan 会阻止 reader 删除。

**结论**：`sensor_msgs/msg/Image`、`PointCloud2` 这类含变长数组的消息在 Jazzy 默认 `rmw_fastrtps` 下**不能 loan**，也就没有应用层零拷贝；若确实需要，只能自定义固定长度数组的消息类型（plain），或换用支持共享内存零拷贝的中间件/传输。

---

## 6. 全链路拷贝总表

> 以一条 4 MB 的 `sensor_msgs/msg/Image` 为例。“整块拷贝”指对约 4 MB 数据做一次 memcpy/读写；“分配”指每帧堆分配。

| 路径 | 序列化 | 发送侧整块拷贝 | 传输/接收侧拷贝 | 接收侧反序列化 | 每帧分配 | 备注 |
|---|---|---|---|---|---|---|
| 进程内，`publish(unique_ptr)`，回调 `shared_ptr<const T>`/`const T&` | 0 | 0 | 0 | 0 | 0（仅 `take_data` 小 pair） | 唯一真正零拷贝路径 |
| 进程内，`publish(const T&)` | 0 | 1 | 0 | 0 | 1 条消息 | 发布端先复制一份 |
| 进程内，存在 N 个 `unique_ptr` 回调 | 0 | N−1 | 0 | 0 | N−1 条消息 | 独占所有权语义导致 |
| 同机跨进程，SHM，未分片 | 1 次写 | 3（组装 2 + 段 1） | 1（reader 池） | 1 次写 | 1 条消息 + data 缓冲 | 约 6 遍内存流量 |
| 同机跨进程，UDP loopback | 1 次写 | 2（组装） | 2（内核收发）+ 1（reader 池） | 1 次写 | 1 条消息 + data 缓冲 | 约 7 遍 |
| 跨机，UDP，未分片 | 1 次写 | 2（组装） | 2（内核收发）+ 1（reader 池） | 1 次写 | 1 条消息 + data 缓冲 | 约 7 遍，再加网络 |
| 跨机，UDP，分片 | 1 次写 | 2×fragment 数 | 2×fragment 数 + 1（reader 池）+ 重组 | 1 次写 | 同上 | 重组与重传放大尾延迟 |
| CycloneDDS，UDP，未分片 | 1 次写 | 0（iovec 引用） | 2（内核收发）+ 1（from_ser） | 1 次写 | 1 块 `serdata` + 1 条消息 + data 缓冲 | 约 5 遍；无组装拷贝 |
| CycloneDDS，iceoryx，可变长类型 | 1 次写（进 chunk） | 0 | 1（rmw 搬进 `m_data`） | 1 次写 | chunk + 消息 | rmw 层没有 chunk 直交用户 |
| loaned message（plain 类型） | 0 | 2（组装） | 1（SHM 段）+ 1（reader 池） | 0（用户原位读） | 0 | 发布侧零拷贝，非端到端 |
| serialized message 发布/订阅 | 0（应用已序列化） | 2（组装）+ 1（CDR_BUFFER memcpy） | 1（SHM 段）+ 1（reader 池） | 2 次 memcpy（take 侧） | 序列化缓冲 | 适合转发 |

### 6.1 数量级示例（假设，不是实测）

以 4 MB 的 `Image`、同机 SHM 普通路径为例，假设单线程 memcpy 有效带宽约 8–12 GB/s（Orin 上的保守量级）：

| 操作 | 内存流量（读+写） | 估算耗时 |
|---|---|---|
| 序列化写 payload | 4 MB 读 + 4 MB 写 | 0.7–1.0 ms |
| `addData` 拷进 submessage | 4 MB + 4 MB | 0.7–1.0 ms |
| `appendMsg` 拷进 full message | 4 MB + 4 MB | 0.7–1.0 ms |
| SHM 段 memcpy | 4 MB + 4 MB | 0.7–1.0 ms |
| reader 池 `get_payload` 拷贝 | 4 MB + 4 MB | 0.7–1.0 ms |
| 反序列化（含 4 MB 分配 + 写） | 4 MB 读 + 4 MB 写 | 0.7–1.5 ms |
| **合计** | **约 48 MB** | **约 4.2–6.5 ms** |

对照：

- 同机 SHM 的“传输”本身包含两次 4 MB 拷贝（段 + reader 池，约 0.8–1.0 ms），**真正的大头仍是序列化/反序列化和报文组装**；
- 跨机 1 Gbps 的传输下限约 32 ms，此时网络是主项，但上述 CPU 开销仍然叠加；
- 若改成进程内零拷贝，这 4–6 ms 基本消失，只剩入队/唤醒/回调调度。

所以“同机换共享内存中间件”通常只能省掉 1 ms 量级；要显著降延迟，优先消除序列化/反序列化（进程内、serialized 转发、或固定大小 plain 类型 + loan）。

---

## 7. 延迟影响因子清单

### 7.1 数据本身

| 因子 | 影响 | 备注 |
|---|---|---|
| 消息字节数 | 线性影响序列化、拷贝、带宽 | 4 MB 图像在 1 Gbps 上仅传输就 ≥32 ms |
| 字段数量/嵌套深度 | 影响逐字段编码与对齐 | 大量小字段比同样字节数的 `uint8[]` 慢 |
| 字符串/变长数组 | 反序列化时分配 + 拷贝 | `Image::data`、`PointCloud2::data` 是典型 |
| 是否 plain | 决定能否 loan、尺寸是否可预计算 | `is_plain=false` 还会导致尺寸扫描两遍 |

### 7.2 进程布局与路径选择

| 因子 | 影响 | 源码依据 |
|---|---|---|
| 同进程 + intra-process | 去掉序列化/DDS/传输 | `publisher.hpp`、IPM |
| 混合进程内/跨进程订阅 | 触发重复投递，DDS 副本被丢弃但成本已付 | `take_type_erased` |
| TransientLocal | 强制走 DDS（即使全进程内） | `buffer_` 非空 |
| 同机 SHM vs loopback UDP | SHM 少两次内核拷贝，但仍有组装/段拷贝 | `SharedMemTransport` |
| Data-sharing | 默认关闭 | `data_sharing().off()` |

### 7.3 QoS

| 因子 | 影响 |
|---|---|
| Reliability = RELIABLE | 丢包时重传；`max_blocking_time`/历史满会阻塞 |
| History depth | 深度大 → 排队延迟大；深度小 → 丢帧 |
| `heartbeatPeriod`（默认 3 s） | 影响尾部丢包/空闲后的恢复 |
| `nackResponseDelay`（默认 5 ms） | 影响 NACK 重传响应 |
| `disable_positive_acks` | 减少 ACK 流量，适合高频可丢场景 |
| `publish_mode` | 同步：调用线程发送；异步：排队/合批，增加延迟但提高吞吐 |

### 7.4 执行器与操作系统

| 因子 | 影响 |
|---|---|
| 回调组（MutuallyExclusive） | 同一组串行，关键回调被阻塞会拉长尾延迟 |
| executor 取 executable 顺序 | timer 优先于 subscription，waitable（进程内）最后 |
| spin 方式与超时 | `spin_some`/带超时的 `rcl_wait` 会把唤醒延迟加到每个消息上；`spin` 常驻等待更稳 |
| 线程优先级/CPU 绑核 | 影响唤醒与序列化线程的调度抖动 |
| 缺页/内存带宽 | 每帧新分配大 `data` 缓冲会触发缺页与清零 |
| 日志/tracepoint | 高频 trace 有开销，测量时需区分 |

### 7.5 网络（跨机）

| 因子 | 影响 |
|---|---|
| 带宽与 MTU | 1 Gbps/1500 MTU 是硬约束；jumbo frame 减少分片 |
| 丢包/重传 | 可靠 QoS 下尾延迟放大 |
| 交换机/无线 | 抖动、乱序 |
| 内核 socket 缓冲 | 过小导致丢包，过大增加排队延迟 |
| 发现（SPDP） | 主要影响启动/重连，不影响稳态单帧 |

---

## 8. 优化建议（按数据链）

### 8.1 进程内

1. **发布端用 `std::move` / `unique_ptr`**；在**开启 intra-process** 的发布端不要用 `publish(const T&)`，后者必拷一次。若发布端不开启 intra-process，`const T&` 反而是零拷贝的正确入口（rclcpp 直接把引用交给 rmw）。
2. **订阅回调优先 `shared_ptr<const T>` 或 `const T&`**，避免 `unique_ptr<T>` 回调导致 N−1 次拷贝。
3. **`depth=1`** 用于“只要最新值”的传感器/状态；但要接受覆盖丢帧。需要完整序列时增大 depth 并接受排队延迟。
4. **不要把 intra-process 当成跨进程优化**：它只在同一进程、同一 Context 内生效。
5. **警惕混合订阅**：如果 topic 同时有跨进程订阅者，进程内订阅者会收到 DDS 副本并在反序列化后被丢弃；此时可考虑把该 topic 拆成“进程内专用”和“跨进程专用”，或接受重复成本。

### 8.2 同机跨进程

1. **确认是否真的走了 SHM**：`ros2 daemon`/Fast DDS 日志、`FASTRTPS_DEFAULT_PROFILES_FILE` 配置 transport；SHM 段大小需 ≥ `max_message_size`。
2. **不要指望默认 Data-sharing**：`rmw_fastrtps` 显式 `data_sharing().off()`；若要开启，需要非默认 QoS 路径，并验证 reader/writer 双方。
3. **大消息优先减少字节数**：缩放/ROI/降采样/压缩，收益通常大于换传输。
4. **减少接收侧分配**：默认 `MessageMemoryStrategy::borrow_message()` 每帧新分配；可通过自定义订阅/内存策略复用消息（非默认公开 API，需要改框架或用支持池化的 RMW），或改用 loan（仅 plain 类型）。
5. **序列化转发用 serialized message**：只转发不解码的桥接节点可跳过逐字段序列化/反序列化。

### 8.3 跨机

1. **带宽预算先行**：`消息字节数 × 频率` 与链路容量比较；1 Gbps 实际有效吞吐约 110–118 MB/s。
2. **可靠 vs 尽力**：高频图像/点云用 BestEffort + 应用层丢帧策略；控制指令用 Reliable 并设置合理 depth。
3. **分片与 MTU**：尽量让单条消息 < 64 KiB，减少 RTPS 分片与 IP 分片；或启用 jumbo frame。
4. **心跳/重传调优**：高动态网络可缩短 `heartbeatPeriod`、调整 `nackResponseDelay`；但会增加控制流量。
5. **同步发布**用于控制链路；异步发布用于高吞吐、可容忍排队的链路。

### 8.4 执行器

1. 关键控制回调放独立 `MutuallyExclusive` 回调组 + 独立 executor/线程；
2. 避免在关键回调里做日志、磁盘 I/O、长计算；
3. 注意 timer 优先于 subscription、waitable 最后取；进程内路径是 waitable，可能被 timer 抢占；
4. 用 `ros2_tracing` 的 `rclcpp_publish`、`rclcpp_take`、`rmw_publish` 区分各段耗时。

### 8.5 测量与验证

1. **分段打点**：在 `publish()` 前后、回调入口/出口用同一时钟打时间戳；不要只测“发布到回调”的总和，否则无法区分序列化、传输、调度。
2. **利用 tracepoint**：`rclcpp_publish`（发布入口）、`rclcpp_take`（取数）、`rclcpp_intra_publish`（进程内发布）、`rmw_publish`（rmw 层）、`rclcpp_ring_buffer_enqueue/dequeue`（进程内缓冲）都已内置；用 `ros2_tracing`/LTTng 或 `tracetools` 采集。
3. **验证拷贝假设**：用 `perf stat` 看 cache-miss/内存带宽；用 `strace -c` 看 `sendmsg/recvmsg` 次数；用 `bpftrace` 统计 `memcpy` 调用（大消息时尤其明显）。
4. **A/B 必须同帧**：同一数据源、同一 QoS、同一进程布局、同一 CPU 频率；比较 P50/P95/P99 与最大观测值，而不是只看均值。
5. **区分“发布端开销”与“端到端延迟”**：`publish()` 返回早不代表对端回调早；跨机时还要加网络与接收端调度。

### 8.6 常见误区

| 误区 | 事实 | 源码依据 |
|---|---|---|
| “用了共享内存就没有拷贝” | SHM transport 仍要 memcpy 进段；组装阶段还有拷贝 | `copy_to_shared_buffer`、`addSubmessageData` |
| “开了 intra-process 就零拷贝” | `publish(const T&)` 必拷一次；owning 回调导致 N−1 次 | `publisher.hpp`、IPM |
| “loaned message 可以用于 Image” | 含 `sequence<>` 即 `is_plain=false`，不能 loan | `max_serialized_size` 判定、`can_loan_messages` |
| “QoS depth 只影响丢包” | 也影响排队延迟；depth 越大，旧帧等待越久 | IPM 环形缓冲容量 = depth；DDS 历史队列 |
| “换中间件一定降延迟” | 换 RMW 改变拷贝结构，但不改变序列化/反序列化/带宽量级 | 本文 §3.7、§6 |

---

## 9. 源码索引

| 主题 | 仓库/文件 | 符号 |
|---|---|---|
| 发布入口 | `rclcpp/include/rclcpp/publisher.hpp` | `Publisher::publish(const T&)`、`publish(unique_ptr)`、`do_inter_process_publish` |
| 进程内管理 | `rclcpp/include/rclcpp/experimental/intra_process_manager.hpp` | `do_intra_process_publish`、`add_shared_msg_to_buffers`、`add_owned_msg_to_buffers` |
| 进程内缓冲 | `rclcpp/include/rclcpp/experimental/buffers/ring_buffer_implementation.hpp` | `RingBufferImplementation::enqueue/dequeue` |
| 缓冲创建 | `rclcpp/include/rclcpp/experimental/create_intra_process_buffer.hpp` | `create_intra_process_buffer` |
| 进程内订阅 | `rclcpp/include/rclcpp/experimental/subscription_intra_process.hpp` | `take_data`、`execute_impl` |
| 订阅取数 | `rclcpp/src/rclcpp/subscription_base.cpp` | `take_type_erased`、`matches_any_intra_process_publishers` |
| 消息内存策略 | `rclcpp/include/rclcpp/message_memory_strategy.hpp` | `borrow_message` |
| rcl 发布 | `rcl/src/rcl/publisher.c` | `rcl_publish`、`rcl_publish_serialized_message`、`rcl_publish_loaned_message` |
| rmw 发布 | `rmw_fastrtps_shared_cpp/src/rmw_publish.cpp` | `__rmw_publish`、`__rmw_publish_serialized_message`、`__rmw_publish_loaned_message` |
| 序列化 | `rmw_fastrtps_shared_cpp/src/TypeSupport_impl.cpp` | `TypeSupport::serialize`、`deserialize` |
| 生成代码桥接 | `rmw_fastrtps_cpp/src/type_support_common.cpp` | `serializeROSmessage`、`deserializeROSmessage`、`getEstimatedSerializedSize` |
| 写路径 | `Fast-DDS 2.14.x src/cpp/fastdds/publisher/DataWriterImpl.cpp` | `perform_create_new_change`、`loan_sample` |
| 历史/分片 | `Fast-DDS 2.14.x src/cpp/rtps/history/WriterHistory.cpp` | `add_change_`、`set_fragments` |
| 可靠性 | `Fast-DDS 2.14.x src/cpp/rtps/writer/StatefulWriter.cpp` | `unsent_change_added_to_history`、`deliver_sample_to_network` |
| 报文组装 | `Fast-DDS 2.14.x src/cpp/rtps/messages/RTPSMessageGroup.cpp` | `add_data`、`append_message` |
| SHM | `Fast-DDS 2.14.x src/cpp/rtps/transport/shared_mem/SharedMemTransport.cpp` | `copy_to_shared_buffer`、`send`、`init` |
| SHM 接收包装 | `Fast-DDS 2.14.x src/cpp/rtps/transport/shared_mem/SharedMemChannelResource.hpp`、`ReceiverResource.cpp` | `perform_listen_operation`、`OnDataReceived`（`msg.wraps = true`） |
| reader 池拷贝 | `Fast-DDS 2.14.x src/cpp/rtps/reader/StatefulReader.cpp` | `processDataMsg`（`payload_pool_->get_payload`） |
| 默认周期 | `Fast-DDS 2.14.x include/fastdds/rtps/attributes/WriterAttributes.h` | `WriterTimes`、`WriterAttributes` |
| rmw QoS 覆盖 | `rmw_fastrtps_cpp/src/publisher.cpp`、`subscription.cpp` | `data_sharing().off()`、`PREALLOCATED_WITH_REALLOC_MEMORY_MODE` |
| CycloneDDS 序列化 | `rmw_cyclonedds_cpp/src/serdata.cpp` | `serialize_into_serdata_rmw`、`serdata_rmw_to_ser_ref`、`serdata_rmw_from_ser`、`serdata_rmw_to_sample` |
| CycloneDDS 报文引用 | `cyclonedds src/core/ddsi/src/q_xmsg.c` | `nn_xmsg_serdata`、`nn_xpack_addmsg` |
| CycloneDDS 发送 | `cyclonedds src/core/ddsi/src/ddsi_udp.c` | `ddsi_udp_conn_write`（`ddsrt_sendmsg`） |
| CycloneDDS iceoryx | `cyclonedds src/core/ddsi/src/ddsi_shm_transport.c` | `from_iox` / `iox_pub_publish_chunk` |

---

## 10. 参考资料

- ROS 2 Jazzy `rclcpp`：https://github.com/ros2/rclcpp/tree/jazzy
- ROS 2 Jazzy `rcl`：https://github.com/ros2/rcl/tree/jazzy
- `rmw_fastrtps`：https://github.com/ros2/rmw_fastrtps/tree/jazzy
- Fast DDS 2.14.x：https://github.com/eProsima/Fast-DDS/tree/2.14.x
- Fast CDR 2.2.x：https://github.com/eProsima/Fast-CDR/tree/2.2.x
- CycloneDDS 0.10.x：https://github.com/eclipse-cyclonedds/cyclonedds/tree/releases/0.10.x
- `rmw_cyclonedds`：https://github.com/ros2/rmw_cyclonedds/tree/jazzy
- iceoryx 2.0.x：https://github.com/eclipse-iceoryx/iceoryx/tree/release_2.0
- ROS 2 Jazzy `ros2.repos`（版本基线）：https://github.com/ros2/ros2/blob/jazzy/ros2.repos
