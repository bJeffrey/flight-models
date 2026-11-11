#include "flight_models/runtime_transport.h"

#include <boost/asio.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast.hpp>
#include <nats/nats.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

namespace flight_models {
namespace {
namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;
using Json = nlohmann::json;

std::string required_string(const Json& value, const char* key) {
  const auto& item = value.at(key);
  if (!item.is_string() || item.get<std::string>().empty())
    throw std::invalid_argument(std::string("transport.") + key + " must be a nonempty string");
  return item.get<std::string>();
}

std::size_t positive_limit(const Json& value, const char* key) {
  const auto number = value.at(key).get<std::int64_t>();
  if (number <= 0) throw std::invalid_argument(std::string("transport.") + key + " must be positive");
  return static_cast<std::size_t>(number);
}

struct Address { std::string host, port, target; };
Address parse_ws(const std::string& url) {
  const auto scheme = url.find("://");
  if (scheme == std::string::npos || url.substr(0, scheme) != "ws")
    throw std::invalid_argument("PRD asb_url must use ws://");
  const auto start = scheme + 3;
  const auto slash = url.find('/', start);
  const auto authority = url.substr(start, slash == std::string::npos ? slash : slash - start);
  const auto colon = authority.rfind(':');
  if (authority.empty()) throw std::invalid_argument("PRD asb_url has no host");
  return {colon == std::string::npos ? authority : authority.substr(0, colon),
          colon == std::string::npos ? "80" : authority.substr(colon + 1),
          slash == std::string::npos ? "/" : url.substr(slash)};
}

void nats_check(natsStatus status, const std::string& where) {
  if (status != NATS_OK) throw std::runtime_error(where + ": " + natsStatus_GetText(status));
}

std::string ordered_event_subject(const std::string& clock, const std::string& request) {
  // One bounded inbox preserves broker order across both exact subjects.
  // Keep the subscription within their shared NATS token prefix.
  std::size_t clock_start = 0;
  std::size_t request_start = 0;
  std::size_t prefix_end = 0;
  while (clock_start < clock.size() && request_start < request.size()) {
    const auto clock_end = clock.find('.', clock_start);
    const auto request_end = request.find('.', request_start);
    const auto clock_token_end = clock_end == std::string::npos ? clock.size() : clock_end;
    const auto request_token_end = request_end == std::string::npos ? request.size() : request_end;
    const auto clock_token = clock.substr(clock_start, clock_token_end - clock_start);
    const auto request_token = request.substr(request_start, request_token_end - request_start);
    if (clock_token != request_token) break;
    prefix_end = clock_token_end;
    if (clock_end == std::string::npos || request_end == std::string::npos) {
      // A wildcard suffix must match at least one token after its dot.
      if (clock_end != request_end) {
        const auto previous_dot = clock.rfind('.', prefix_end);
        prefix_end = previous_dot == std::string::npos ? 0 : previous_dot;
      }
      break;
    }
    clock_start = clock_end + 1;
    request_start = request_end + 1;
  }
  if (prefix_end == 0) throw std::invalid_argument("runtime event subjects have no bounded common prefix");
  return clock.substr(0, prefix_end) + ".>";
}
}  // namespace

struct RuntimeTransport::Impl {
  struct ReportRoute { std::string url, topic; };
  struct Peer : std::enable_shared_from_this<Peer> {
    struct Item { std::string topic, payload; std::size_t reserved_bytes{}; };
    Impl& owner;
    std::string url, service;
    asio::strand<asio::io_context::executor_type> strand;
    tcp::resolver resolver;
    websocket::stream<beast::tcp_stream> socket;
    asio::steady_timer timer;
    asio::steady_timer write_timer;
    beast::flat_buffer read_buffer;
    std::deque<Item> unsent;
    struct PendingAck { std::size_t bytes; std::chrono::steady_clock::time_point sent; };
    std::deque<PendingAck> awaiting;
    std::string write_buffer;
    Address address;
    bool ready{}, writing{};
    Peer(Impl& parent, std::string endpoint, std::string id)
        : owner(parent), url(std::move(endpoint)), service(std::move(id)),
          strand(asio::make_strand(parent.io)), resolver(parent.io), socket(parent.io), timer(parent.io),
          write_timer(parent.io) {
      socket.read_message_max(static_cast<std::uint64_t>(owner.max_bytes));
    }
    void connect() {
      // Resolve, connect, negotiate INIT, and wait for INFO before declaring readiness.
      address = parse_ws(url);
      resolver.async_resolve(address.host, address.port, asio::bind_executor(strand,
        [self=shared_from_this()](beast::error_code ec, tcp::resolver::results_type endpoints) {
          if (ec) return self->owner.fail("OWP resolve " + self->url + ": " + ec.message());
          beast::get_lowest_layer(self->socket).expires_after(self->owner.op_timeout);
          beast::get_lowest_layer(self->socket).async_connect(endpoints,
            asio::bind_executor(self->strand, [self](beast::error_code err, const tcp::endpoint&) {
              if (err) return self->owner.fail("OWP connect " + self->url + ": " + err.message());
              beast::get_lowest_layer(self->socket).expires_never();
              self->socket.set_option(websocket::stream_base::decorator([](websocket::request_type& req) {
                req.set("Sec-WebSocket-Protocol", "owp");
              }));
              self->socket.async_handshake(self->address.host, self->address.target,
                asio::bind_executor(self->strand, [self](beast::error_code handshake_error) {
                  if (handshake_error) return self->owner.fail("OWP handshake " + self->url + ": " + handshake_error.message());
                  self->control("INIT " + Json{{"versions", {"1.0"}}, {"schema", "002.5.0"}, {"verbose", true},
                    {"service_id", self->service}}.dump(), [self](std::string ack) {
                    if (ack.rfind("+OK", 0) != 0) return self->owner.fail("OWP INIT " + self->url + ": " + ack);
                    self->read_control([self](std::string info) {
                      if (info.rfind("INFO ", 0) != 0) return self->owner.fail("OWP INIT expected INFO from " + self->url);
                      self->ready = true;
                      self->owner.peer_ready();
                      self->read();
                      self->write_next();
                    });
                  });
                }));
            }));
        }));
      deadline("OWP startup " + url);
    }
    void deadline(std::string context) {
      timer.expires_after(owner.op_timeout);
      timer.async_wait(asio::bind_executor(strand, [self=shared_from_this(), context=std::move(context)](beast::error_code ec) {
        if (!ec) self->owner.fail(context + " timed out");
      }));
    }
    void control(std::string text, std::function<void(std::string)> done) {
      write_buffer = std::move(text);
      socket.async_write(asio::buffer(write_buffer), asio::bind_executor(strand,
        [self=shared_from_this(), done=std::move(done)](beast::error_code ec, std::size_t) mutable {
          if (ec) return self->owner.fail("OWP control write " + self->url + ": " + ec.message());
          self->read_control(std::move(done));
        }));
      deadline("OWP control " + url);
    }
    void read_control(std::function<void(std::string)> done) {
      socket.async_read(read_buffer, asio::bind_executor(strand,
        [self=shared_from_this(), done=std::move(done)](beast::error_code ec, std::size_t) mutable {
          if (ec) return self->owner.fail("OWP control read " + self->url + ": " + ec.message());
          self->timer.cancel();
          auto frame = beast::buffers_to_string(self->read_buffer.data());
          self->read_buffer.consume(self->read_buffer.size());
          if (frame.rfind("MSG ", 0) == 0) return self->read_control(std::move(done));
          done(std::move(frame));
        }));
      deadline("OWP acknowledgement " + url);
    }
    void read() {
      socket.async_read(read_buffer, asio::bind_executor(strand, [self=shared_from_this()](beast::error_code ec, std::size_t) {
        if (ec) return self->owner.fail("OWP receive " + self->url + ": " + ec.message());
        auto frame = beast::buffers_to_string(self->read_buffer.data());
        self->read_buffer.consume(self->read_buffer.size());
        if (frame.rfind("+OK", 0) == 0 || frame.rfind("-ERR", 0) == 0) {
          if (self->awaiting.empty()) return self->owner.fail("unexpected OWP ACK " + self->url);
          if (frame.rfind("+OK", 0) != 0) return self->owner.fail("OWP PUB " + self->url + ": " + frame);
          self->owner.release(self->awaiting.front().bytes);
          self->awaiting.pop_front();
          self->arm_ack_deadline();
          self->owner.signal();
          self->write_next();
        }
        self->read();
      }));
    }
    void enqueue(Item item) {
      // Reserve before posting so payloads waiting on the strand are inside the same bound.
      item.reserved_bytes = 5 + item.topic.size() + item.payload.size();
      if (!owner.reserve(item.reserved_bytes)) throw std::runtime_error("OWP pending limit exceeded " + url);
      const auto reserved = item.reserved_bytes;
      try {
        asio::post(strand, [self=shared_from_this(), item=std::move(item)]() mutable {
          self->unsent.push_back(std::move(item));
          self->write_next();
        });
      } catch (...) {
        owner.release(reserved);
        throw;
      }
    }
    void write_next() {
      // Keep one socket write active while allowing earlier PUBs to await ACKs concurrently.
      if (!ready || writing || unsent.empty()) return;
      auto item = std::move(unsent.front());
      unsent.pop_front();
      write_buffer = "PUB " + item.topic + " " + item.payload;
      awaiting.push_back({item.reserved_bytes, std::chrono::steady_clock::now()});
      writing = true;
      write_timer.expires_after(owner.op_timeout);
      write_timer.async_wait(asio::bind_executor(strand, [self=shared_from_this()](beast::error_code ec) {
        if (!ec && self->writing) self->owner.fail("OWP PUB write " + self->url + " timed out");
      }));
      socket.async_write(asio::buffer(write_buffer), asio::bind_executor(strand,
        [self=shared_from_this()](beast::error_code ec, std::size_t) {
          self->writing = false;
          self->write_timer.cancel();
          if (ec) return self->owner.fail("OWP PUB write " + self->url + ": " + ec.message());
          self->arm_ack_deadline();
          self->write_next();
        }));
    }
    void arm_ack_deadline() {
      if (awaiting.empty()) { timer.cancel(); return; }
      timer.expires_at(awaiting.front().sent + owner.op_timeout);
      timer.async_wait(asio::bind_executor(strand, [self=shared_from_this()](beast::error_code ec) {
        if (!ec && !self->awaiting.empty()) self->owner.fail("OWP PUB acknowledgement " + self->url + " timed out");
      }));
    }
    void close() {
      beast::error_code ignored;
      timer.cancel(); write_timer.cancel(); resolver.cancel();
      beast::get_lowest_layer(socket).socket().close(ignored);
    }
  };

  explicit Impl(const Json& config) : work(asio::make_work_guard(io)) {
    const auto& t = config.at("transport");
    nats_url = required_string(t, "nats_url"); clock_subject = required_string(t, "clock_subject");
    if (config.contains("checkpoint")) {
      dis_receipts = std::make_unique<checkpoint_runtime::DeliveryReceiptAccumulator>("dis");
      const auto& checkpoint = config.at("checkpoint");
      checkpoint_request_subject = required_string(checkpoint, "request_subject");
      checkpoint_response_subject = required_string(checkpoint, "response_subject");
      if (checkpoint_request_subject == clock_subject)
        throw std::invalid_argument("checkpoint request subject must differ from clock subject");
      event_subject = ordered_event_subject(clock_subject, checkpoint_request_subject);
    }
    host = required_string(t, "dis_host");
    const auto port = t.at("dis_port").get<int>();
    if (port <= 0 || port > 65535) throw std::invalid_argument("transport.dis_port out of range");
    dis_port = static_cast<unsigned short>(port);
    poll_timeout = std::chrono::milliseconds(positive_limit(t, "clock_poll_timeout_ms"));
    op_timeout = std::chrono::milliseconds(positive_limit(t, "operation_timeout_ms"));
    if (poll_timeout.count() > INT_MAX || op_timeout.count() > INT_MAX)
      throw std::invalid_argument("transport timeout exceeds libnats millisecond range");
    max_messages = positive_limit(t, "pending_message_limit");
    max_bytes = positive_limit(t, "pending_byte_limit");
    service = required_string(t, "owp_service_id");
    for (const auto& entity : config.at("entities")) {
      const auto id = required_string(entity, "id");
      const auto& prd = entity.at("prd");
      if (prd.is_null()) continue;
      reports.emplace(id, ReportRoute{required_string(prd, "asb_url"), required_string(prd, "topic")});
    }
  }
  ~Impl() { stop(); }
  void fail(std::string message) {
    std::lock_guard<std::mutex> lock(mutex);
    if (failure.empty()) failure = std::move(message);
    cv.notify_all();
  }
  void signal() { std::lock_guard<std::mutex> lock(mutex); cv.notify_all(); }
  void check_failure() {
    // Polling status avoids connection callbacks outliving this owner at shutdown.
    if (connection && natsConnection_Status(connection) != NATS_CONN_STATUS_CONNECTED)
      fail("NATS runtime connection is no longer connected");
    if (subscription) {
      std::int64_t dropped = 0;
      const auto status = natsSubscription_GetDropped(subscription, &dropped);
      if (status != NATS_OK) fail(std::string("read NATS runtime drop count: ") + natsStatus_GetText(status));
      else if (dropped > 0) fail("NATS runtime pending limit dropped messages");
    }
    std::lock_guard<std::mutex> lock(mutex);
    if (!failure.empty()) throw std::runtime_error(failure);
  }
  void stop() {
    if (stopped) return;
    stopped = true;
    io.stop();
    if (thread.joinable()) thread.join();
    for (auto& [key, peer] : peers) { (void)key; peer->close(); }
    if (subscription) natsSubscription_Destroy(subscription);
    if (connection) natsConnection_Destroy(connection);
  }
  asio::io_context io;
  asio::executor_work_guard<asio::io_context::executor_type> work;
  std::thread thread;
  asio::ip::udp::socket udp{io};
  asio::ip::udp::endpoint dis_endpoint;
  std::string nats_url, clock_subject, checkpoint_request_subject, checkpoint_response_subject;
  std::string event_subject, host, service;
  unsigned short dis_port{};
  std::chrono::milliseconds poll_timeout, op_timeout;
  std::size_t max_messages{}, max_bytes{};
  std::map<std::string, ReportRoute> reports;
  std::map<std::string, std::shared_ptr<Peer>> peers;
  natsConnection* connection{};
  natsSubscription* subscription{};
  std::mutex mutex;
  std::condition_variable cv;
  std::string failure;
  bool opened{}, stopped{};
  std::size_t ready_peers{};
  std::atomic<std::size_t> pending_messages{}, pending_bytes{};
  std::unique_ptr<checkpoint_runtime::DeliveryReceiptAccumulator> dis_receipts;
  bool reserve(std::size_t bytes) {
    auto count = pending_messages.load();
    do { if (count >= max_messages) return false; }
    while (!pending_messages.compare_exchange_weak(count, count + 1));
    auto used = pending_bytes.load();
    do {
      if (bytes > max_bytes - std::min(used, max_bytes)) { pending_messages.fetch_sub(1); return false; }
    } while (!pending_bytes.compare_exchange_weak(used, used + bytes));
    return true;
  }
  void release(std::size_t bytes) {
    pending_bytes.fetch_sub(bytes);
    pending_messages.fetch_sub(1);
    signal();
  }
  void peer_ready() { std::lock_guard<std::mutex> lock(mutex); ++ready_peers; cv.notify_all(); }
};

RuntimeTransport::RuntimeTransport(const Json& config) : impl_(std::make_unique<Impl>(config)) {}
RuntimeTransport::~RuntimeTransport() = default;
RuntimeTransport::RuntimeTransport(RuntimeTransport&&) noexcept = default;
RuntimeTransport& RuntimeTransport::operator=(RuntimeTransport&&) noexcept = default;

void RuntimeTransport::open() {
  auto& p = *impl_;
  if (p.opened || p.stopped) throw std::logic_error("runtime transport cannot be opened again");
  try {
    // Establish clock admission first; open returns only after every PRD peer is ready.
    natsOptions* options = nullptr;
    nats_check(natsOptions_Create(&options), "create NATS options");
    const std::unique_ptr<natsOptions, decltype(&natsOptions_Destroy)> options_guard(options, &natsOptions_Destroy);
    nats_check(natsOptions_SetURL(options, p.nats_url.c_str()), "set NATS URL");
    nats_check(natsOptions_SetTimeout(options, static_cast<int>(p.op_timeout.count())), "set NATS timeout");
    nats_check(natsOptions_SetMaxReconnect(options, 0), "disable NATS reconnect");
    nats_check(natsConnection_Connect(&p.connection, options), "connect NATS " + p.nats_url);
    if (p.max_messages > INT_MAX || p.max_bytes > INT_MAX)
      throw std::invalid_argument("NATS pending limits exceed libnats integer range");
    const std::string& subscription_subject = p.event_subject.empty() ? p.clock_subject : p.event_subject;
    nats_check(natsConnection_SubscribeSync(&p.subscription, p.connection, subscription_subject.c_str()),
               "subscribe NATS runtime events " + subscription_subject);
    nats_check(natsSubscription_SetPendingLimits(p.subscription, static_cast<int>(p.max_messages),
               static_cast<int>(std::min<std::size_t>(p.max_bytes, static_cast<std::size_t>(INT_MAX)))),
               "set NATS runtime pending limits");
    nats_check(natsConnection_FlushTimeout(p.connection, static_cast<int>(p.op_timeout.count())),
               "flush NATS runtime subscription " + subscription_subject);
    asio::ip::udp::resolver udp_resolver(p.io);
    const auto udp_endpoints = udp_resolver.resolve(asio::ip::udp::v4(), p.host, std::to_string(p.dis_port));
    if (udp_endpoints.empty()) throw std::runtime_error("resolve DIS host " + p.host + ": no UDP endpoint");
    p.dis_endpoint = *udp_endpoints.begin();
    p.udp.open(p.dis_endpoint.protocol());
    for (const auto& entry : p.reports) {
      const auto& route = entry.second;
      if (p.peers.find(route.url) == p.peers.end()) {
        auto peer = std::make_shared<Impl::Peer>(p, route.url, p.service);
        p.peers.emplace(route.url, peer);
        peer->connect();
      }
    }
    p.thread = std::thread([&p] {
      try { p.io.run(); }
      catch (const std::exception& error) { p.fail(std::string("runtime transport IO thread: ") + error.what()); }
      catch (...) { p.fail("runtime transport IO thread: unknown exception"); }
    });
    std::unique_lock<std::mutex> lock(p.mutex);
    const bool complete = p.cv.wait_for(lock, p.op_timeout, [&] {
      return !p.failure.empty() || p.ready_peers == p.peers.size();
    });
    if (!complete && p.failure.empty()) p.failure = "OWP startup timed out";
    if (!p.failure.empty()) throw std::runtime_error(p.failure);
    p.opened = true;
  } catch (const std::exception& error) {
    p.stop();
    throw std::runtime_error(std::string("runtime transport open: ") + error.what());
  }
}

std::optional<RuntimeEvent> RuntimeTransport::next_event() {
  auto& p = *impl_; p.check_failure();
  if (!p.opened) throw std::logic_error("runtime transport is not open");
  const auto deadline = std::chrono::steady_clock::now() + p.poll_timeout;
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      p.check_failure();
      return std::nullopt;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    if (remaining.count() <= 0) {
      p.check_failure();
      return std::nullopt;
    }
    const auto timeout_ms = static_cast<int>(remaining.count());
    natsMsg* raw_message = nullptr;
    const auto status = natsSubscription_NextMsg(&raw_message, p.subscription,
                                                  timeout_ms);
    if (status == NATS_TIMEOUT) { p.check_failure(); return std::nullopt; }
    nats_check(status, "receive NATS runtime event");
    const std::unique_ptr<natsMsg, decltype(&natsMsg_Destroy)> message(raw_message, &natsMsg_Destroy);
    const std::string subject = natsMsg_GetSubject(message.get());
    RuntimeEventKind kind;
    if (subject == p.clock_subject) kind = RuntimeEventKind::clock;
    else if (!p.checkpoint_request_subject.empty() && subject == p.checkpoint_request_subject)
      kind = RuntimeEventKind::checkpoint_request;
    else {
      // The prefix subscription may admit unrelated subjects. Discard them
      // before copying data, while keeping the original poll deadline fixed.
      p.check_failure();
      continue;
    }
    const int length = natsMsg_GetDataLength(message.get());
    const auto* data = natsMsg_GetData(message.get());
    RuntimeEvent event{kind, {}};
    if (length > 0) event.payload.assign(data, data + length);
    p.check_failure();
    return event;
  }
}

std::optional<std::vector<std::uint8_t>> RuntimeTransport::next_clock() {
  auto event = next_event();
  if (!event) return std::nullopt;
  if (event->kind != RuntimeEventKind::clock)
    throw std::runtime_error("checkpoint request received by clock-only runtime caller");
  return std::move(event->payload);
}

void RuntimeTransport::publish_checkpoint_response(const std::vector<std::uint8_t>& payload) {
  auto& p = *impl_;
  p.check_failure();
  if (!p.opened) throw std::logic_error("runtime transport is not open");
  if (p.checkpoint_response_subject.empty())
    throw std::logic_error("checkpoint response transport is not configured");
  if (payload.size() > static_cast<std::size_t>(INT_MAX))
    throw std::length_error("checkpoint response exceeds libnats payload range");
  nats_check(natsConnection_Publish(p.connection, p.checkpoint_response_subject.c_str(),
             payload.data(), static_cast<int>(payload.size())),
             "publish NATS checkpoint response " + p.checkpoint_response_subject);
  nats_check(natsConnection_FlushTimeout(p.connection, static_cast<int>(p.op_timeout.count())),
             "flush NATS checkpoint response " + p.checkpoint_response_subject);
  p.check_failure();
}

checkpoint_runtime::DeliveryReceipt RuntimeTransport::dis_delivery_receipt() const {
  if (!impl_->dis_receipts) return {};
  return impl_->dis_receipts->snapshot();
}

void RuntimeTransport::reset_delivery_receipts() {
  if (impl_->dis_receipts) impl_->dis_receipts->reset();
}

void RuntimeTransport::publish(const std::vector<RuntimeOutput>& outputs) {
  auto& p = *impl_; p.check_failure();
  if (!p.opened) throw std::logic_error("runtime transport is not open");
  for (const auto& output : outputs) {
    if (output.kind == SampleKind::physical) {
      const auto sent = p.udp.send_to(asio::buffer(output.dis), p.dis_endpoint);
      if (sent != output.dis.size()) throw std::runtime_error("short DIS UDP send to " + p.host);
      if (p.dis_receipts) {
        // Count only complete local UDP sends, preserving the exact OpenDIS payload bytes.
        p.dis_receipts->observe(std::string_view(
            reinterpret_cast<const char*>(output.dis.data()), output.dis.size()));
      }
      continue;
    }
    const auto route = p.reports.find(output.entity_id);
    if (route == p.reports.end()) throw std::runtime_error("PRD output has no configured route for " + output.entity_id);
    p.peers.at(route->second.url)->enqueue({route->second.topic, output.prd.dump()});
  }
  p.check_failure();
}

void RuntimeTransport::flush() {
  auto& p = *impl_;
  if (!p.opened) throw std::logic_error("runtime transport is not open");
  std::unique_lock<std::mutex> lock(p.mutex);
  const auto done = p.cv.wait_for(lock, p.op_timeout, [&] {
    return !p.failure.empty() || p.pending_messages.load() == 0;
  });
  if (!done) throw std::runtime_error("OWP flush timed out");
  if (!p.failure.empty()) throw std::runtime_error(p.failure);
}

}  // namespace flight_models
